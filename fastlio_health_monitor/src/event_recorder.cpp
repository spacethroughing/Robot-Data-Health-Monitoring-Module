/**
 * 日志实现：事件/指标排队 -> writer线程序列化落盘 -> 关键事件fsync -> EOF/退出封存。
 * 用户删除的字段在统一JSON出口过滤，CSV也移除实例/代际列；历史日志不改写。
 */
#include <dlfcn.h>
#include <fastlio_health_monitor/event_recorder.h>
#include <fcntl.h>
#include <openssl/sha.h>
#include <sys/utsname.h>
#include <unistd.h>

#include <boost/filesystem.hpp>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "source_build_manifest.h"

namespace robot_health {
// 正确转义引号、反斜杠及控制字符，保证事件JSONL逐行可被解析。
std::string jsonString(const std::string& v) {
  std::ostringstream s;
  s << '"';
  for (unsigned char c : v) {
    switch (c) {
      case '"':
        s << "\\\"";
        break;
      case '\\':
        s << "\\\\";
        break;
      case '\n':
        s << "\\n";
        break;
      case '\r':
        s << "\\r";
        break;
      case '\t':
        s << "\\t";
        break;
      default:
        if (c < 32)
          s << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c);
        else
          s << c;
    }
  }
  s << '"';
  return s.str();
}
// 把键值上下文编码为JSON，并在统一出口排除本版删除的日志字段。
std::string jsonValues(const Values& v) {
  std::string s = "{";
  bool first = true;
  for (const auto& p : v) {
    // 唯一日志出口过滤：防止上下文或后续新增调用把已删除字段重新写回。
    if (p.first == "producer_instance_id" || p.first == "module_epoch" ||
        p.first == "metric" || p.first == "effective_config_sha256" ||
        p.first == "data_sha256" || p.first == "epoch") continue;
    if (!first) s += ',';
    first = false;
    s += jsonString(p.first) + ":" + jsonString(p.second);
  }
  return s + "}";
}
// 读取UTC墙钟文字，仅供静态辅助信息使用；事件记录时间仍只用用户指定的三项。
std::string utcNow() {
  auto now = std::chrono::system_clock::now();
  auto t = std::chrono::system_clock::to_time_t(now);
  std::tm tm;
  gmtime_r(&t, &tm);
  char buf[40];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return buf;
}
// 对源码/配置文字做摘要以辅助版本追溯；本版不输出effective_config_sha256字段。
std::string sha256Text(const std::string& v) {
  unsigned char d[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char*>(v.data()), v.size(), d);
  std::ostringstream out;
  for (auto c : d)
    out << std::hex << std::setw(2) << std::setfill('0') << int(c);
  return out.str();
}
// 流式计算程序或核心库文件摘要；本版不再为了日志启动而读取整个bag做哈希。
std::string sha256File(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return "unavailable";
  SHA256_CTX ctx;
  SHA256_Init(&ctx);
  char b[65536];
  while (in) {
    in.read(b, sizeof(b));
    if (in.gcount()) SHA256_Update(&ctx, b, in.gcount());
  }
  unsigned char d[SHA256_DIGEST_LENGTH];
  SHA256_Final(d, &ctx);
  std::ostringstream out;
  for (auto c : d)
    out << std::hex << std::setw(2) << std::setfill('0') << int(c);
  return out.str();
}
namespace {
// 转义CSV中的逗号、引号和换行，使一行指标可以正确导入表格。
std::string csv(const std::string& text) {
  std::string s = "\"";
  for (char c : text) {
    if (c == '"') s += '"';
    s += c;
  }
  return s + '"';
}
const char* header =
    "bag_elapsed_sec,source_stamp_sec,steady_elapsed_sec,module_id,"
    "operating_state,health_level,operational_valid,values_json\n";
// Source nanoseconds remain internal for exact decimal formatting. Only the
// three selected time fields are serialized; diagnostic durations stay intact.
// 从日志上下文中去掉重复的源时间字段；精确纳秒只在内部用于格式化。
Values loggedValues(Values values) {
  values.erase("last_stamp_sec");
  values.erase("source_stamp_ns");
  values.erase("source_stamp_sec");
  return values;
}
// 使用整数拆分纳秒为秒和9位小数，避免Unix大时间戳被低精度格式化。
std::string nsSeconds(uint64_t ns) {
  std::ostringstream out;
  out << ns / 1000000000ULL << '.' << std::setw(9) << std::setfill('0')
      << ns % 1000000000ULL;
  return out.str();
}
struct BagFields {
  bool valid = false;
  std::string elapsed;
};
// 从可选时间线读取/clock并计算相对bag起点秒数；未就绪/超边界时记录null。
BagFields bagFields(const LogTimeline& timeline) {
  BagFields fields;
  if (!timeline.current_ns) return fields;
  const auto ns = timeline.current_ns();
  if (!ns || ns < timeline.begin_ns || ns > timeline.end_ns) return fields;
  fields.valid = true;
  fields.elapsed = nsSeconds(ns - timeline.begin_ns);
  return fields;
}
// 优先用适配器保存的精确源时间戳；无关联样本时返回null，不拿当前时间顶替。
std::pair<std::string, std::string> sourceFields(const Values& values) {
  auto ns = values.find("source_stamp_ns");
  if (ns != values.end()) {
    try {
      size_t used = 0;
      const uint64_t value = std::stoull(ns->second, &used);
      if (used == ns->second.size() && !ns->second.empty() && ns->second[0] != '-')
        return {nsSeconds(value), ns->second};
    } catch (const std::exception&) {}
  }
  auto seconds = values.find("source_stamp_sec");
  if (seconds != values.end()) {
    try {
      size_t used = 0;
      const double value = std::stod(seconds->second, &used);
      if (used == seconds->second.size() && std::isfinite(value)) {
        std::ostringstream out;
        out << std::setprecision(17) << value;
        return {out.str(), ""};
      }
    } catch (const std::exception&) {}
  }
  return {"null", ""};
}
// 执行fsync确保关键记录交给系统持久化；失败会进入日志异常状态。
bool syncFile(const std::string& path) {
  int fd = open(path.c_str(), O_RDONLY | O_CLOEXEC);
  if (fd < 0) return false;
  bool ok = fsync(fd) == 0;
  close(fd);
  return ok;
}
}  // namespace
// 建立本次run目录、保存配置/程序版本，启动单写入线程，提交RUN_BEGIN。
EventRecorder::EventRecorder(const MonitorConfig& c, const LogTimeline& timeline)
    : config_(c),
      timeline_(timeline),
      run_id_(uniqueId()),
      started_(steadyNow()) {
  if (!c.logging) return;
  try {
    directory_ = (boost::filesystem::path(c.log_directory) / run_id_).string();
    boost::filesystem::create_directories(directory_);
    auto write = [&](const std::string& name, const std::string& text) {
      std::ofstream out(directory_ + "/" + name);
      out << text;
      out.flush();
      if (!out) throw std::runtime_error("cannot write " + name);
    };
    write("config_source.yaml", c.source_text);
    write("effective_config.yaml", c.effective_text);
    char exe[4096];
    auto len = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
    if (len > 0) exe[len] = 0;
    Values manifest{
        {"log_schema_version", "3"},
        {"monitor_run_id", run_id_},
        {"config_source_path", c.source_path},
        {"config_source_sha256", sha256Text(c.source_text)},
        {"binary_sha256", len > 0 ? sha256File(exe) : "unavailable"},
        {"binary_path", len > 0 ? exe : "unavailable"},
        {"time_policy",
         "bag_elapsed_sec from /clock minus bag start; source_stamp_sec from header; steady_elapsed_sec from recorder start"},
        {"source_build_manifest",
         "source_build_manifest.json (compiled source hashes and Git "
         "state)"}};
    Dl_info library{};
    if (dladdr(reinterpret_cast<void*>(&steadyNow), &library) &&
        library.dli_fname)
      manifest["core_library_sha256"] = sha256File(library.dli_fname);
    char hostname[256]{};
    if (gethostname(hostname, sizeof(hostname) - 1) == 0)
      manifest["hostname"] = hostname;
    struct utsname system {};
    if (uname(&system) == 0) manifest["kernel"] = system.release;
    manifest["compiler"] = __VERSION__;
    manifest["data_reference"] = c.data_reference;
    manifest["bag_time_available"] = timeline_.current_ns ? "true" : "false";
    manifest["bag_start_time_sec"] = timeline_.current_ns ? nsSeconds(timeline_.begin_ns) : "unavailable";
    manifest["bag_end_time_sec"] = timeline_.current_ns ? nsSeconds(timeline_.end_ns) : "unavailable";
    manifest["log_time_fields"] = "bag_elapsed_sec,source_stamp_sec,steady_elapsed_sec";
    write("manifest.json", jsonValues(manifest));
    write("source_build_manifest.json", robot_health_source_manifest);
    writer_ = std::thread(&EventRecorder::worker, this);
    Finding f;
    f.id = "RUN";
    f.reason = "monitor started";
    event("RUN_BEGIN", "monitor", "", 0, f, 0, 0, {}, true);
  } catch (const std::exception& e) {
    failed_ = true;
    std::cerr << "LOG_WRITE_FAILED: " << e.what() << std::endl;
  }
}
// 析构时幂等封存日志，确保正常关节点仍写RUN_END并排空队列。
EventRecorder::~EventRecorder() {
  finish("node shutdown");
}
// 提交唯一RUN_END，等待写线程排空、flush/fsync并关闭；结束后不再追加。
bool EventRecorder::finish(const std::string& reason, const Values& context) {
  std::lock_guard<std::mutex> finish_lock(finish_mutex_);
  if (finished_) return healthy();
  bool queued = true;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    finish_reason_ = reason;
    stopping_ = true;
    if (writer_.joinable()) {
      Finding f;
      f.id = "RUN";
      f.reason = reason;
      queued = enqueueEvent("RUN_END", "monitor", "", 0, f, 0, 0,
                            context, true, lock);
    }
  }
  available_.notify_all();
  if (writer_.joinable()) writer_.join();
  finished_ = true;
  return queued && healthy();
}
// 线程安全的事件入口；将业务线程与磁盘写入分离，封存后忽略追加。
bool EventRecorder::event(const std::string& type, const std::string& module,
                          const std::string& instance, uint64_t epoch,
                          const Finding& f, int before, int after,
                          const Values& context, bool durable) {
  if (!config_.logging) return false;
  std::unique_lock<std::mutex> lock(mutex_);
  if (stopping_ || finished_) return healthy();
  return enqueueEvent(type, module, instance, epoch, f, before, after,
                      context, durable, lock);
}
// 生成三时间字段事件、删除精简字段、进入关键队列；必要时等待持久化确认。
bool EventRecorder::enqueueEvent(const std::string& type, const std::string& module,
                                const std::string& instance, uint64_t epoch,
                                const Finding& f, int before, int after,
                                const Values& context, bool durable,
                                std::unique_lock<std::mutex>& lock) {
  (void)instance;
  (void)epoch;
  if (failed_ || !writer_.joinable()) {
    ++dropped_events_;
    return false;
  }
  uint64_t seq = ++sequence_;
  const auto bag = bagFields(timeline_);
  const auto source = sourceFields(context);
  std::string value = std::isfinite(f.value) ? number(f.value) : "null";
  std::string row =
      "{\"event_seq\":" + std::to_string(seq) +
      ",\"monitor_run_id\":" + jsonString(run_id_) +
      ",\"bag_elapsed_sec\":" + (bag.valid ? bag.elapsed : "null") +
      ",\"source_stamp_sec\":" + source.first +
      ",\"steady_elapsed_sec\":" + number(steadyNow() - started_) +
      ",\"event_type\":" + jsonString(type) +
      ",\"module_id\":" + jsonString(module) +
      ",\"rule_id\":" + jsonString(f.id) +
      ",\"value\":" + value +
      ",\"raw_value_text\":" +
      jsonString(std::isfinite(f.value)
                     ? number(f.value)
                     : (std::isnan(f.value) ? "NaN" : "Inf")) +
      ",\"unit\":" + jsonString(f.unit) +
      ",\"threshold\":" + (std::isfinite(f.limit) ? number(f.limit) : "null") +
      ",\"from_level\":" + jsonString(levelName(before)) +
      ",\"to_level\":" + jsonString(levelName(after)) +
      ",\"reason\":" + jsonString(f.reason) +
      ",\"context\":" + jsonValues(loggedValues(context)) + "}\n";
  if (events_.size() >= config_.event_capacity) {
    ++dropped_events_;
    committed_.notify_all();
    return false;
  }
  events_.push_back(Item{seq, durable, std::move(row)});
  available_.notify_one();
  if (durable) {
    if (!committed_.wait_for(lock, std::chrono::seconds(2), [&] {
          return failed_ || committed_sequence_ >= seq;
        })) {
      failed_ = true;
      return false;
    }
    return !failed_;
  }
  return true;
}
// 定期把模块报告写入CSV队列；去掉实例/代际列及嵌套旧字段，保留有用指标值。
void EventRecorder::metrics(const ModuleReport& r, double now) {
  if (!config_.logging || failed_) return;
  std::lock_guard<std::mutex> lock(mutex_);
  if (stopping_ || finished_) return;
  const auto bag = bagFields(timeline_);
  const auto source = sourceFields(r.values);
  std::ostringstream out;
  out << bag.elapsed << ',' << (source.first == "null" ? "" : source.first)
      << ',' << number(now - started_) << ',' << csv(r.id)
      << ',' << csv(r.state) << ','
      << r.level << ',' << (r.valid ? "true" : "false") << ','
      << csv(jsonValues(loggedValues(r.values))) << '\n';
  if (metrics_.size() >= config_.metrics_capacity) {
    ++dropped_metrics_;
    return;
  }
  metrics_.push_back(out.str());
  available_.notify_one();
}
// 提供日志健康、目录、丢弃计数、封存状态；供总体诊断和界面显示。
Values EventRecorder::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  const bool incomplete =
      failed_ || dropped_events_ || dropped_metrics_ || rotated_files_;
  return {{"data_reference", config_.data_reference},
          {"monitor_run_id", run_id_},
          {"log_directory", directory_},
          {"logs_enabled", config_.logging ? "true" : "false"},
          {"logs_recording", config_.logging && !stopping_ && !failed_ ? "true" : "false"},
          {"logs_finished", finished_ ? "true" : "false"},
          {"log_recording_state", !config_.logging ? "DISABLED" : failed_ ? "FAILED" : finished_ ? "FINISHED" : stopping_ ? "FINISHING" : "RECORDING"},
          {"log_finish_reason", finish_reason_},
          {"logs_normal", config_.logging && !incomplete ? "true" : "false"},
          {"logs_incomplete", incomplete ? "true" : "false"},
          {"log_write_failed", failed_ ? "true" : "false"},
          {"dropped_events", std::to_string(dropped_events_)},
          {"dropped_metrics", std::to_string(dropped_metrics_)},
          {"rotated_files_removed", std::to_string(rotated_files_)}};
}
// 按大小切换新文件，超保留数量删除最老文件并明确标记日志不完整。
void EventRecorder::rotate(std::ofstream& out, const std::string& stem,
                           unsigned& index, size_t& bytes, bool is_csv) {
  out.close();
  ++index;
  bytes = 0;
  std::string suffix = is_csv ? ".csv" : ".jsonl";
  out.open(directory_ + "/" + stem + "." + std::to_string(index) + suffix,
           std::ios::app);
  if (is_csv) out << header;
  if (index >= config_.rotate_count) {
    boost::filesystem::remove(directory_ + "/" + stem + "." +
                              std::to_string(index - config_.rotate_count) +
                              suffix);
    ++rotated_files_;
  }
}
// 唯一磁盘写线程：关键事件优先、指标后写，检查预算/写失败，最终flush/fsync封存。
void EventRecorder::worker() {
  try {
    std::ofstream events(directory_ + "/events.0.jsonl"),
        metrics(directory_ + "/metrics.0.csv");
    metrics << header;
    unsigned ei = 0, mi = 0;
    size_t eb = 0, mb = 0, total = 0;
    double last_flush = steadyNow();
    for (;;) {
      Item event_item{0, false, ""};
      std::string metric;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        available_.wait_for(lock, std::chrono::milliseconds(100), [&] {
          return stopping_ || !events_.empty() || !metrics_.empty();
        });
        if (events_.empty() && metrics_.empty() && stopping_) break;
        if (!events_.empty()) {
          event_item = std::move(events_.front());
          events_.pop_front();
        } else if (!metrics_.empty()) {
          metric = std::move(metrics_.front());
          metrics_.pop_front();
        }
      }
      if (event_item.sequence) {
        if (eb + event_item.text.size() > config_.rotate_bytes)
          rotate(events, "events", ei, eb, false);
        events << event_item.text;
        eb += event_item.text.size();
        total += event_item.text.size();
      }
      if (!metric.empty()) {
        if (mb + metric.size() > config_.rotate_bytes)
          rotate(metrics, "metrics", mi, mb, true);
        metrics << metric;
        mb += metric.size();
        total += metric.size();
      }
      if (total > config_.budget_bytes)
        throw std::runtime_error("run log write budget exceeded");
      if (event_item.durable ||
          steadyNow() - last_flush >= config_.flush_period) {
        events.flush();
        metrics.flush();
        last_flush = steadyNow();
      }
      if (!events || !metrics)
        throw std::runtime_error("log stream write failed");
      if (event_item.durable &&
          !syncFile(directory_ + "/events." + std::to_string(ei) + ".jsonl"))
        throw std::runtime_error("critical fsync failed");
      if (event_item.durable) {
        std::lock_guard<std::mutex> lock(mutex_);
        committed_sequence_ = event_item.sequence;
        committed_.notify_all();
      }
    }
    events.flush();
    metrics.flush();
    if (!events || !metrics) throw std::runtime_error("final log flush failed");
    if (!syncFile(directory_ + "/events." + std::to_string(ei) + ".jsonl") ||
        !syncFile(directory_ + "/metrics." + std::to_string(mi) + ".csv"))
      throw std::runtime_error("final log fsync failed");
  } catch (const std::exception& e) {
    failed_ = true;
    committed_.notify_all();
    std::cerr << "LOG_WRITE_FAILED: " << e.what() << std::endl;
  }
}
}  // namespace robot_health
