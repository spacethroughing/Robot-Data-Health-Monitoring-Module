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
std::string jsonValues(const Values& v) {
  std::string s = "{";
  bool first = true;
  for (const auto& p : v) {
    if (!first) s += ',';
    first = false;
    s += jsonString(p.first) + ":" + jsonString(p.second);
  }
  return s + "}";
}
std::string utcNow() {
  auto now = std::chrono::system_clock::now();
  auto t = std::chrono::system_clock::to_time_t(now);
  std::tm tm;
  gmtime_r(&t, &tm);
  char buf[40];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return buf;
}
std::string sha256Text(const std::string& v) {
  unsigned char d[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char*>(v.data()), v.size(), d);
  std::ostringstream out;
  for (auto c : d)
    out << std::hex << std::setw(2) << std::setfill('0') << int(c);
  return out.str();
}
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
std::string csv(const std::string& text) {
  std::string s = "\"";
  for (char c : text) {
    if (c == '"') s += '"';
    s += c;
  }
  return s + '"';
}
const char* header =
    "wall_time_utc,steady_elapsed_sec,module_id,producer_instance_id,epoch,"
    "operating_state,health_level,operational_valid,metrics_json\n";
bool syncFile(const std::string& path) {
  int fd = open(path.c_str(), O_RDONLY | O_CLOEXEC);
  if (fd < 0) return false;
  bool ok = fsync(fd) == 0;
  close(fd);
  return ok;
}
}  // namespace
EventRecorder::EventRecorder(const MonitorConfig& c)
    : config_(c),
      run_id_(uniqueId()),
      config_hash_(sha256Text(c.effective_text)),
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
        {"monitor_run_id", run_id_},
        {"wall_time_utc", utcNow()},
        {"config_source_path", c.source_path},
        {"config_source_sha256", sha256Text(c.source_text)},
        {"effective_config_sha256", config_hash_},
        {"binary_sha256", len > 0 ? sha256File(exe) : "unavailable"},
        {"binary_path", len > 0 ? exe : "unavailable"},
        {"time_policy",
         "steady watchdog; source derivatives; UTC presentation"},
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
    data_hash_ =
        c.data_reference.empty() ? "unspecified" : sha256File(c.data_reference);
    manifest["data_sha256"] = data_hash_;
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
EventRecorder::~EventRecorder() {
  if (writer_.joinable()) {
    Finding f;
    f.id = "RUN";
    f.reason = "monitor stopping";
    event("RUN_END", "monitor", "", 0, f, 0, 0, {}, true);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    available_.notify_all();
    writer_.join();
  }
}
bool EventRecorder::event(const std::string& type, const std::string& module,
                          const std::string& instance, uint64_t epoch,
                          const Finding& f, int before, int after,
                          const Values& context, bool durable) {
  if (!config_.logging) return false;
  std::unique_lock<std::mutex> lock(mutex_);
  if (failed_ || !writer_.joinable()) {
    ++dropped_events_;
    return false;
  }
  uint64_t seq = ++sequence_;
  std::string value = std::isfinite(f.value) ? number(f.value) : "null";
  std::string row =
      "{\"event_seq\":" + std::to_string(seq) +
      ",\"monitor_run_id\":" + jsonString(run_id_) +
      ",\"wall_time_utc\":" + jsonString(utcNow()) +
      ",\"steady_elapsed_sec\":" + number(steadyNow() - started_) +
      ",\"event_type\":" + jsonString(type) +
      ",\"module_id\":" + jsonString(module) +
      ",\"producer_instance_id\":" + jsonString(instance) +
      ",\"module_epoch\":" + std::to_string(epoch) +
      ",\"rule_id\":" + jsonString(f.id) +
      ",\"metric\":" + jsonString(f.metric) + ",\"value\":" + value +
      ",\"raw_value_text\":" +
      jsonString(std::isfinite(f.value)
                     ? number(f.value)
                     : (std::isnan(f.value) ? "NaN" : "Inf")) +
      ",\"unit\":" + jsonString(f.unit) +
      ",\"threshold\":" + (std::isfinite(f.limit) ? number(f.limit) : "null") +
      ",\"from_level\":" + jsonString(levelName(before)) +
      ",\"to_level\":" + jsonString(levelName(after)) +
      ",\"reason\":" + jsonString(f.reason) +
      ",\"effective_config_sha256\":" + jsonString(config_hash_) +
      ",\"context\":" + jsonValues(context) + "}\n";
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
void EventRecorder::metrics(const ModuleReport& r, double now) {
  if (!config_.logging || failed_) return;
  std::ostringstream out;
  out << csv(utcNow()) << ',' << number(now - started_) << ',' << csv(r.id)
      << ',' << csv(r.instance) << ',' << r.epoch << ',' << csv(r.state) << ','
      << r.level << ',' << (r.valid ? "true" : "false") << ','
      << csv(jsonValues(r.values)) << '\n';
  std::lock_guard<std::mutex> lock(mutex_);
  if (metrics_.size() >= config_.metrics_capacity) {
    ++dropped_metrics_;
    return;
  }
  metrics_.push_back(out.str());
  available_.notify_one();
}
Values EventRecorder::status() const {
  const bool incomplete =
      failed_ || dropped_events_ || dropped_metrics_ || rotated_files_;
  return {{"data_reference", config_.data_reference},
          {"data_sha256", data_hash_},
          {"monitor_run_id", run_id_},
          {"log_directory", directory_},
          {"logs_enabled", config_.logging ? "true" : "false"},
          {"logs_normal", config_.logging && !incomplete ? "true" : "false"},
          {"logs_incomplete", incomplete ? "true" : "false"},
          {"log_write_failed", failed_ ? "true" : "false"},
          {"dropped_events", std::to_string(dropped_events_)},
          {"dropped_metrics", std::to_string(dropped_metrics_)},
          {"rotated_files_removed", std::to_string(rotated_files_)},
          {"effective_config_sha256", config_hash_}};
}
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
  } catch (const std::exception& e) {
    failed_ = true;
    committed_.notify_all();
    std::cerr << "LOG_WRITE_FAILED: " << e.what() << std::endl;
  }
}
}  // namespace robot_health
