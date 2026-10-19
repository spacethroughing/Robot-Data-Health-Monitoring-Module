/**
 * 可追溯运行日志的公共接口。
 *
 * 数据流：健康规则变化/模块报告 -> EventRecorder入队 -> 专用写线程 -> JSONL事件
 * 和CSV指标文件。健康检测线程不直接执行普通磁盘写入，避免日志I/O拖住消息回调。
 * 关键启停事件可选择等待写入确认；队列溢出、磁盘失败和日志封存状态可由status()
 * 查询并反馈到诊断。
 *
 * 每条事件只使用bag_elapsed_sec、source_stamp_sec、steady_elapsed_sec三种时间。
 * finish()排空队列、关闭文件并封存本次run；封存后忽略新日志，诊断节点仍可继续发布。
 */
#pragma once
#include <fastlio_health_monitor/health_core.h>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <mutex>
#include <thread>

namespace robot_health {
// 将UTF-8文本转为可放入JSON字符串的转义形式，处理引号、反斜线和控制字符。
std::string jsonString(const std::string& value);
// 将键值表序列化为JSON对象；用于日志context和运行清单。
std::string jsonValues(const Values& values);
// 生成UTC墙上时间文本，用于人类查看的运行元数据；不作为回放相对时间。
std::string utcNow();
// 计算文件SHA-256，用来记录配置或可执行文件身份；失败时由实现报告不可用。
std::string sha256File(const std::string& path);
// 计算内存文本SHA-256，例如记录本次实际加载配置内容的摘要。
std::string sha256Text(const std::string& value);
/**
 * 可选回放时间接口，将ROS相关时间读取与通用日志核心分开。
 * begin_ns/end_ns是bag首末记录的纳秒时间；current_ns返回当前/clock时间纳秒值。
 * 记录器用当前时间减bag起点生成bag_elapsed_sec。实机或时间不可用时不提供回调，
 * 对应日志时间留空，不会因此影响source_stamp_sec和steady_elapsed_sec。
 */
struct LogTimeline {
  uint64_t begin_ns = 0, end_ns = 0;
  std::function<uint64_t()> current_ns;
};
/**
 * 单次运行的线程安全日志记录器。
 * 构造时创建唯一run目录并保存源配置/有效配置/程序清单，然后启动写线程。
 * event()记录离散状态变化；metrics()记录周期性模块快照；finish()写入结束事件、
 * 等待待写队列排空并封存文件。对象析构也会安全调用finish()。
 * 事件和指标队列有界，容量来自MonitorConfig；可通过healthy()/status()发现写入失败。
 */
class EventRecorder {
 public:
  // 创建本次运行日志；timeline仅用于可选的bag_elapsed_sec时间字段。
  explicit EventRecorder(const MonitorConfig& config,
                         const LogTimeline& timeline = {});
  // 析构前封存日志；finish()可重复调用，不会重复写结束记录。
  ~EventRecorder();
  /**
   * 提交一条规则/控制事件。
   * type是事件类型，module是模块ID，finding包含规则等级和值，before/after是状态变化；
   * context附加可追溯数据。durable=true时等待写线程确认提交，适合启停等关键事件。
   * 返回false表示日志关闭、写入失败、队列满或关键事件未能确认。
   */
  bool event(const std::string& type, const std::string& module,
             const std::string& instance, uint64_t epoch,
             const Finding& finding, int before, int after,
             const Values& context = {}, bool durable = false);
  // 把一个模块的本轮报告加入CSV指标队列；调用方负责按metrics_period_sec节流。
  void metrics(const ModuleReport& report, double now);
  /**
   * 封存本次run：写RUN_END、排空事件/指标队列、flush并关闭文件。
   * @param reason 结束原因，例如bag正常播完或节点关闭。
   * @param context 可附加最终统计信息。
   * 返回封存及日志健康结果；重复调用保持幂等，封存后的event/metrics不会重开文件。
   */
  bool finish(const std::string& reason, const Values& context = {});
  // 查询是否已完成封存；该状态只描述日志文件，不代表健康节点已退出。
  bool finished() const { return finished_; }
  // 查询是否发生写盘失败或关键事件丢失；普通指标丢弃数也会由status()给出。
  bool healthy() const { return !failed_ && dropped_events_ == 0; }
  // 查询配置是否启用日志；false时记录器不创建run目录，也不写文件。
  bool enabled() const { return config_.logging; }
  // 获取当前日志状态与计数，供诊断发布或界面展示。
  Values status() const;
  // 获取本次run目录的绝对/配置解析后路径；构造失败时可能为空。
  const std::string& directory() const { return directory_; }
  // 获取本次run的唯一标识，用于关联文件名和事件。
  const std::string& runId() const { return run_id_; }

 private:
  // 写线程内部的单条事件；sequence用于确认关键记录是否已提交。
  struct Item {
    uint64_t sequence;
    bool durable;
    std::string text;
  };
  // 消费队列、轮转文件并按配置周期flush；只由writer_线程执行。
  void worker();
  // 构造JSON事件并入队；调用者持有mutex_，durable事件会等待提交确认。
  bool enqueueEvent(const std::string& type, const std::string& module,
                    const std::string& instance, uint64_t epoch,
                    const Finding& finding, int before, int after,
                    const Values& context, bool durable,
                    std::unique_lock<std::mutex>& lock);
  // 按单文件大小轮转events或metrics文件，并维护序号、字节数和轮转统计。
  void rotate(std::ofstream& stream, const std::string& stem, unsigned& index,
              size_t& bytes, bool csv);
  // 本次运行不可变的参数快照；写线程读取以确定目录、队列容量和轮转策略。
  MonitorConfig config_;
  // 可空的bag时钟提供器；不持有ROS对象，调用回调取当前回放时间。
  LogTimeline timeline_;
  // 单次运行标识、日志目录和steady时钟起点，用于生成日志字段。
  std::string run_id_, directory_;
  double started_;
  // mutex_保护日志队列和关闭状态；finish_mutex_使并发封存请求串行执行。
  mutable std::mutex mutex_;
  std::mutex finish_mutex_;
  // 通知写线程有新记录；committed_用于唤醒等待durable事件确认的线程。
  std::condition_variable available_, committed_;
  // 离散事件和周期指标分开排队，容量分别由MonitorConfig控制。
  std::deque<Item> events_;
  std::deque<std::string> metrics_;
  // 唯一后台写线程；stopping_/finished_标记关闭进度，队列排空后退出。
  std::thread writer_;
  bool stopping_ = false;
  std::atomic<bool> finished_{false};
  std::string finish_reason_;
  // 序号和写入确认位置；用于durable事件等待其已写入。
  uint64_t sequence_ = 0, committed_sequence_ = 0;
  // 原子健康标志与丢弃/轮转计数，可跨检测和写线程读取。
  std::atomic<bool> failed_{false};
  std::atomic<uint64_t> dropped_events_{0}, dropped_metrics_{0},
      rotated_files_{0};
};
}  // namespace robot_health
