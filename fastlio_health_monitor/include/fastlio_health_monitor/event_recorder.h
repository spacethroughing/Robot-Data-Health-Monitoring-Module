#pragma once
#include <fastlio_health_monitor/health_core.h>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <mutex>
#include <thread>

namespace robot_health {
std::string jsonString(const std::string& value);
std::string jsonValues(const Values& values);
std::string utcNow();
std::string sha256File(const std::string& path);
std::string sha256Text(const std::string& value);
// Optional replay time provider. The core logger stays independent of ROS.
struct LogTimeline {
  uint64_t begin_ns = 0, end_ns = 0;
  std::function<uint64_t()> current_ns;
};
class EventRecorder {
 public:
  explicit EventRecorder(const MonitorConfig& config,
                         const LogTimeline& timeline = {});
  ~EventRecorder();
  bool event(const std::string& type, const std::string& module,
             const std::string& instance, uint64_t epoch,
             const Finding& finding, int before, int after,
             const Values& context = {}, bool durable = false);
  void metrics(const ModuleReport& report, double now);
  // Drain, flush, close once. Intentional calls after finish are ignored,
  // without being reported as dropped records or reopening the run.
  bool finish(const std::string& reason, const Values& context = {});
  bool finished() const { return finished_; }
  bool healthy() const { return !failed_ && dropped_events_ == 0; }
  bool enabled() const { return config_.logging; }
  Values status() const;
  const std::string& directory() const { return directory_; }
  const std::string& runId() const { return run_id_; }

 private:
  struct Item {
    uint64_t sequence;
    bool durable;
    std::string text;
  };
  void worker();
  bool enqueueEvent(const std::string& type, const std::string& module,
                    const std::string& instance, uint64_t epoch,
                    const Finding& finding, int before, int after,
                    const Values& context, bool durable,
                    std::unique_lock<std::mutex>& lock);
  void rotate(std::ofstream& stream, const std::string& stem, unsigned& index,
              size_t& bytes, bool csv);
  MonitorConfig config_;
  LogTimeline timeline_;
  std::string run_id_, config_hash_, directory_, data_hash_;
  double started_;
  mutable std::mutex mutex_;
  std::mutex finish_mutex_;
  std::condition_variable available_, committed_;
  std::deque<Item> events_;
  std::deque<std::string> metrics_;
  std::thread writer_;
  bool stopping_ = false;
  std::atomic<bool> finished_{false};
  std::string finish_reason_;
  uint64_t sequence_ = 0, committed_sequence_ = 0;
  std::atomic<bool> failed_{false};
  std::atomic<uint64_t> dropped_events_{0}, dropped_metrics_{0},
      rotated_files_{0};
};
}  // namespace robot_health
