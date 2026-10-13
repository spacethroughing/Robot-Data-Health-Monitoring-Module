#pragma once

#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace robot_health {

enum Level { OK = 0, WARNING = 1, ERROR = 2 };
using Values = std::map<std::string, std::string>;
double steadyNow();
std::string number(double value);
std::string uniqueId();
const char* levelName(int level);

struct Finding {
  std::string id, reason, metric, unit;
  int level = OK;
  double value = 0, limit = 0;
  bool hard = false;
};
using Findings = std::vector<Finding>;
struct Bounds {
  double warning = 0, error = 0;
  bool lower = false;
  int level(double value) const;
};
struct StreamConfig {
  std::string frequency_basis = "source", stamp_domain = "device_relative";
  Bounds age{0.5, 1.5, false}, low_rate{5, 1, true}, high_rate{15, 30, false};
  Bounds gap{0.3, 1.0, false};
  int duplicate_level = WARNING, regression_level = ERROR;
  bool require_positive = true;
};
struct LifecycleConfig {
  std::string binding;
  double startup = 10, stop = 5;
  unsigned ready_samples = 10;
};
struct ModuleConfig {
  std::string id, checker, adapter, topic, frame, child_frame, continuity;
  std::string visual_topic, visual_frame, task_topic;
  bool required = true;
  std::vector<std::string> dependencies;
  StreamConfig stream;
  LifecycleConfig lifecycle;
  YAML::Node quality;
};
struct MonitorConfig {
  double publish_rate = 5, window = 5, frequency_coverage = 1;
  unsigned frequency_samples = 3;
  double soft_hold = 0.5, recovery_hold = 1;
  bool strict_control = false;
  double heartbeat_warning = 1, heartbeat_error = 3;
  std::string state_topic = "/robot_runtime/module_state";
  std::string diagnostics_topic = "/fastlio_health_monitor/diagnostics";
  std::string status_topic = "/fastlio_health_monitor/status";
  bool logging = true;
  std::string log_directory;
  double metrics_period = 1, flush_period = 1;
  size_t metrics_capacity = 4096, event_capacity = 512,
         rotate_bytes = 20 * 1024 * 1024;
  unsigned rotate_count = 10;
  size_t budget_bytes = 512 * 1024 * 1024;
  std::vector<ModuleConfig> modules;
  std::string source_path, source_text, effective_text;
  std::string data_reference;
};
class ConfigLoader {
 public:
  static MonitorConfig load(const std::string& path);
};

struct Sample {
  double stamp = 0, received = 0;
  bool valid = true, covariance_available = true;
  std::string frame, child_frame;
  uint64_t total = 0, valid_points = 0;
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d twist = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
  Findings findings;
  Values values;
};

class StreamTracker {
 public:
  StreamTracker(const StreamConfig& config, const MonitorConfig& monitor);
  Findings receive(double stamp, double received);
  Findings evaluate(double now, Values& values, bool& ready);
  void reset(double now);
  bool received() const { return count_ > 0; }
  double lastReceive() const { return last_receive_; }
  double lastStamp() const { return last_stamp_; }

 private:
  StreamConfig config_;
  double window_, coverage_, started_ = 0, last_receive_ = 0, last_stamp_ = 0;
  unsigned min_samples_;
  uint64_t count_ = 0;
  std::deque<double> receipts_;
  std::deque<std::pair<double, double>> sources_;
};

// A separate gate per rule preserves a brief hard failure through one publish
// period, debounces soft candidates, and delays recovery without altering
// samples.
class RuleLatch {
 public:
  using EventSink =
      std::function<void(const std::string&, const Finding&, int, int)>;
  RuleLatch(double soft_hold, double recovery_hold, EventSink sink);
  void apply(const Findings& findings, double now);
  int level() const;
  int periodLevel() const;
  std::string reasons() const;
  void reset();
  void published();

 private:
  struct State {
    int active = OK, candidate = OK, period = OK;
    double since = 0, good_since = -1;
    Finding finding;
  };
  double soft_hold_, recovery_hold_;
  EventSink sink_;
  std::map<std::string, State> states_;
};

class IHealthChecker {
 public:
  virtual ~IHealthChecker() = default;
  virtual Findings check(const Sample& sample, Values& values) = 0;
  virtual void reset() {}
};
class LidarChecker : public IHealthChecker {
 public:
  explicit LidarChecker(const YAML::Node& quality);
  Findings check(const Sample& sample, Values& values) override;

 private:
  Bounds total_, valid_, ratio_;
};
class ImuChecker : public IHealthChecker {
 public:
  explicit ImuChecker(const YAML::Node& quality);
  Findings check(const Sample& sample, Values& values) override;

 private:
  Bounds acceleration_, gyro_;
};
class CovarianceChecker {
 public:
  static bool valid(const Eigen::MatrixXd& matrix, double tolerance,
                    bool symmetry, bool psd);
};
class KinematicsTracker {
 public:
  Findings update(const Sample& sample, const YAML::Node& quality,
                  Values& values);
  void reset();

 private:
  bool have_pose_ = false, have_velocity_ = false;
  double stamp_ = 0, midpoint_ = 0;
  std::string frame_;
  Eigen::Vector3d position_ = Eigen::Vector3d::Zero(),
                  velocity_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
};
class LocalizationChecker : public IHealthChecker {
 public:
  explicit LocalizationChecker(const YAML::Node& quality) : quality_(quality) {}
  Findings check(const Sample& sample, Values& values) override;
  void reset() override { motion_.reset(); }

 private:
  YAML::Node quality_;
  KinematicsTracker motion_;
};
class RelayChecker : public IHealthChecker {
 public:
  Findings check(const Sample& sample, Values& values) override {
    values = sample.values;
    return sample.findings;
  }
};
class CheckerFactory {
 public:
  static std::unique_ptr<IHealthChecker> create(const ModuleConfig& config);
};

struct RuntimeUpdate {
  std::string controller, instance, command, desired, observed, reason;
  uint64_t sequence = 0, frame_epoch = 0;
  bool trace_healthy = true;
  std::string controller_run_id, controller_log_directory, data_reference,
      data_sha256;
};
struct LifecycleReport {
  std::string state;
  bool expected_active = true, valid_state = false;
  Findings findings;
};
class LifecycleTracker {
 public:
  LifecycleTracker(const LifecycleConfig& config, const MonitorConfig& monitor,
                   double now);
  bool control(const RuntimeUpdate& update, double now);
  bool controlAccepted() const { return control_accepted_; }
  void sample(bool good, double now);
  LifecycleReport evaluate(double now, bool stream_ready, double receive_age);
  std::string instance() const { return update_.instance; }
  std::string command() const { return update_.command; }
  uint64_t epoch() const { return epoch_; }
  const RuntimeUpdate& runtime() const { return update_; }
  bool dataDemandEnabled() const {
    return state_ == "RUNNING" || state_ == "FAULT" || state_ == "UNKNOWN";
  }

 private:
  LifecycleConfig config_;
  bool strict_, controlled_ = false, control_accepted_ = false;
  std::set<std::string> retired_instances_, retired_controllers_;
  double heartbeat_warning_, heartbeat_error_, heartbeat_ = 0, transition_;
  unsigned good_samples_ = 0;
  uint64_t epoch_ = 1;
  RuntimeUpdate update_;
  std::string state_;
};

struct ModuleReport {
  std::string id, checker, topic, state, instance, command, message;
  uint64_t epoch = 1;
  int level = WARNING, data_level = OK;
  bool required = true, has_data = false, valid = false, expected_active = true;
  Values values;
};
class ModuleMonitor {
 public:
  using EventSink = RuleLatch::EventSink;
  ModuleMonitor(const ModuleConfig& config, const MonitorConfig& monitor,
                EventSink sink, double now);
  void receive(const Sample& sample);
  bool control(const RuntimeUpdate& update, double now);
  ModuleReport evaluate(double now);
  const ModuleConfig& config() const { return config_; }
  std::string instance() const { return lifecycle_.instance(); }
  std::string command() const { return lifecycle_.command(); }
  uint64_t epoch() const { return lifecycle_.epoch(); }
  const RuntimeUpdate& runtime() const { return lifecycle_.runtime(); }

 private:
  ModuleConfig config_;
  StreamTracker stream_;
  LifecycleTracker lifecycle_;
  RuleLatch rules_;
  RuleLatch control_rules_;
  std::unique_ptr<IHealthChecker> checker_;
  Findings latest_sample_;
  Values values_;
  bool have_data_ = false;
  EventSink sink_;
  std::set<std::string> sample_rule_ids_;
  std::string last_command_, last_state_;
};

}  // namespace robot_health
