/**
 * V2 ROS节点数据流：加载配置 -> 注册Adapter和ModuleMonitor -> 收样本/控制意图 -> SteadyTimer。
 * 定时输出标准诊断及UInt8状态，记录事件/指标；验证回放结束后封存，界面诊断继续。
 */
#include <diagnostic_msgs/DiagnosticArray.h>
#include <fastlio_health_monitor/RuntimeState.h>
#include <fastlio_health_monitor/bag_log_timeline.h>
#include <fastlio_health_monitor/event_recorder.h>
#include <fastlio_health_monitor/health_core.h>
#include <fastlio_health_monitor/input_adapters.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>

namespace robot_health {
class HealthRegistry {
 public:
  // 按配置创建模块、适配器和事件回调；ROS消息 -> Sample -> ModuleMonitor -> 事件记录。
  HealthRegistry(ros::NodeHandle& nh, const MonitorConfig& config,
                 EventRecorder& logs)
      : nh_(nh), config_(config), logs_(logs) {
    for (const auto& c : config.modules) {
      auto sink = [this, id = c.id](const std::string& type, const Finding& f,
                                    int before, int after) {
        auto it = modules_.find(id);
        ModuleMonitor* m = it == modules_.end() ? nullptr : it->second.get();
        Values context;
        if (m) {
          context["command_id"] = m->command();
          context["topic"] = m->config().topic;
          context["stamp_domain"] = m->config().stream.stamp_domain;
          const auto& runtime = m->runtime();
          context["controller_run_id"] = runtime.controller_run_id;
          context["controller_log_directory"] =
              runtime.controller_log_directory;
          context["data_reference"] = runtime.data_reference;
        }
        auto sc = sample_context_.find(id);
        if (sc != sample_context_.end() && type != "INSTANCE_RESET" &&
            type != "CONTROL_INTENT")
          context.insert(sc->second.begin(), sc->second.end());
        if (m && type != "INSTANCE_RESET" && type != "CONTROL_INTENT") {
          const auto& values = m->sampleValues();
          for (const auto* key : {"roll_deg", "pitch_deg", "attitude_status",
               "attitude_rotation_jump_deg", "attitude_angular_speed_dps",
               "range_m", "range_status", "range_delta_m", "range_rate_mps"}) {
            auto it = values.find(key);
            if (it != values.end()) context[it->first] = it->second;
          }
        }
        context["soft_warning_hold_sec"] = number(config_.soft_hold);
        context["recovery_hold_sec"] = number(config_.recovery_hold);
        logs_.event(type, id, m ? m->instance() : "", m ? m->epoch() : 1, f,
                    before, after, context);
      };
      modules_[c.id].reset(new ModuleMonitor(c, config, sink, steadyNow()));
      adapters_[c.id] =
          AdapterFactory::create(nh_, c, [this, id = c.id](const Sample& s) {
            sample_context_[id] = s.values;
            modules_.at(id)->receive(s);
          });
    }
    control_sub_ =
        nh_.subscribe(config.state_topic, 100, &HealthRegistry::control, this);
  }
  // 轮询任务型适配器并收集模块报告，再沿无环依赖图传播业务不可用状态。
  std::vector<ModuleReport> evaluate(double now) {
    for (auto& kv : adapters_) kv.second->poll(now);
    std::vector<ModuleReport> reports;
    for (auto& kv : modules_) reports.push_back(kv.second->evaluate(now));
    std::map<std::string, size_t> index;
    for (size_t i = 0; i < reports.size(); ++i) index[reports[i].id] = i;
    std::function<void(ModuleReport&)> dependencies = [&](ModuleReport& r) {
      for (const auto& d : modules_.at(r.id)->config().dependencies) {
        auto& dependency = reports[index.at(d)];
        dependencies(dependency);
        if (!dependency.valid) {
          r.valid = false;
          r.values["operational_valid"] = "false";
          r.values["dependency_" + d] = "unavailable";
        }
      }
    };
    for (auto& r : reports) dependencies(r);
    return reports;
  }
  // 封存前确认所有参与模块都属于本次播放器且已确认STOPPED，防止错误的EOF信号关闭日志。
  bool ownedByPlayback(const std::string& controller, const std::string& run,
                       const std::string& instance) const {
    bool have_required = false;
    for (const auto& kv : modules_) {
      if (!kv.second->contributesToOverall()) continue;
      have_required = true;
      const auto& u = kv.second->runtime();
      if (u.controller != controller || u.controller_run_id != run ||
          u.instance != instance || u.observed != "STOPPED" || u.desired != "STOP")
        return false;
    }
    return have_required;
  }

 private:
  // 接收非latched RuntimeState，更新对应binding模块；可信重置时重连订阅以清除旧队列。
  void control(
      const ros::MessageEvent<fastlio_health_monitor::RuntimeState const>&
          event) {
    const auto& header = *event.getConnectionHeaderPtr();
    auto latched = header.find("latching");
    if (latched != header.end() && latched->second == "1") {
      ROS_ERROR_THROTTLE(
          5, "Ignoring latched runtime state; heartbeat must be live");
      return;
    }
    auto m = event.getMessage();
    RuntimeUpdate u;
    u.controller = m->controller_id;
    u.instance = m->producer_instance_id;
    u.command = m->command_id;
    u.sequence = m->sequence;
    u.frame_epoch = m->frame_epoch;
    u.desired = m->desired_state;
    u.observed = m->observed_state;
    u.reason = m->reason;
    u.trace_healthy = m->trace_healthy;
    u.controller_run_id = m->controller_run_id;
    u.controller_log_directory = m->controller_log_directory;
    u.data_reference = m->data_reference;
    u.data_sha256 = m->data_sha256;
    for (auto& kv : modules_)
      if (kv.second->config().lifecycle.binding == m->binding) {
        if (kv.second->control(u, steadyNow())) {
          sample_context_[kv.first].clear();
          adapters_.at(kv.first)->reconnect();
        }
      }
  }
  ros::NodeHandle nh_;
  MonitorConfig config_;
  EventRecorder& logs_;
  ros::Subscriber control_sub_;
  std::map<std::string, std::unique_ptr<ModuleMonitor>> modules_;
  std::map<std::string, std::unique_ptr<IInputAdapter>> adapters_;
  std::map<std::string, Values> sample_context_;
};
class RobotHealthMonitor {
 public:
  // 加载配置、建立日志与模块注册表，发布标准诊断，并创建独立SteadyTimer时间探针。
  RobotHealthMonitor()
      : private_("~"),
        config_(load()),
        logs_(config_, bagLogTimeline(config_.data_reference)),
        registry_(nh_, config_, logs_) {
    private_.param<bool>("finish_log_on_playback_complete", finish_on_complete_, false);
    private_.param<std::string>("playback_state_topic", playback_state_topic_, "");
    if (finish_on_complete_) {
      if (playback_state_topic_.empty() || config_.data_reference.empty())
        throw std::invalid_argument("automatic log finish requires playback_state_topic and data_reference");
      playback_state_topic_ = nh_.resolveName(playback_state_topic_);
      expected_playback_node_ = playback_state_topic_.substr(0, playback_state_topic_.find_last_of('/'));
      playback_sub_ = nh_.subscribe(playback_state_topic_, 20,
                                    &RobotHealthMonitor::playbackState, this);
    }
    diagnostics_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(
        config_.diagnostics_topic, 10);
    status_ = nh_.advertise<std_msgs::UInt8>(config_.status_topic, 1, true);
    timer_ = nh_.createSteadyTimer(ros::WallDuration(1 / config_.publish_rate),
                                   &RobotHealthMonitor::tick, this);
    ROS_INFO_STREAM("C++ health V2 ready; modules="
                    << config_.modules.size() << "; config="
                    << config_.source_path << "; log=" << logs_.directory());
  }

 private:
  // 只接受指定播放器的新鲜完成状态，核对来源/实例/关联bag后安排封存。
  void playbackState(const ros::MessageEvent<fastlio_health_monitor::RuntimeState const>& event) {
    if (logs_.finished()) return;
    const auto& header = *event.getConnectionHeaderPtr();
    const auto caller = header.find("callerid"), latch = header.find("latching");
    if (caller == header.end() || caller->second != expected_playback_node_ ||
        (latch != header.end() && latch->second == "1")) return;
    const auto m = event.getMessage();
    if (m->data_reference != config_.data_reference || m->controller_id.empty() ||
        m->controller_run_id.empty() || m->producer_instance_id.empty()) return;
    // A manual STOP, PAUSED, FAULT or process exit alone is not verified EOF.
    const bool completed = m->desired_state == "STOP" && m->observed_state == "STOPPED" &&
                           m->exit_code == 0 && m->reason.find("clean EOF verified;") == 0;
    if (completed) {
      completed_controller_ = m->controller_id;
      completed_run_ = m->controller_run_id;
      completed_instance_ = m->producer_instance_id;
      finish_pending_ = true;
    } else finish_pending_ = false;
  }
  // 读取配置并应用launch传入的数据引用，保存对应的有效配置快照。
  MonitorConfig load() {
    std::string file;
    private_.param<std::string>("config_file", file, "");
    try {
      auto config = ConfigLoader::load(file);
      std::string reference_override;
      private_.param<std::string>("data_reference", reference_override, "");
      if (!reference_override.empty())
        config.data_reference = reference_override;
      auto effective = YAML::Load(config.effective_text);
      effective["monitor"]["data_reference"] = config.data_reference;
      YAML::Emitter out;
      out << effective;
      config.effective_text = out.c_str();
      return config;
    } catch (const std::exception& e) {
      ROS_FATAL_STREAM("Invalid V2 config: " << e.what());
      throw;
    }
  }
  // 把ModuleReport转换为标准DiagnosticStatus，保留module_id等稳定接入键。
  diagnostic_msgs::DiagnosticStatus diagnostic(const ModuleReport& r) {
    diagnostic_msgs::DiagnosticStatus s;
    s.name = "Robot health/" + r.id;
    s.hardware_id = r.id;
    s.level = r.level;
    s.message = r.message;
    Values values = r.values;
    values["module_id"] = r.id;
    values["checker"] = r.checker;
    values["topic"] = r.topic;
    values["required"] = r.required ? "true" : "false";
    for (const auto& kv : values) {
      diagnostic_msgs::KeyValue v;
      v.key = kv.first;
      v.value = kv.second;
      s.values.push_back(v);
    }
    return s;
  }
  // 定时评估全部模块 -> 汇总OK/WARNING/ERROR -> 发布诊断 -> 周期采样指标 -> 验证EOF后封存。
  void tick(const ros::SteadyTimerEvent&) {
    double now = steadyNow();
    auto reports = registry_.evaluate(now);
    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    int overall = OK;
    bool valid = true, all_active = true, all_participating_active = true;
    bool all_paused = true, all_stopped = true;
    unsigned participating_count = 0, optional_no_data_count = 0;
    Values summary = logs_.status();
    for (const auto& r : reports) {
      array.status.push_back(diagnostic(r));
      if (r.required)
        all_active = all_active && (r.state == "RUNNING" || r.state == "IDLE");
      if (!r.contributes_to_overall &&
          r.values.at("missing_data_policy") == "no_data_until_first_sample" &&
          r.values.at("data_presence") == "NO_DATA") ++optional_no_data_count;
      if (r.contributes_to_overall) {
        ++participating_count;
        all_paused = all_paused && r.state == "PAUSED";
        all_stopped = all_stopped && r.state == "STOPPED";
        overall = std::max(overall, r.level);
        valid = valid && r.valid;
        all_participating_active = all_participating_active &&
                                  (r.state == "RUNNING" || r.state == "IDLE");
      }
    }
    if (!logs_.healthy()) {
      overall = ERROR;
      valid = false;
    } else if (!logs_.enabled())
      overall = std::max(overall, int(WARNING));
    summary["module_id"] = "overall";
    summary["operational_valid"] = valid ? "true" : "false";
    summary["all_required_modules_active"] = all_active ? "true" : "false";
    summary["all_participating_modules_active"] = all_participating_active ? "true" : "false";
    summary["participating_module_count"] = std::to_string(participating_count);
    summary["optional_no_data_count"] = std::to_string(optional_no_data_count);
    if (participating_count && (all_paused || all_stopped))
      summary["operating_state"] = all_paused ? "PAUSED" : "STOPPED";
    diagnostic_msgs::DiagnosticStatus s;
    s.name = "Robot health/overall";
    s.hardware_id = "monitor";
    s.level = overall;
    s.message = std::string(levelName(overall)) +
                " | operational_valid=" + (valid ? "true" : "false");
    if (optional_no_data_count)
      s.message += " | optional_no_data=" + std::to_string(optional_no_data_count);
    for (const auto& kv : summary) {
      diagnostic_msgs::KeyValue v;
      v.key = kv.first;
      v.value = kv.second;
      s.values.push_back(v);
    }
    array.status.insert(array.status.begin(), s);
    diagnostic_msgs::DiagnosticStatus logging;
    logging.name = "Robot health/logging";
    logging.hardware_id = "logger";
    logging.level = !logs_.healthy() ? ERROR : logs_.enabled() ? OK : WARNING;
    logging.message = logs_.healthy()
                          ? logs_.enabled() ? (logs_.finished() ? "trace logging finished" : "trace logging active")
                                            : "trace logging disabled"
                          : "LOG_WRITE_FAILED or critical events dropped";
    for (const auto& kv : logs_.status()) {
      diagnostic_msgs::KeyValue v;
      v.key = kv.first;
      v.value = kv.second;
      logging.values.push_back(v);
    }
    array.status.push_back(logging);
    diagnostics_.publish(array);
    std_msgs::UInt8 legacy;
    legacy.data = overall;
    status_.publish(legacy);
    bool seal = finish_on_complete_ && finish_pending_ && !logs_.finished() &&
                registry_.ownedByPlayback(completed_controller_, completed_run_, completed_instance_);
    for (const auto& r : reports)
      if (r.contributes_to_overall && (r.state != "STOPPED" || r.instance != completed_instance_)) seal = false;
    if (!logs_.finished() && (seal || now - last_metrics_ >= config_.metrics_period)) {
      for (const auto& r : reports) logs_.metrics(r, now);
      last_metrics_ = now;
    }
    if (seal) {
      const bool ok = logs_.finish("verified bag playback completed; log sealed",
                                   {{"controller_run_id", completed_run_},
                                    {"producer_instance_id", completed_instance_},
                                    {"completion", "verified_eof"}});
      finish_pending_ = false;
      if (ok) ROS_INFO_STREAM("Health log finished at verified EOF: " << logs_.directory());
      else ROS_ERROR_STREAM("Health log finalization failed: " << logs_.directory());
    }
  }
  ros::NodeHandle nh_, private_;
  MonitorConfig config_;
  EventRecorder logs_;
  HealthRegistry registry_;
  ros::Publisher diagnostics_, status_;
  ros::Subscriber playback_sub_;
  ros::SteadyTimer timer_;
  std::string playback_state_topic_, expected_playback_node_;
  std::string completed_controller_, completed_run_, completed_instance_;
  bool finish_on_complete_ = false, finish_pending_ = false;
  double last_metrics_ = 0;
};
}  // namespace robot_health
// ROS程序入口：初始化节点、创建业务对象、运行回调循环；初始化失败返回非零退出码。
int main(int argc, char** argv) {
  ros::init(argc, argv, "fastlio_health_monitor");
  try {
    robot_health::RobotHealthMonitor monitor;
    ros::spin();
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "health V2 failure: " << e.what() << std::endl;
    return 1;
  }
}
