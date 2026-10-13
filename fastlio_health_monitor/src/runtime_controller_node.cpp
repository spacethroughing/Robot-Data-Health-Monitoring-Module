#include <fastlio_health_monitor/ControlModule.h>
#include <fastlio_health_monitor/RuntimeState.h>
#include <fastlio_health_monitor/event_recorder.h>
#include <fastlio_health_monitor/health_core.h>
#include <ros/ros.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <map>

namespace robot_health {
class RuntimeController {
 public:
  RuntimeController() : private_("~"), id_(uniqueId()), logs_(logConfig()) {
    std::string path;
    private_.param<std::string>("config_file", path, "");
    auto root = YAML::LoadFile(path);
    state_topic_ = root["state_topic"].as<std::string>();
    for (const auto& n : root["groups"]) {
      Group g;
      g.binding = n["binding"].as<std::string>();
      g.args = n["command"].as<std::vector<std::string>>();
      g.autostart = n["autostart"] ? n["autostart"].as<bool>() : false;
      g.stop_timeout =
          n["stop_timeout_sec"] ? n["stop_timeout_sec"].as<double>() : 5;
      g.clean_exit_stop = n["clean_exit_is_task_completion"]
                              ? n["clean_exit_is_task_completion"].as<bool>()
                              : false;
      g.instance = uniqueId();
      g.command = uniqueId();
      if (g.binding.empty() || g.args.empty() || g.args[0].empty() ||
          !std::isfinite(g.stop_timeout) || g.stop_timeout <= 0 ||
          groups_.count(g.binding))
        throw std::invalid_argument("invalid runtime group");
      groups_[g.binding] = g;
    }
    pub_ = nh_.advertise<fastlio_health_monitor::RuntimeState>(state_topic_,
                                                               100, false);
    service_ = nh_.advertiseService("/robot_runtime/control",
                                    &RuntimeController::command, this);
    timer_ = nh_.createSteadyTimer(ros::WallDuration(.2),
                                   &RuntimeController::tick, this);
    for (auto& kv : groups_)
      if (kv.second.autostart) start(kv.second);
  }
  ~RuntimeController() {
    for (auto& kv : groups_)
      if (kv.second.pid > 0) {
        stop(kv.second);
      }
  }
  void shutdown() {
    for (auto& kv : groups_)
      if (kv.second.pid > 0) stop(kv.second);
    double until = steadyNow() + 6;
    while (steadyNow() < until) {
      bool any = false;
      for (auto& kv : groups_) {
        reap(kv.second);
        any = any || kv.second.pid > 0;
      }
      if (!any) return;
      usleep(20000);
    }
    for (auto& kv : groups_)
      if (kv.second.pid > 0) {
        kill(-kv.second.pid, SIGKILL);
        waitpid(kv.second.pid, nullptr, 0);
        kv.second.pid = 0;
      }
  }

 private:
  struct Group {
    std::string binding, instance, command,
        desired = "STOP", observed = "STOPPED", reason = "initial stopped";
    std::vector<std::string> args;
    pid_t pid = 0;
    double stopping_since = 0, stop_timeout = 5;
    bool autostart = false, clean_exit_stop = false, forced = false;
    int exit_code = 0;
  };
  MonitorConfig logConfig() {
    MonitorConfig c;
    std::string path;
    private_.param<std::string>("config_file", path, "");
    auto n = YAML::LoadFile(path);
    c.source_path = path;
    std::ifstream in(path);
    c.source_text.assign(std::istreambuf_iterator<char>(in), {});
    c.effective_text = c.source_text;
    c.log_directory = n["log_directory"].as<std::string>();
    for (const auto& group : n["groups"])
      for (const auto& argument : group["command"]) {
        auto value = argument.as<std::string>();
        if (value.size() > 4 && value.substr(value.size() - 4) == ".bag")
          c.data_reference = value.find(":=") == std::string::npos
                                 ? value
                                 : value.substr(value.find(":=") + 2);
      }
    return c;
  }
  void record(Group& g, const std::string& event, bool durable = false) {
    Finding f;
    f.id = "RUNTIME_CONTROL";
    f.reason = g.desired + " / " + g.observed + " / " + g.reason;
    if (!logs_.event(event, g.binding, g.instance, 1, f, 0,
                     g.observed == "FAULT" ? ERROR : OK,
                     {{"command_id", g.command},
                      {"controller_id", id_},
                      {"exit_code", std::to_string(g.exit_code)}},
                     durable))
      ROS_ERROR_STREAM("Runtime trace incomplete for " << event);
  }
  bool start(Group& g) {
    if (g.pid > 0) return false;
    g.instance = uniqueId();
    g.command = uniqueId();
    g.desired = "RUN";
    g.observed = "STARTING";
    g.reason = "explicit run";
    g.forced = false;
    record(g, "START_REQUESTED", true);
    publish(g);
    std::vector<char*> args;
    for (auto& a : g.args) args.push_back(const_cast<char*>(a.c_str()));
    args.push_back(nullptr);
    pid_t child = fork();
    if (child == 0) {
      setpgid(0, 0);
      execvp(args[0], args.data());
      _exit(127);
    }
    if (child < 0) {
      g.observed = "FAULT";
      g.reason = "fork failed";
      record(g, "START_FAILED", true);
      return false;
    }
    setpgid(child, child);
    g.pid = child;
    g.observed = "RUNNING";
    g.reason = "managed process alive; data readiness evaluated by monitor";
    record(g, "PROCESS_STARTED", true);
    publish(g);
    return true;
  }
  void stop(Group& g) {
    g.command = uniqueId();
    g.desired = "STOP";
    if (g.pid > 0) {
      g.observed = "STOPPING";
      g.reason = "explicit stop; waiting for child reap";
      g.stopping_since = steadyNow();
      record(g, "STOP_REQUESTED", true);
      publish(g);
      kill(-g.pid, SIGINT);
    } else {
      g.observed = "STOPPED";
      g.reason = "already stopped";
      record(g, "STOP_CONFIRMED", true);
      publish(g);
    }
  }
  bool command(fastlio_health_monitor::ControlModule::Request& req,
               fastlio_health_monitor::ControlModule::Response& res) {
    auto it = groups_.find(req.binding);
    if (it == groups_.end()) {
      res.reason = "unknown control binding";
      return true;
    }
    auto& g = it->second;
    if (req.command == "RUN") {
      res.accepted = start(g);
      res.reason =
          res.accepted ? "start accepted" : "already running/start failed";
    } else if (req.command == "STOP") {
      stop(g);
      res.accepted = true;
      res.reason = "stop intention accepted; wait for STOPPED heartbeat";
    } else if (req.command == "RESET_FRAME") {
      ++frame_epoch_[g.binding];
      g.command = uniqueId();
      g.reason = "authorized frame reset";
      record(g, "FRAME_RESET", true);
      publish(g);
      res.accepted = true;
      res.reason = "frame epoch advanced";
    } else {
      res.reason = "commands: RUN, STOP, RESET_FRAME";
    }
    res.command_id = g.command;
    res.producer_instance_id = g.instance;
    return true;
  }
  void reap(Group& g) {
    if (g.pid <= 0) return;
    int status = 0;
    pid_t p = waitpid(g.pid, &status, WNOHANG);
    if (p == g.pid) {
      g.exit_code =
          WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
      g.pid = 0;
      if (g.desired == "STOP" && !g.forced) {
        g.observed = "STOPPED";
        g.reason = "managed process group stopped and child reaped";
        record(g, "STOP_CONFIRMED", true);
      } else if (g.clean_exit_stop && g.exit_code == 0 && !g.forced) {
        g.desired = "STOP";
        g.observed = "STOPPED";
        g.reason = "configured task completion";
        record(g, "TASK_COMPLETED", true);
      } else {
        g.observed = "FAULT";
        g.reason = "unexpected process exit / forced stop";
        record(g, "PROCESS_LOST", true);
      }
      publish(g);
    } else if (g.desired == "STOP" &&
               steadyNow() - g.stopping_since > g.stop_timeout && !g.forced) {
      g.forced = true;
      g.observed = "FAULT";
      g.reason = "stop deadline exceeded; forcing process group termination";
      record(g, "STOP_FAILED", true);
      publish(g);
      kill(-g.pid, SIGKILL);
    }
  }
  void publish(Group& g) {
    fastlio_health_monitor::RuntimeState m;
    m.header.stamp = ros::Time::now();
    m.binding = g.binding;
    m.controller_id = id_;
    m.producer_instance_id = g.instance;
    m.command_id = g.command;
    m.sequence = ++sequence_;
    m.frame_epoch = frame_epoch_[g.binding];
    m.desired_state = g.desired;
    m.observed_state = g.observed;
    m.reason = g.reason;
    m.exit_code = g.exit_code;
    m.controller_run_id = logs_.runId();
    m.controller_log_directory = logs_.directory();
    m.trace_healthy = logs_.healthy() && logs_.enabled();
    const auto status = logs_.status();
    m.data_reference = status.at("data_reference");
    m.data_sha256 = status.at("data_sha256");
    pub_.publish(m);
  }
  void tick(const ros::SteadyTimerEvent&) {
    for (auto& kv : groups_) {
      reap(kv.second);
      publish(kv.second);
    }
  }
  ros::NodeHandle nh_, private_;
  std::string id_, state_topic_;
  EventRecorder logs_;
  std::map<std::string, Group> groups_;
  std::map<std::string, uint64_t> frame_epoch_;
  uint64_t sequence_ = 0;
  ros::Publisher pub_;
  ros::ServiceServer service_;
  ros::SteadyTimer timer_;
};
}  // namespace robot_health
int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_runtime_controller");
  try {
    robot_health::RuntimeController controller;
    ros::spin();
    controller.shutdown();
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "Runtime controller failed: " << e.what() << std::endl;
    return 1;
  }
}
