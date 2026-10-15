#include <fastlio_health_monitor/ControlModule.h>
#include <fastlio_health_monitor/RuntimeState.h>
#include <fastlio_health_monitor/event_recorder.h>
#include <fastlio_health_monitor/health_core.h>
#include <ros/ros.h>
#include <ros/master.h>
#include <ros/network.h>
#include <XmlRpcClient.h>
#include <std_srvs/SetBool.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
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
      g.node_name = n["supervised_ros_node"] ? n["supervised_ros_node"].as<std::string>() : "";
      g.args = n["command"].as<std::vector<std::string>>();
      g.autostart = n["autostart"] ? n["autostart"].as<bool>() : false;
      g.stop_timeout =
          n["stop_timeout_sec"] ? n["stop_timeout_sec"].as<double>() : 5;
      g.clean_exit_stop = n["clean_exit_is_task_completion"]
                              ? n["clean_exit_is_task_completion"].as<bool>()
                              : false;
      g.playback_topic = n["playback_state_topic"] ? n["playback_state_topic"].as<std::string>() : "";
      if (n["suspend_output_bindings"])
        g.suspend_outputs = n["suspend_output_bindings"].as<std::vector<std::string>>();
      g.instance = uniqueId();
      g.command = uniqueId();
      if (g.binding.empty() || g.args.empty() || g.args[0].empty() ||
          !std::isfinite(g.stop_timeout) || g.stop_timeout <= 0 ||
          groups_.count(g.binding))
        throw std::invalid_argument("invalid runtime group");
      groups_[g.binding] = g;
    }
    for (const auto& kv : groups_) {
      for (const auto& target : kv.second.suspend_outputs)
        if (!groups_.count(target)) throw std::invalid_argument("unknown suspended output binding");
      if (!kv.second.playback_topic.empty())
        playback_subs_.push_back(nh_.subscribe<fastlio_health_monitor::RuntimeState>(
            kv.second.playback_topic, 20,
            [this, binding = kv.first](const fastlio_health_monitor::RuntimeState::ConstPtr& m) {
              auto& group = groups_.at(binding);
              if (group.pid <= 0 || m->sequence == 0 || m->producer_instance_id.empty()) return;
              if (m->producer_instance_id == group.source_instance && m->sequence <= group.source_sequence) return;
              if (m->observed_state != "STARTING" && m->observed_state != "RUNNING" &&
                  m->observed_state != "PAUSED" && m->observed_state != "STOPPED" &&
                  m->observed_state != "STOPPING" &&
                  m->observed_state != "FAULT") return;
              const bool changed = group.source_state != m->observed_state ||
                                   group.source_instance != m->producer_instance_id;
              group.source_instance = m->producer_instance_id;
              group.source_sequence = m->sequence;
              group.source_state = m->observed_state;
              group.source_trace = m->trace_healthy;
              group.source_heartbeat = steadyNow();
              if (changed) {
                group.command = uniqueId();
                group.reason = "playback source: " + m->observed_state + " / " + m->reason;
                record(group, "PLAYBACK_SOURCE_CHANGED");
              }
              for (auto& item : groups_) publish(item.second);
            }));
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
    std::string playback_topic, source_instance, source_state;
    std::vector<std::string> suspend_outputs;
    uint64_t source_sequence = 0;
    double source_heartbeat = 0;
    bool source_trace = true;
    std::string node_name;
    bool node_verified = false, node_alive = false;
    double node_check = 0, started = 0;
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
                      {"playback_state", g.source_state},
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
    g.source_state.clear();
    g.source_instance.clear();
    g.source_sequence = 0;
    g.source_heartbeat = steadyNow();
    g.source_trace = true;
    g.node_verified = g.node_alive = false;
    g.started = steadyNow();
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
    if (req.command == "PAUSE" || req.command == "RESUME" ||
        (req.command == "RUN" && g.pid > 0 && g.source_state == "PAUSED")) {
      if (g.playback_topic.empty() || g.pid <= 0) {
        res.reason = "pause/resume requires an active playback group";
      } else {
        auto endpoint = g.playback_topic;
        const auto suffix = endpoint.rfind("/state");
        if (suffix == std::string::npos || suffix + 6 != endpoint.size()) {
          res.reason = "playback state topic must end with /state";
        } else {
          endpoint.replace(suffix, 6, "/pause");
          auto client = nh_.serviceClient<std_srvs::SetBool>(endpoint);
          std_srvs::SetBool call;
          call.request.data = req.command == "PAUSE";
          res.accepted = client.exists() && client.call(call) && call.response.success;
          res.reason = res.accepted ? "playback command acknowledged" : "playback command not acknowledged";
        }
      }
    } else if (req.command == "RUN") {
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
      res.reason = "commands: RUN, STOP, RESET_FRAME; playback groups also support PAUSE, RESUME";
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
    if (g.desired == "RUN" && g.observed == "RUNNING" && !g.node_name.empty() && !g.node_alive) {
      m.observed_state = !g.node_verified && steadyNow() - g.started < 10 ? "STARTING" : "FAULT";
      m.reason = "supervised ROS node unavailable: " + g.node_name;
    }
    // Keep process supervision authoritative. Replay inactivity suspends output
    // demand without pretending that a failed mapping process is healthy.
    if (g.desired == "RUN" && g.observed == "RUNNING" && !g.playback_topic.empty()) {
      if (g.source_instance.empty() && steadyNow() - g.source_heartbeat <= 10) {
        m.observed_state = "STARTING";
        m.reason = "waiting for playback source startup heartbeat";
      } else if (steadyNow() - g.source_heartbeat > 3) {
        m.observed_state = "FAULT";
        m.reason = "playback source heartbeat lost";
      } else if (g.source_state == "PAUSED") {
        m.desired_state = "PAUSE";
        m.observed_state = "PAUSED";
      } else if (g.source_state == "STOPPED") {
        m.desired_state = "STOP";
        m.observed_state = "STOPPED";
      } else if (g.source_state == "STOPPING") {
        m.desired_state = "STOP";
        m.observed_state = "STOPPING";
      } else if (g.source_state == "STARTING" || g.source_state == "FAULT")
        m.observed_state = g.source_state;
    }
    if (m.desired_state == "RUN" && m.observed_state == "RUNNING")
      for (const auto& item : groups_) {
        const auto& source = item.second;
        if (std::find(source.suspend_outputs.begin(), source.suspend_outputs.end(), g.binding) == source.suspend_outputs.end()) continue;
        if (source.desired == "STOP" && (source.observed == "STOPPED" || source.observed == "STOPPING")) {
          m.desired_state = source.observed == "STOPPED" ? "STOP" : "PAUSE";
          m.observed_state = source.observed == "STOPPED" ? "STOPPED" : "PAUSED";
          m.reason = "input replay stopped; output demand inactive, managed process retained";
        } else if (source.desired == "RUN" && source.observed == "RUNNING" &&
                   steadyNow() - source.source_heartbeat <= 3 &&
                   (source.source_state == "PAUSED" || source.source_state == "STOPPED" ||
                    source.source_state == "STOPPING")) {
          m.desired_state = source.source_state == "STOPPED" ? "STOP" : "PAUSE";
          m.observed_state = source.source_state == "STOPPED" ? "STOPPED" : "PAUSED";
          m.reason = "input replay " + source.source_state + "; output demand suspended, managed process retained";
        }
      }
    m.exit_code = g.exit_code;
    m.controller_run_id = logs_.runId();
    m.controller_log_directory = logs_.directory();
    m.trace_healthy = logs_.healthy() && logs_.enabled();
    if (!g.playback_topic.empty()) m.trace_healthy = m.trace_healthy && g.source_trace;
    const auto status = logs_.status();
    m.data_reference = status.at("data_reference");
    m.data_sha256 = status.at("data_sha256");
    pub_.publish(m);
  }
  void tick(const ros::SteadyTimerEvent&) {
    for (auto& kv : groups_) {
      auto& group = kv.second;
      if (group.pid > 0 && !group.node_name.empty() && steadyNow() - group.node_check >= .5) {
        group.node_check = steadyNow();
        const bool alive = nodeAlive(group.node_name);
        if (group.node_verified && group.node_alive && !alive) {
          group.reason = "supervised ROS node lost: " + group.node_name;
          record(group, "ROS_NODE_LOST");
        }
        group.node_alive = alive;
        group.node_verified = group.node_verified || alive;
      }
      reap(kv.second);
      publish(kv.second);
    }
  }
  bool nodeAlive(const std::string& name) {
    XmlRpc::XmlRpcValue request, response, uri;
    request[0] = ros::this_node::getName();
    request[1] = name;
    if (!ros::master::execute("lookupNode", request, response, uri, false)) return false;
    std::string host;
    uint32_t port;
    if (!ros::network::splitURI(static_cast<std::string>(uri), host, port)) return false;
    XmlRpc::XmlRpcClient client(host.c_str(), port, "/");
    XmlRpc::XmlRpcValue params, result;
    params[0] = ros::this_node::getName();
    if (!client.executeNonBlock("getPid", params)) return false;
    const double until = steadyNow() + .15;
    bool done = false;
    while (ros::ok() && steadyNow() < until) {
      client._disp.work(.005);
      if (client.executeCheckDone(result)) { done = true; break; }
      usleep(1000);
    }
    client.close();
    return done && result.getType() == XmlRpc::XmlRpcValue::TypeArray &&
           result.size() >= 3 && static_cast<int>(result[0]) == 1;
  }
  ros::NodeHandle nh_, private_;
  std::string id_, state_topic_;
  EventRecorder logs_;
  std::map<std::string, Group> groups_;
  std::map<std::string, uint64_t> frame_epoch_;
  uint64_t sequence_ = 0;
  ros::Publisher pub_;
  ros::ServiceServer service_;
  std::vector<ros::Subscriber> playback_subs_;
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
