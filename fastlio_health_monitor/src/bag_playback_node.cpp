#include <XmlRpcClient.h>
#include <XmlRpcValue.h>
#include <fastlio_health_monitor/RuntimeState.h>
#include <fastlio_health_monitor/bag_log_timeline.h>
#include <fastlio_health_monitor/event_recorder.h>
#include <fcntl.h>
#include <ros/master.h>
#include <ros/network.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>
#include <signal.h>
#include <std_srvs/SetBool.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <set>
#include <string>
#include <vector>

// Own the real terminal; rosbag gets a separate PTY. Every supported pause
// action goes through the same acknowledged service and lifecycle publication.
class TerminalInput {
 public:
  TerminalInput() {
    if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &saved_) != 0) return;
    flags_ = fcntl(STDIN_FILENO, F_GETFL);
    if (flags_ < 0) return;
    termios raw = saved_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) return;
    active_ = true;
    if (fcntl(STDIN_FILENO, F_SETFL, flags_ | O_NONBLOCK) < 0) restore();
  }
  ~TerminalInput() { restore(); }
  int readKey() {
    char c;
    return active_ && read(STDIN_FILENO, &c, 1) == 1 ? static_cast<unsigned char>(c) : -1;
  }
 private:
  void restore() {
    if (!active_) return;
    tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
    fcntl(STDIN_FILENO, F_SETFL, flags_);
    active_ = false;
  }
  termios saved_{};
  int flags_ = 0;
  bool active_ = false;
};

class BagPlayback {
 public:
  BagPlayback() : private_("~"), started_(robot_health::steadyNow()),
                  id_(robot_health::uniqueId()), instance_(robot_health::uniqueId()),
                  command_(robot_health::uniqueId()), logs_(logConfig(), logTimeline()) {
    private_.param<std::string>("bag", bag_, "");
    private_.param<double>("rate", rate_, 1.0);
    private_.param<double>("startup_timeout_sec", timeout_, 15.0);
    private_.param<double>("connection_delay_sec", delay_, 1.0);
    private_.param<std::string>("lidar_topic", lidar_topic_, "/livox/lidar");
    private_.param<std::string>("imu_topic", imu_topic_, "/livox/imu");
    private_.param<std::string>("mapping_node", mapping_node_, "/laserMapping");
    private_.param<std::string>("monitor_node", monitor_node_, "/fastlio_health_monitor");
    // Recorded-result review has no mapping process. Keep the live mapping
    // consumer and liveness requirement enabled for all existing launches.
    private_.param<bool>("require_mapping_node", require_mapping_node_, true);
    if (!private_.getParam("required_input_topics", required_input_topics_))
      required_input_topics_ = {lidar_topic_, imu_topic_};
    if (required_input_topics_.empty())
      throw std::invalid_argument("required_input_topics must not be empty");
    for (const auto& topic : required_input_topics_)
      if (topic.empty()) throw std::invalid_argument("required input topic must not be empty");
    private_.param<bool>("publish_lifecycle", direct_, false);
    private_.param<bool>("finish_log_on_playback_complete", finish_log_on_complete_, false);
    if (bag_.empty() || !std::isfinite(rate_) || rate_ <= 0 ||
        !std::isfinite(timeout_) || timeout_ <= 0 || !std::isfinite(delay_) || delay_ < 0)
      throw std::invalid_argument("invalid playback parameters");
    if (direct_) {
      std::string config;
      private_.param<std::string>("lifecycle_config_file", config, "");
      auto root = YAML::LoadFile(config);
      state_topic_ = root["control"]["state_topic"].as<std::string>();
      std::set<std::string> unique;
      for (const auto& module : root["modules"]) {
        const auto lifecycle = module["lifecycle"];
        unique.insert(lifecycle && lifecycle["control_binding"]
                          ? lifecycle["control_binding"].as<std::string>()
                          : module["id"].as<std::string>());
      }
      bindings_.assign(unique.begin(), unique.end());
      runtime_ = nh_.advertise<fastlio_health_monitor::RuntimeState>(state_topic_, 20, false);
    }
    // A managed controller consumes this private source heartbeat, retaining
    // ownership of the public runtime topic and checking its managed processes.
    source_state_ = private_.advertise<fastlio_health_monitor::RuntimeState>("state", 10, false);
    pause_service_ = private_.advertiseService("pause", &BagPlayback::pauseRequest, this);
    clock_sub_ = nh_.subscribe("/clock", 100, &BagPlayback::playerClock, this);
    timer_ = nh_.createSteadyTimer(ros::WallDuration(.02), &BagPlayback::tick, this);
    try {
      rosbag::Bag bag(bag_, rosbag::bagmode::Read);
      rosbag::View view(bag);
      if (!view.size()) throw std::runtime_error("empty bag");
      end_clock_ = view.getEndTime().toSec();
    } catch (const std::exception& e) {
      fail(std::string("cannot inspect bag completion boundary: ") + e.what());
      return;
    }
    record("PLAYBACK_STARTING", "waiting for required consumers and live transport");
    publish();
  }
  ~BagPlayback() {
    if (pid_ > 0) {
      kill(-pid_, SIGINT);
      double deadline = robot_health::steadyNow() + 3;
      int status = 0;
      while (robot_health::steadyNow() < deadline) {
        if (waitpid(pid_, &status, WNOHANG) == pid_) { pid_ = 0; break; }
        usleep(10000);
      }
      if (pid_ > 0) { kill(-pid_, SIGKILL); waitpid(pid_, &status, 0); }
    }
    if (pty_ >= 0) close(pty_);
  }
  int result() const { return result_; }
 private:
  robot_health::LogTimeline logTimeline() {
    std::string path;
    private_.param<std::string>("bag", path, "");
    return robot_health::bagLogTimeline(path);
  }
  void playerClock(const ros::MessageEvent<rosgraph_msgs::Clock const>& event) {
    const auto& header = *event.getConnectionHeaderPtr();
    const auto caller = header.find("callerid");
    if (caller != header.end() && caller->second == nh_.resolveName("fastlio_test_bag_player"))
      last_clock_ = event.getMessage()->clock.toSec();
  }
  robot_health::MonitorConfig logConfig() {
    robot_health::MonitorConfig c;
    private_.param<std::string>("bag", c.data_reference, "");
    private_.param<std::string>("log_directory", c.log_directory,
                               "/home/lmw/fastlio_ws/logs/robot_playback");
    YAML::Node settings;
    settings["node"] = ros::this_node::getName();
    settings["bag"] = c.data_reference;
    double rate;
    private_.param<double>("rate", rate, 1.0);
    settings["rate"] = rate;
    bool require_mapping;
    private_.param<bool>("require_mapping_node", require_mapping, true);
    settings["require_mapping_node"] = require_mapping;
    bool finish_log;
    private_.param<bool>("finish_log_on_playback_complete", finish_log, false);
    settings["finish_log_on_playback_complete"] = finish_log;
    std::vector<std::string> topics;
    if (private_.getParam("required_input_topics", topics))
      settings["required_input_topics"] = topics;
    settings["state_policy"] = "explicit pause acknowledgement; clean EOF stopped; unexpected exit fault";
    YAML::Emitter text;
    text << settings;
    c.source_text = c.effective_text = text.c_str();
    c.source_path = "ROS private playback parameters";
    return c;
  }
  void record(const std::string& type, const std::string& reason) {
    command_ = robot_health::uniqueId();
    reason_ = reason;
    robot_health::Finding f;
    f.id = "PLAYBACK_STATE";
    f.reason = desired_ + " / " + observed_ + " / " + reason;
    logs_.event(type, "playback", instance_, 1, f, 0,
                observed_ == "FAULT" ? robot_health::ERROR : robot_health::OK,
                {{"command_id", command_}, {"exit_code", std::to_string(result_)},
                 {"native_player_exit_code", std::to_string(player_exit_code_)}});
  }
  void publish() {
    fastlio_health_monitor::RuntimeState m;
    m.header.stamp = ros::Time::now();
    m.controller_id = id_;
    m.producer_instance_id = instance_;
    m.command_id = command_;
    m.desired_state = desired_;
    m.observed_state = observed_;
    m.reason = reason_;
    m.exit_code = result_;
    m.controller_run_id = logs_.runId();
    m.controller_log_directory = logs_.directory();
    m.trace_healthy = logs_.healthy() && logs_.enabled();
    const auto status = logs_.status();
    m.data_reference = status.at("data_reference");
    m.data_sha256 = status.at("data_sha256");
    m.binding = "sensors";
    m.sequence = ++sequence_;
    source_state_.publish(m);
    if (direct_) for (const auto& binding : bindings_) {
      m.binding = binding;
      m.sequence = ++sequence_;
      if (binding != "sensors" && mapper_verified_ && !mapper_alive_) {
        m.desired_state = "RUN";
        m.observed_state = "FAULT";
        m.reason = "mapping node unavailable; input pause/EOF does not certify its health";
      } else {
        m.desired_state = desired_;
        m.observed_state = observed_;
        m.reason = reason_;
      }
      runtime_.publish(m);
    }
  }
  bool ready() {
    XmlRpc::XmlRpcValue request, response, payload;
    request[0] = ros::this_node::getName();
    if (!ros::master::execute("getSystemState", request, response, payload, false) ||
        payload.getType() != XmlRpc::XmlRpcValue::TypeArray || payload.size() != 3) return false;
    auto& subscriptions = payload[1];
    for (const auto& topic : required_input_topics_) {
      bool mapper = !require_mapping_node_, monitor = false;
      for (int i = 0; i < subscriptions.size(); ++i) {
        auto& item = subscriptions[i];
        if (static_cast<std::string>(item[0]) != topic) continue;
        for (int j = 0; j < item[1].size(); ++j) {
          std::string node = item[1][j];
          mapper = mapper || node == mapping_node_;
          monitor = monitor || node == monitor_node_;
        }
      }
      if (!mapper || !monitor) return false;
    }
    return true;
  }
  bool query(const std::string& node, const char* method, XmlRpc::XmlRpcValue& result) {
    XmlRpc::XmlRpcValue request, response, uri;
    request[0] = ros::this_node::getName();
    request[1] = node;
    if (!ros::master::execute("lookupNode", request, response, uri, false)) return false;
    std::string host;
    uint32_t port;
    if (!ros::network::splitURI(static_cast<std::string>(uri), host, port)) return false;
    XmlRpc::XmlRpcClient client(host.c_str(), port, "/");
    XmlRpc::XmlRpcValue params;
    params[0] = ros::this_node::getName();
    if (!client.executeNonBlock(method, params)) return false;
    double deadline = robot_health::steadyNow() + .15;
    bool complete = false;
    while (robot_health::steadyNow() < deadline && ros::ok()) {
      client._disp.work(.005);
      if (client.executeCheckDone(result)) { complete = true; break; }
      usleep(1000);
    }
    client.close();
    return complete && result.getType() == XmlRpc::XmlRpcValue::TypeArray &&
           result.size() >= 3 && static_cast<int>(result[0]) == 1;
  }
  bool connected(const std::string& node) {
    XmlRpc::XmlRpcValue result;
    if (!query(node, "getBusInfo", result)) return false;
    XmlRpc::XmlRpcValue request, response, uri;
    request[0] = ros::this_node::getName();
    request[1] = nh_.resolveName("fastlio_test_bag_player");
    if (!ros::master::execute("lookupNode", request, response, uri, false)) return false;
    auto& bus = result[2];
    for (const auto& topic : required_input_topics_) {
      bool found = false;
      for (int i = 0; i < bus.size(); ++i) {
        auto& c = bus[i];
        if (c.size() >= 6 && static_cast<std::string>(c[1]) == static_cast<std::string>(uri) &&
            static_cast<std::string>(c[2]) == "i" && static_cast<std::string>(c[4]) == topic &&
            static_cast<bool>(c[5])) found = true;
      }
      if (!found) return false;
    }
    return true;
  }
  bool setPause(bool paused) {
    if (pid_ <= 0 || completed_ || !resumed_) return false;
    if (paused_ == paused) { publish(); return true; }
    auto client = nh_.serviceClient<std_srvs::SetBool>(
        nh_.resolveName("fastlio_test_bag_player/pause_playback"));
    std_srvs::SetBool call;
    call.request.data = paused;
    if (!client.exists() || !client.call(call) || !call.response.success) return false;
    paused_ = paused;
    desired_ = paused ? "PAUSE" : "RUN";
    observed_ = paused ? "PAUSED" : "RUNNING";
    record(paused ? "PLAYBACK_PAUSED" : "PLAYBACK_RESUMED",
           paused ? "pause acknowledged; input and dependent output demand suspended"
                  : "resume acknowledged; consumers must regain data readiness");
    publish();
    return true;
  }
  bool pauseRequest(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    res.success = setPause(req.data);
    res.message = res.success ? (req.data ? "PAUSED" : "RUNNING") : "player not ready/completed or acknowledgement failed";
    return true;
  }
  void fail(const std::string& reason, int code = 1) {
    result_ = code;
    desired_ = "RUN";
    observed_ = "FAULT";
    completed_ = true;
    record("PLAYBACK_FAILED", reason);
    publish();
    ROS_ERROR_STREAM(reason);
    if (!direct_) ros::shutdown();
  }
  void spawn() {
    pty_ = posix_openpt(O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (pty_ < 0 || grantpt(pty_) != 0 || unlockpt(pty_) != 0)
      throw std::runtime_error("cannot create rosbag keyboard PTY");
    int slave = open(ptsname(pty_), O_RDWR | O_NOCTTY | O_CLOEXEC);
    if (slave < 0) throw std::runtime_error("cannot open rosbag PTY");
    std::vector<std::string> values{"rosbag", "play", "--clock", "--pause",
                                  "--rate=" + std::to_string(rate_),
                                  "--delay=" + std::to_string(delay_), bag_,
                                  "__name:=fastlio_test_bag_player"};
    std::vector<char*> args;
    for (auto& value : values) args.push_back(const_cast<char*>(value.c_str()));
    args.push_back(nullptr);
    pid_ = fork();
    if (pid_ == 0) {
      setpgid(0, 0);
      dup2(slave, STDIN_FILENO);
      close(slave);
      close(pty_);
      execvp(args[0], args.data());
      _exit(127);
    }
    close(slave);
    if (pid_ < 0) { fail("cannot spawn bag player"); return; }
    setpgid(pid_, pid_);
    ROS_INFO("Required consumers subscribed; player waiting for actual connections. SPACE: pause/resume.");
  }
  void tick(const ros::SteadyTimerEvent&) {
    const double now = robot_health::steadyNow();
    if (mapper_verified_ && now - last_mapper_check_ >= .5) {
      last_mapper_check_ = now;
      XmlRpc::XmlRpcValue response;
      const bool alive = query(mapping_node_, "getPid", response);
      if (alive != mapper_alive_)
        record(alive ? "MAPPING_NODE_RESTORED" : "MAPPING_NODE_LOST",
               alive ? "mapping node responds again" : "mapping node no longer responds");
      mapper_alive_ = alive;
    }
    if (pid_ > 0) {
      int status = 0;
      const auto p = waitpid(pid_, &status, WNOHANG);
      if (p == pid_) {
        result_ = WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
        player_exit_code_ = result_;
        pid_ = 0;
        // rosnode kill/SIGINT can also return exit code 0. Require independent
        // evidence that this owned player's clock reached the file's end.
        if (result_ == 0 && resumed_) {
          completed_ = true;
          exit_pending_ = true;
          verify_exit_until_ = now + .1;
          desired_ = "STOP";
          observed_ = "STOPPING";
          record("PLAYBACK_VERIFYING_EOF", "player exited; verifying its clock reached the file end");
          publish();
        } else fail("bag player exited unexpectedly", result_ ? result_ : 1);
      } else if (p < 0 && errno == ECHILD) { pid_ = 0; fail("bag player ownership lost"); }
      else if (!resumed_ && !completed_) {
        if (now - started_ > timeout_) fail("bag transport readiness timed out");
        else if ((!require_mapping_node_ || connected(mapping_node_)) && connected(monitor_node_)) {
          resumed_ = true;
          if (!setPause(false)) { resumed_ = false; }
          else {
            mapper_verified_ = require_mapping_node_;
            mapper_alive_ = require_mapping_node_;
            last_mapper_check_ = now;
          }
        }
      }
    } else if (!completed_) {
      if (ready()) spawn();
      else if (now - started_ > timeout_) fail("timed out waiting for required consumer subscriptions");
    }
    if (exit_pending_ && (last_clock_ >= end_clock_ - .05 || now >= verify_exit_until_)) {
      exit_pending_ = false;
      if (last_clock_ >= end_clock_ - .05) {
        desired_ = "STOP";
        observed_ = "STOPPED";
        record("PLAYBACK_COMPLETED", "clean EOF verified; data demand stopped, map and node retained");
        publish();
        if (finish_log_on_complete_) {
          if (!logs_.finish("verified bag playback completed; log sealed",
                             {{"completion", "verified_eof"},
                              {"producer_instance_id", instance_}}))
            ROS_ERROR("Playback log finalization failed");
        }
        ROS_INFO("Bag complete: STOPPED. Keep monitor heartbeat alive; Ctrl+C closes the testbench.");
        if (!direct_) finish_wait_until_ = now + .3;
      } else fail("player exited before the bag end (including early zero-code shutdown)");
    }
    int key;
    while ((key = terminal_.readKey()) >= 0) {
      if (key == ' ' && !setPause(!paused_))
        ROS_WARN("Playback is not ready or already complete; restart launch after EOF.");
    }
    if (now - last_publish_ >= .05) { publish(); last_publish_ = now; }
    if (!direct_ && finish_wait_until_ > 0 && now >= finish_wait_until_) ros::shutdown();
  }
  ros::NodeHandle nh_, private_;
  double started_;
  std::string id_, instance_, command_;
  robot_health::EventRecorder logs_;
  TerminalInput terminal_;
  ros::SteadyTimer timer_;
  ros::Publisher source_state_, runtime_;
  ros::ServiceServer pause_service_;
  ros::Subscriber clock_sub_;
  std::vector<std::string> bindings_;
  std::vector<std::string> required_input_topics_;
  std::string bag_, lidar_topic_, imu_topic_, mapping_node_, monitor_node_, state_topic_;
  std::string desired_ = "RUN", observed_ = "STARTING", reason_;
  double rate_ = 1, timeout_ = 15, delay_ = 1, last_publish_ = 0, last_mapper_check_ = 0,
         finish_wait_until_ = 0;
  double last_clock_ = 0, end_clock_ = 0;
  double verify_exit_until_ = 0;
  pid_t pid_ = 0;
  int result_ = 0, pty_ = -1;
  int player_exit_code_ = -1;
  uint64_t sequence_ = 0;
  bool direct_ = false, resumed_ = false, paused_ = true, completed_ = false;
  bool require_mapping_node_ = true;
  bool finish_log_on_complete_ = false;
  bool mapper_verified_ = false, mapper_alive_ = false;
  bool exit_pending_ = false;
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "fastlio_bag_playback");
  try { BagPlayback playback; ros::spin(); return playback.result(); }
  catch (const std::exception& e) { ROS_FATAL_STREAM(e.what()); return 1; }
}
