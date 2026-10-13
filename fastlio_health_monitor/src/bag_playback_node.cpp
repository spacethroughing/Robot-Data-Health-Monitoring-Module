#include <XmlRpcClient.h>
#include <XmlRpcValue.h>
#include <ros/master.h>
#include <ros/network.h>
#include <ros/ros.h>
#include <signal.h>
#include <std_srvs/SetBool.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <string>
#include <vector>

// This C++ coordinator verifies named subscriptions before spawning rosbag.
// rosbag's --wait-for-subscribers only requires ANY subscriber per topic.
class BagPlayback {
 public:
  BagPlayback() : private_("~"), started_(now()) {
    private_.param<std::string>("bag", bag_, "");
    private_.param<double>("rate", rate_, 1.0);
    private_.param<double>("startup_timeout_sec", timeout_, 15.0);
    private_.param<double>("connection_delay_sec", delay_, 1.0);
    private_.param<std::string>("lidar_topic", lidar_topic_, "/livox/lidar");
    private_.param<std::string>("imu_topic", imu_topic_, "/livox/imu");
    private_.param<std::string>("mapping_node", mapping_node_, "/laserMapping");
    private_.param<std::string>("monitor_node", monitor_node_,
                                "/fastlio_health_monitor");
    if (bag_.empty() || !std::isfinite(rate_) || rate_ <= 0 ||
        !std::isfinite(timeout_) || timeout_ <= 0 || !std::isfinite(delay_) ||
        delay_ < 0)
      throw std::invalid_argument("invalid playback parameters");
    timer_ =
        nh_.createSteadyTimer(ros::WallDuration(.1), &BagPlayback::tick, this);
  }
  ~BagPlayback() {
    if (pid_ > 0) {
      kill(-pid_, SIGINT);
      double deadline = now() + 3;
      int status;
      while (now() < deadline) {
        if (waitpid(pid_, &status, WNOHANG) == pid_) {
          pid_ = 0;
          break;
        }
        usleep(10000);
      }
      if (pid_ > 0) {
        kill(-pid_, SIGKILL);
        waitpid(pid_, &status, 0);
      }
    }
  }
  int result() const { return result_; }

 private:
  static double now() {
    return std::chrono::duration<double>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
  }
  bool ready() {
    XmlRpc::XmlRpcValue request, response, payload;
    request[0] = ros::this_node::getName();
    if (!ros::master::execute("getSystemState", request, response, payload,
                              false) ||
        payload.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        payload.size() != 3)
      return false;
    auto& subscriptions = payload[1];
    for (const auto& topic : {lidar_topic_, imu_topic_}) {
      bool mapper = false, monitor = false;
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
  bool connected(const std::string& node) {
    XmlRpc::XmlRpcValue request, response, uri;
    request[0] = ros::this_node::getName();
    request[1] = node;
    if (!ros::master::execute("lookupNode", request, response, uri, false))
      return false;
    XmlRpc::XmlRpcValue player_uri;
    request[1] = nh_.resolveName("fastlio_test_bag_player");
    if (!ros::master::execute("lookupNode", request, response, player_uri,
                              false))
      return false;
    std::string host;
    uint32_t port;
    if (!ros::network::splitURI(static_cast<std::string>(uri), host, port))
      return false;
    XmlRpc::XmlRpcClient client(host.c_str(), port, "/");
    XmlRpc::XmlRpcValue params, result;
    params[0] = ros::this_node::getName();
    if (!client.executeNonBlock("getBusInfo", params)) return false;
    double deadline = now() + .25;
    bool complete = false;
    while (now() < deadline && ros::ok()) {
      client._disp.work(
          .01);  // drive nonblocking I/O; checkDone alone only inspects state.
      if (client.executeCheckDone(result)) {
        complete = true;
        break;
      }
      usleep(1000);
    }
    client.close();
    if (!complete || result.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        result.size() < 3 || static_cast<int>(result[0]) != 1)
      return false;
    auto& bus = result[2];
    for (const auto& topic : {lidar_topic_, imu_topic_}) {
      bool found = false;
      for (int i = 0; i < bus.size(); ++i) {
        auto& connection = bus[i];
        if (connection.size() >= 6 &&
            static_cast<std::string>(connection[1]) ==
                static_cast<std::string>(player_uri) &&
            static_cast<std::string>(connection[2]) == "i" &&
            static_cast<std::string>(connection[4]) == topic &&
            static_cast<bool>(connection[5]))
          found = true;
      }
      if (!found) return false;
    }
    return true;
  }
  void tick(const ros::SteadyTimerEvent&) {
    if (pid_ > 0) {
      int status = 0;
      auto p = waitpid(pid_, &status, WNOHANG);
      if (p == pid_) {
        result_ =
            WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status);
        pid_ = 0;
        ROS_INFO_STREAM("bag playback completed, exit=" << result_);
        ros::shutdown();
      }
      if (!resumed_) {
        if (now() - started_ > timeout_) {
          result_ = 1;
          ROS_ERROR("Bag transport readiness timed out");
          ros::shutdown();
          return;
        }
        if (connected(mapping_node_) && connected(monitor_node_)) {
          auto client = nh_.serviceClient<std_srvs::SetBool>(
              nh_.resolveName("fastlio_test_bag_player/pause_playback"));
          std_srvs::SetBool pause;
          pause.request.data = false;
          if (client.exists() && client.call(pause) && pause.response.success) {
            resumed_ = true;
            ROS_INFO(
                "Both consumers have live bag connections; playback resumed");
          }
        }
      }
      return;
    }
    if (!ready()) {
      if (now() - started_ > timeout_) {
        result_ = 1;
        ROS_ERROR(
            "Timed out waiting for both mapping and health monitor "
            "subscriptions");
        ros::shutdown();
      }
      return;
    }
    std::vector<std::string> values{"rosbag",
                                    "play",
                                    "--clock",
                                    "--pause",
                                    "--rate=" + std::to_string(rate_),
                                    "--delay=" + std::to_string(delay_),
                                    bag_,
                                    "__name:=fastlio_test_bag_player"};
    std::vector<char*> args;
    for (auto& s : values) args.push_back(const_cast<char*>(s.c_str()));
    args.push_back(nullptr);
    pid_ = fork();
    if (pid_ == 0) {
      setpgid(0, 0);
      execvp(args[0], args.data());
      _exit(127);
    }
    if (pid_ < 0) {
      result_ = 1;
      ROS_ERROR("Cannot spawn bag player");
      ros::shutdown();
      return;
    }
    setpgid(pid_, pid_);
    ROS_INFO(
        "Mapping and monitor subscribed; starting bag with connection delay");
  }
  ros::NodeHandle nh_, private_;
  ros::SteadyTimer timer_;
  std::string bag_, lidar_topic_, imu_topic_, mapping_node_, monitor_node_;
  double rate_ = 1, timeout_ = 15, delay_ = 1, started_;
  pid_t pid_ = 0;
  int result_ = 0;
  bool resumed_ = false;
};
int main(int argc, char** argv) {
  ros::init(argc, argv, "fastlio_bag_playback");
  try {
    BagPlayback playback;
    ros::spin();
    return playback.result();
  } catch (const std::exception& e) {
    ROS_FATAL_STREAM(e.what());
    return 1;
  }
}
