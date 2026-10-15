#include <diagnostic_msgs/DiagnosticArray.h>
#include <fastlio_health_monitor/TaskState.h>
#include <fastlio_health_monitor/input_adapters.h>
#include <geometry_msgs/PoseStamped.h>
#include <livox_ros_driver/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>

namespace robot_health {
namespace {
template <class T>
T get(const YAML::Node& n, const char* k, T d) {
  return n[k] ? n[k].as<T>() : d;
}
void finding(Sample& s, const std::string& id, const std::string& reason,
             int level, double value = 0, double limit = 0, bool hard = false) {
  s.findings.push_back(Finding{id, reason, id, "", level, value, limit, hard});
  if (level == ERROR) s.valid = false;
}
void binary(Sample& s, const std::string& id, bool valid,
            const std::string& reason, double value = 0) {
  finding(s, id, valid ? "valid" : reason, valid ? OK : ERROR, value, 0,
          !valid);
}
bool latched(const ros::M_string& header) {
  auto it = header.find("latching");
  return it != header.end() && it->second == "1";
}
class AdapterBase : public IInputAdapter {
 public:
  AdapterBase(ros::NodeHandle& nh, const ModuleConfig& c, SampleSink sink)
      : nh_(nh), config_(c), sink_(std::move(sink)) {}
  void poll(double) override {}

 protected:
  ros::NodeHandle nh_;
  ModuleConfig config_;
  SampleSink sink_;
  ros::Subscriber subscriber_;
  ros::Publisher visualization_;
  Sample sample(const std_msgs::Header& header) const {
    Sample s;
    s.stamp = header.stamp.toSec();
    s.received = steadyNow();
    s.frame = header.frame_id;
    // Keep the exact ROS header time; 12 significant digits lose fractional
    // detail at modern Unix epoch values. Core derivatives still use seconds.
    std::ostringstream stamp;
    stamp << header.stamp.sec << '.' << std::setw(9) << std::setfill('0')
          << header.stamp.nsec;
    s.values["source_stamp_sec"] = stamp.str();
    s.values["source_stamp_ns"] = std::to_string(header.stamp.toNSec());
    s.values["source_frame"] = header.frame_id;
    return s;
  }
};
bool validPoint(double x, double y, double z, double min, double max) {
  double r = std::hypot(x, std::hypot(y, z));
  return std::isfinite(r) && r > min && r <= max;
}
class LivoxAdapter : public AdapterBase {
 public:
  LivoxAdapter(ros::NodeHandle& n, const ModuleConfig& c, SampleSink s)
      : AdapterBase(n, c, std::move(s)) {
    if (!c.visual_topic.empty())
      visualization_ =
          nh_.advertise<sensor_msgs::PointCloud2>(c.visual_topic, 1);
    reconnect();
  }
  void reconnect() override {
    subscriber_.shutdown();
    subscriber_ =
        nh_.subscribe(config_.topic, 20, &LivoxAdapter::callback, this);
  }

 private:
  void callback(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
    Sample s = sample(msg->header);
    s.total = msg->points.size();
    binary(s, "LIDAR_STRUCTURE", msg->point_num == s.total,
           "point_num does not match array length", msg->point_num);
    double min = get<double>(config_.quality["range_m"], "minimum", .0001),
           max = get<double>(config_.quality["range_m"], "maximum", 500);
    for (const auto& p : msg->points)
      if (validPoint(p.x, p.y, p.z, min, max)) ++s.valid_points;
    s.values["driver_point_num"] = std::to_string(msg->point_num);
    sink_(s);
    if (!s.valid || !visualization_ || visualization_.getNumSubscribers() == 0)
      return;
    sensor_msgs::PointCloud2 cloud;
    cloud.header = msg->header;
    if (!config_.visual_frame.empty())
      cloud.header.frame_id = config_.visual_frame;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::FLOAT32, "z",
                                  1, sensor_msgs::PointField::FLOAT32,
                                  "intensity", 1,
                                  sensor_msgs::PointField::FLOAT32);
    modifier.resize(s.total);
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"),
        z(cloud, "z"), i(cloud, "intensity");
    cloud.is_dense = true;
    for (const auto& p : msg->points) {
      *x = p.x;
      *y = p.y;
      *z = p.z;
      *i = p.reflectivity;
      ++x;
      ++y;
      ++z;
      ++i;
      cloud.is_dense = cloud.is_dense && std::isfinite(p.x) &&
                       std::isfinite(p.y) && std::isfinite(p.z);
    }
    visualization_.publish(cloud);
  }
};
unsigned scalarSize(uint8_t t) {
  switch (t) {
    case 1:
    case 2:
      return 1;
    case 3:
    case 4:
      return 2;
    case 5:
    case 6:
    case 7:
      return 4;
    case 8:
      return 8;
    default:
      return 0;
  }
}
template <class T>
double read(const uint8_t* p, bool swap) {
  T v;
  uint8_t b[sizeof(T)];
  for (size_t i = 0; i < sizeof(T); ++i) b[i] = p[swap ? sizeof(T) - 1 - i : i];
  std::memcpy(&v, b, sizeof(T));
  return v;
}
double readNumber(const uint8_t* p, uint8_t t, bool swap) {
  switch (t) {
    case 1:
      return read<int8_t>(p, swap);
    case 2:
      return read<uint8_t>(p, swap);
    case 3:
      return read<int16_t>(p, swap);
    case 4:
      return read<uint16_t>(p, swap);
    case 5:
      return read<int32_t>(p, swap);
    case 6:
      return read<uint32_t>(p, swap);
    case 7:
      return read<float>(p, swap);
    case 8:
      return read<double>(p, swap);
    default:
      return std::numeric_limits<double>::quiet_NaN();
  }
}
class CloudAdapter : public AdapterBase {
 public:
  CloudAdapter(ros::NodeHandle& n, const ModuleConfig& c, SampleSink s)
      : AdapterBase(n, c, std::move(s)) {
    if (!c.visual_topic.empty())
      visualization_ =
          nh_.advertise<sensor_msgs::PointCloud2>(c.visual_topic, 1);
    reconnect();
  }
  void reconnect() override {
    subscriber_.shutdown();
    subscriber_ =
        nh_.subscribe(config_.topic, 20, &CloudAdapter::callback, this);
  }

 private:
  void callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    Sample s = sample(msg->header);
    s.total = uint64_t(msg->width) * msg->height;
    const sensor_msgs::PointField* fields[3] = {nullptr, nullptr, nullptr};
    for (const auto& field : msg->fields) {
      int index = field.name == "x"   ? 0
                  : field.name == "y" ? 1
                  : field.name == "z" ? 2
                                      : -1;
      if (index >= 0) fields[index] = &field;
    }
    // All dimension/offset products use uint64_t. Bounds use subtraction so a
    // UINT32_MAX offset cannot wrap and pass a field-fits test.
    bool valid = msg->point_step > 0 &&
                 uint64_t(msg->point_step) * msg->width <= msg->row_step &&
                 uint64_t(msg->row_step) * msg->height <= msg->data.size();
    for (const auto* f : fields)
      valid = valid && f && f->count == 1 && scalarSize(f->datatype) > 0 &&
              f->offset <= msg->point_step &&
              scalarSize(f->datatype) <= msg->point_step - f->offset;
    binary(s, "LIDAR_STRUCTURE", valid,
           "PointCloud2 fields/dimensions/buffer invalid");
    if (valid) {
      uint16_t endian = 1;
      bool big = reinterpret_cast<uint8_t*>(&endian)[0] == 0,
           swap = big != msg->is_bigendian;
      double min = get<double>(config_.quality["range_m"], "minimum", .0001),
             max = get<double>(config_.quality["range_m"], "maximum", 500);
      for (uint32_t row = 0; row < msg->height; ++row)
        for (uint32_t col = 0; col < msg->width; ++col) {
          size_t base =
              uint64_t(row) * msg->row_step + uint64_t(col) * msg->point_step;
          double p[3];
          for (unsigned i = 0; i < 3; ++i)
            p[i] = readNumber(msg->data.data() + base + fields[i]->offset,
                              fields[i]->datatype, swap);
          if (validPoint(p[0], p[1], p[2], min, max)) ++s.valid_points;
        }
    }
    sink_(s);
    if (!valid || !visualization_ || visualization_.getNumSubscribers() == 0)
      return;
    if (config_.visual_frame.empty() ||
        config_.visual_frame == msg->header.frame_id)
      visualization_.publish(msg);
    else {
      auto copy = *msg;
      copy.header.frame_id = config_.visual_frame;
      visualization_.publish(copy);
    }
  }
};
class ImuAdapter : public AdapterBase {
 public:
  ImuAdapter(ros::NodeHandle& n, const ModuleConfig& c, SampleSink s)
      : AdapterBase(n, c, std::move(s)) {
    reconnect();
  }
  void reconnect() override {
    subscriber_.shutdown();
    subscriber_ =
        nh_.subscribe(config_.topic, 1000, &ImuAdapter::callback, this);
  }

 private:
  template <class Array>
  bool covariance(Sample& s, const Array& a, const std::string& name,
                  double scale) {
    auto c = config_.quality["covariance"];
    if (!get<bool>(c, "enabled", true)) {
      s.values[name + "_covariance"] = "DISABLED";
      return true;
    }
    if (a[0] == -1) {
      s.values[name + "_covariance"] = "UNAVAILABLE";
      return false;
    }
    bool zero =
        std::all_of(a.begin(), a.end(), [](double v) { return v == 0; });
    if (zero) {
      s.values[name + "_covariance"] = "UNKNOWN";
      auto policy = get<std::string>(c, "unknown_policy", "report_only");
      finding(s, "IMU_" + name + "_COV_UNKNOWN", "unknown covariance",
              policy == "error"     ? ERROR
              : policy == "warning" ? WARNING
                                    : OK);
      if (get<bool>(c, "required_for_valid", false))
        s.values["required_checks_available"] = "false";
      return true;
    }
    s.values[name + "_covariance"] = "AVAILABLE";
    Eigen::Matrix3d matrix;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) matrix(i, j) = a[i * 3 + j] * scale;
    binary(s, "IMU_" + name + "_COV_INVALID",
           CovarianceChecker::valid(matrix,
                                    get<double>(c, "numeric_tolerance", 1e-9),
                                    get<bool>(c, "require_symmetric", true),
                                    get<bool>(c, "require_psd", true)),
           "nonfinite/asymmetric/non-PSD IMU covariance");
    return true;
  }
  void callback(const sensor_msgs::Imu::ConstPtr& msg) {
    Sample s = sample(msg->header);
    auto q = config_.quality;
    auto req = q["required_measurements"];
    double scale =
        get<std::string>(q, "acceleration_unit", "g") == "g" ? 9.80665 : 1;
    const auto& a = msg->linear_acceleration;
    const auto& g = msg->angular_velocity;
    s.acceleration = Eigen::Vector3d(a.x, a.y, a.z) * scale;
    s.gyro = Eigen::Vector3d(g.x, g.y, g.z);
    s.values["imu_acceleration_unit"] =
        get<std::string>(q, "acceleration_unit", "g");
    s.values["imu_acceleration_scale"] = number(scale);
    s.values["linear_acceleration_norm_raw"] =
        number(Eigen::Vector3d(a.x, a.y, a.z).stableNorm());
    s.values["linear_acceleration_norm_mps2"] =
        number(s.acceleration.stableNorm());
    s.values["angular_velocity_norm_rps"] = number(s.gyro.stableNorm());
    s.values["required_checks_available"] = "true";
    bool accel_available = msg->linear_acceleration_covariance[0] != -1,
         gyro_available = msg->angular_velocity_covariance[0] != -1;
    binary(s, "IMU_ACCEL_AVAILABLE",
           accel_available || !get<bool>(req, "acceleration", true),
           "required acceleration estimate unavailable");
    binary(s, "IMU_GYRO_AVAILABLE",
           gyro_available || !get<bool>(req, "angular_velocity", true),
           "required gyro estimate unavailable");
    binary(s, "IMU_ORIENTATION_AVAILABLE",
           msg->orientation_covariance[0] != -1 ||
               !get<bool>(req, "orientation", false),
           "required orientation unavailable");
    if (accel_available) {
      binary(s, "IMU_ACCEL_FINITE", s.acceleration.allFinite(),
             "NaN/Inf or SI conversion overflow", s.acceleration.stableNorm());
      covariance(s, msg->linear_acceleration_covariance, "ACCEL",
                 get<std::string>(q["covariance"], "acceleration_unit",
                                  "input_squared") == "input_squared"
                     ? scale * scale
                     : 1);
    } else {
      s.acceleration.setConstant(std::numeric_limits<double>::quiet_NaN());
      s.values["linear_acceleration_norm_mps2"] = "N/A";
    }
    if (gyro_available) {
      binary(s, "IMU_GYRO_FINITE", s.gyro.allFinite(), "NaN/Inf gyro",
             s.gyro.stableNorm());
      covariance(s, msg->angular_velocity_covariance, "GYRO", 1);
    } else {
      s.gyro.setConstant(std::numeric_limits<double>::quiet_NaN());
      s.values["angular_velocity_norm_rps"] = "N/A";
    }
    if (get<bool>(req, "orientation", false) &&
        msg->orientation_covariance[0] != -1) {
      const auto& o = msg->orientation;
      Eigen::Quaterniond quat(o.w, o.x, o.y, o.z);
      binary(s, "IMU_ORIENTATION_FINITE",
             quat.coeffs().allFinite() && std::abs(quat.norm() - 1) < .1,
             "invalid required orientation");
      covariance(s, msg->orientation_covariance, "ORIENTATION", 1);
    }
    s.values["checks_available"] =
        "finite,units,time,amplitude; noise/stuck/saturation uncalibrated";
    sink_(s);
  }
};
class OdomAdapter : public AdapterBase {
 public:
  OdomAdapter(ros::NodeHandle& n, const ModuleConfig& c, SampleSink s)
      : AdapterBase(n, c, std::move(s)) {
    reconnect();
  }
  void reconnect() override {
    subscriber_.shutdown();
    if (config_.adapter == "PoseStampedAdapter")
      subscriber_ =
          nh_.subscribe(config_.topic, 100, &OdomAdapter::poseCallback, this);
    else
      subscriber_ =
          nh_.subscribe(config_.topic, 100, &OdomAdapter::odomCallback, this);
  }

 private:
  void populate(Sample& s, const geometry_msgs::Pose& p) {
    s.position = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
    s.orientation = Eigen::Quaterniond(p.orientation.w, p.orientation.x,
                                       p.orientation.y, p.orientation.z);
    binary(s, "LOC_POSE_FINITE",
           s.position.allFinite() && s.orientation.coeffs().allFinite(),
           "nonfinite pose");
    double norm = s.orientation.coeffs().stableNorm();
    binary(s, "LOC_QUATERNION_ZERO", std::isfinite(norm) && norm > 1e-12,
           "zero/nonfinite quaternion", norm);
    if (std::isfinite(norm) && norm > 1e-12) {
      auto q = config_.quality["quaternion_norm_error"];
      Bounds b{get<double>(q, "warning_above", .02),
               get<double>(q, "error_above", .1), false};
      finding(s, "LOC_QUATERNION_NORM", "quaternion normalization error",
              b.level(std::abs(norm - 1)), std::abs(norm - 1), b.warning);
    }
    auto frame = get<std::string>(config_.quality, "expected_pose_frame", "");
    binary(s, "LOC_EXPECTED_FRAME",
           !s.frame.empty() && (frame.empty() || s.frame == frame),
           "unexpected pose frame");
    s.values["position_x_m"] = number(p.position.x);
    s.values["position_y_m"] = number(p.position.y);
    s.values["position_z_m"] = number(p.position.z);
    s.values["quaternion_norm"] = number(norm);
  }
  void odomCallback(const ros::MessageEvent<nav_msgs::Odometry const>& event) {
    if (latched(*event.getConnectionHeaderPtr())) return;
    const auto msg = event.getMessage();
    Sample s = sample(msg->header);
    s.child_frame = msg->child_frame_id;
    populate(s, msg->pose.pose);
    const auto& t = msg->twist.twist.linear;
    s.twist = Eigen::Vector3d(t.x, t.y, t.z);
    auto child = get<std::string>(config_.quality, "expected_child_frame", "");
    binary(s, "LOC_CHILD_FRAME", child.empty() || child == s.child_frame,
           "unexpected child frame");
    if (get<std::string>(config_.quality["kinematics"], "velocity_source",
                         "position_difference") == "twist")
      binary(s, "LOC_TWIST_FINITE", s.twist.allFinite(),
             "nonfinite trusted twist");
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j)
        s.covariance(i, j) = msg->pose.covariance[i * 6 + j];
    sink_(s);
  }
  void poseCallback(
      const ros::MessageEvent<geometry_msgs::PoseStamped const>& event) {
    if (latched(*event.getConnectionHeaderPtr())) return;
    const auto msg = event.getMessage();
    Sample s = sample(msg->header);
    populate(s, msg->pose);
    s.covariance_available = false;
    sink_(s);
  }
};
class DiagnosticAdapter : public AdapterBase {
 public:
  DiagnosticAdapter(ros::NodeHandle& n, const ModuleConfig& c, SampleSink s)
      : AdapterBase(n, c, std::move(s)) {
    reconnect();
  }
  void reconnect() override {
    subscriber_.shutdown();
    subscriber_ =
        nh_.subscribe(config_.topic, 100, &DiagnosticAdapter::callback, this);
  }

 private:
  void callback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg) {
    Sample s = sample(msg->header);
    int level = OK;
    std::string reason;
    for (const auto& status : msg->status) {
      binary(s, "RELAY_LEVEL_VALID", status.level >= 0 && status.level <= 3,
             "invalid source diagnostic level", status.level);
      level =
          std::max(level, status.level >= 2 ? int(ERROR) : int(status.level));
      if (status.level) reason += status.name + ":" + status.message + "; ";
    }
    binary(s, "RELAY_NONEMPTY", !msg->status.empty(), "empty diagnostic array");
    finding(s, "RELAY_STATUS",
            reason.empty() ? "source diagnostics OK" : reason, level, level, 0,
            level == ERROR);
    s.values["diagnostic_count"] = std::to_string(msg->status.size());
    s.values["checks_available"] =
        "source diagnostic relay; semantic accuracy not checked";
    sink_(s);
  }
};
class PathAdapter : public AdapterBase {
 public:
  PathAdapter(ros::NodeHandle& n, const ModuleConfig& c, SampleSink s)
      : AdapterBase(n, c, std::move(s)) {
    reconnect();
  }
  void reconnect() override {
    subscriber_.shutdown();
    task_sub_.shutdown();
    task_receive_ = 0;
    have_path_ = false;
    active_ = false;
    task_stamp_ = 0;
    goal_.clear();
    subscriber_ =
        nh_.subscribe(config_.topic, 20, &PathAdapter::callback, this);
    task_sub_ =
        nh_.subscribe(config_.quality["task_state_topic"].as<std::string>(), 20,
                      &PathAdapter::task, this);
  }
  void poll(double now) override {
    if (now - last_poll_ < .2) return;
    last_poll_ = now;
    Sample s;
    s.received = now;
    s.stamp = last_stamp_ > 0 ? last_stamp_ : 1;
    s.values["goal_id"] = goal_;
    s.values["task_state"] = active_ ? "ACTIVE" : "IDLE";
    bool current =
        task_receive_ > 0 &&
        now - task_receive_ <=
            get<double>(config_.quality, "task_state_timeout_sec", 2);
    binary(s, "PLANNING_TASK_FRESH", current, "task state missing or stale");
    if (task_receive_ > 0)
      binary(s, "PLANNING_GOAL_ID",
             !active_ || (!goal_.empty() && task_stamp_ > 0),
             "active goal requires nonempty goal_id and valid goal stamp");
    if (active_)
      binary(s, "PLANNING_DEADLINE",
             now - path_receive_ <=
                 get<double>(config_.quality, "result_deadline_sec", 2),
             "active goal has no fresh path");
    else
      finding(s, "PLANNING_DEADLINE", "no active goal; path demand suspended",
              OK);
    s.values["synthetic_status"] = "true";
    s.values["required_checks_available"] =
        active_ && (!have_path_ ||
                    now - path_receive_ >
                        get<double>(config_.quality, "result_deadline_sec", 2))
            ? "false"
            : "true";
    s.values["task_receive_age_sec"] =
        number(task_receive_ > 0 ? now - task_receive_ : 1e9);
    sink_(s);
  }

 private:
  ros::Subscriber task_sub_;
  double task_receive_ = 0, task_stamp_ = 0, path_receive_ = 0, last_stamp_ = 0,
         last_poll_ = 0;
  bool active_ = false, have_path_ = false;
  std::string goal_;
  void task(const fastlio_health_monitor::TaskState::ConstPtr& msg) {
    task_receive_ = steadyNow();
    bool changed = goal_ != msg->goal_id || active_ != msg->active;
    if (changed) {
      task_stamp_ = msg->header.stamp.toSec();
      have_path_ = false;
      path_receive_ = task_receive_;
    }
    goal_ = msg->goal_id;
    active_ = msg->active;
  }
  void callback(const nav_msgs::Path::ConstPtr& msg) {
    Sample s = sample(msg->header);
    last_stamp_ = s.stamp;
    s.values["goal_id"] = goal_;
    s.values["path_points"] = std::to_string(msg->poses.size());
    bool finite = true, frames = true, continuous = true;
    Eigen::Vector3d previous;
    bool first = true;
    for (const auto& p : msg->poses) {
      Eigen::Vector3d pos(p.pose.position.x, p.pose.position.y,
                          p.pose.position.z);
      const auto& o = p.pose.orientation;
      Eigen::Quaterniond q(o.w, o.x, o.y, o.z);
      finite = finite && pos.allFinite() && q.coeffs().allFinite() &&
               q.norm() > 1e-12;
      frames = frames && (p.header.frame_id.empty() ||
                          p.header.frame_id == msg->header.frame_id);
      if (!first)
        continuous = continuous &&
                     (pos - previous).stableNorm() <=
                         get<double>(config_.quality, "maximum_segment_m", 2);
      first = false;
      previous = pos;
    }
    binary(s, "PLANNING_PATH_FINITE", finite, "invalid path pose");
    binary(s, "PLANNING_PATH_FRAME", frames, "inconsistent path frames");
    binary(s, "PLANNING_PATH_CONTINUITY", continuous,
           "path segment exceeds configured limit");
    binary(s, "PLANNING_PATH_EMPTY", !active_ || !msg->poses.empty(),
           "empty path for active goal");
    binary(s, "PLANNING_GOAL_EPOCH",
           !active_ || (task_stamp_ > 0 && s.stamp >= task_stamp_),
           "path predates active goal timestamp");
    have_path_ = s.valid && !msg->poses.empty();
    if (have_path_) path_receive_ = s.received;
    s.values["checks_available"] =
        "path geometry,task freshness; collision/optimality not checked";
    sink_(s);
  }
};
}  // namespace
std::unique_ptr<IInputAdapter> AdapterFactory::create(
    ros::NodeHandle& n, const ModuleConfig& c, IInputAdapter::SampleSink sink) {
  if (c.adapter == "LivoxCustomAdapter")
    return std::unique_ptr<IInputAdapter>(
        new LivoxAdapter(n, c, std::move(sink)));
  if (c.adapter == "PointCloud2Adapter")
    return std::unique_ptr<IInputAdapter>(
        new CloudAdapter(n, c, std::move(sink)));
  if (c.adapter == "RosImuAdapter")
    return std::unique_ptr<IInputAdapter>(
        new ImuAdapter(n, c, std::move(sink)));
  if (c.adapter == "OdometryAdapter" || c.adapter == "PoseStampedAdapter")
    return std::unique_ptr<IInputAdapter>(
        new OdomAdapter(n, c, std::move(sink)));
  if (c.adapter == "DiagnosticRelayAdapter")
    return std::unique_ptr<IInputAdapter>(
        new DiagnosticAdapter(n, c, std::move(sink)));
  if (c.adapter == "PathAndTaskStateAdapter")
    return std::unique_ptr<IInputAdapter>(
        new PathAdapter(n, c, std::move(sink)));
  throw std::invalid_argument("unregistered adapter: " + c.adapter);
}
}  // namespace robot_health
