#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <livox_ros_driver/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointField.h>
#include <std_msgs/UInt8.h>

namespace {

constexpr double STANDARD_GRAVITY_MPS2 = 9.80665;

enum HealthLevel : uint8_t {
  HEALTH_OK = 0,
  HEALTH_WARNING = 1,
  HEALTH_ERROR = 2
};

struct StreamState {
  bool received = false;
  bool stamp_valid = false;
  bool stamp_monotonic = true;
  double last_stamp = 0.0;
  double last_receive_wall = 0.0;
  std::deque<double> receive_times;
  uint64_t message_count = 0;
  uint8_t sample_level = HEALTH_OK;
  std::string sample_message = "sample OK";
  std::vector<diagnostic_msgs::KeyValue> sample_values;
};

struct ComponentResult {
  uint8_t level = HEALTH_ERROR;
  std::string message;
  std::vector<diagnostic_msgs::KeyValue> values;
};

diagnostic_msgs::KeyValue makeValue(const std::string& key,
                                    const std::string& value) {
  diagnostic_msgs::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}

std::string number(double value, int precision = 3) {
  std::ostringstream stream;
  stream.precision(precision);
  stream << std::fixed << value;
  return stream.str();
}

void promote(uint8_t candidate, const std::string& reason,
             uint8_t& level, std::string& message) {
  if (candidate > level) {
    level = candidate;
    message = reason;
  } else if (candidate == level && candidate != HEALTH_OK &&
             message.find(reason) == std::string::npos) {
    message += "; " + reason;
  }
}

const char* levelName(uint8_t level) {
  if (level == HEALTH_OK) return "OK";
  if (level == HEALTH_WARNING) return "WARNING";
  return "ERROR";
}

uint8_t diagnosticLevel(uint8_t level) {
  if (level == HEALTH_OK) return diagnostic_msgs::DiagnosticStatus::OK;
  if (level == HEALTH_WARNING) return diagnostic_msgs::DiagnosticStatus::WARN;
  return diagnostic_msgs::DiagnosticStatus::ERROR;
}

const sensor_msgs::PointField* findField(
    const sensor_msgs::PointCloud2& cloud, const std::string& name) {
  for (const auto& field : cloud.fields) {
    if (field.name == name) return &field;
  }
  return nullptr;
}

template <typename Scalar>
void readScalar(const uint8_t* data, bool swap_bytes, Scalar& value) {
  if (!swap_bytes || sizeof(Scalar) == 1) {
    std::memcpy(&value, data, sizeof(Scalar));
    return;
  }
  uint8_t reversed[sizeof(Scalar)];
  for (size_t index = 0; index < sizeof(Scalar); ++index) {
    reversed[index] = data[sizeof(Scalar) - index - 1];
  }
  std::memcpy(&value, reversed, sizeof(Scalar));
}

bool readFieldAsDouble(const uint8_t* data, uint8_t datatype,
                       bool swap_bytes, double& value) {
  switch (datatype) {
    case sensor_msgs::PointField::INT8: {
      int8_t v; readScalar(data, swap_bytes, v); value = v; return true;
    }
    case sensor_msgs::PointField::UINT8: {
      uint8_t v; readScalar(data, swap_bytes, v); value = v; return true;
    }
    case sensor_msgs::PointField::INT16: {
      int16_t v; readScalar(data, swap_bytes, v); value = v; return true;
    }
    case sensor_msgs::PointField::UINT16: {
      uint16_t v; readScalar(data, swap_bytes, v); value = v; return true;
    }
    case sensor_msgs::PointField::INT32: {
      int32_t v; readScalar(data, swap_bytes, v); value = v; return true;
    }
    case sensor_msgs::PointField::UINT32: {
      uint32_t v; readScalar(data, swap_bytes, v); value = v; return true;
    }
    case sensor_msgs::PointField::FLOAT32: {
      float v; readScalar(data, swap_bytes, v); value = v; return true;
    }
    case sensor_msgs::PointField::FLOAT64: {
      double v; readScalar(data, swap_bytes, v); value = v; return true;
    }
    default:
      return false;
  }
}

bool fieldFits(const sensor_msgs::PointField& field, uint32_t point_step) {
  uint32_t scalar_size = 0;
  switch (field.datatype) {
    case sensor_msgs::PointField::INT8:
    case sensor_msgs::PointField::UINT8: scalar_size = 1; break;
    case sensor_msgs::PointField::INT16:
    case sensor_msgs::PointField::UINT16: scalar_size = 2; break;
    case sensor_msgs::PointField::INT32:
    case sensor_msgs::PointField::UINT32:
    case sensor_msgs::PointField::FLOAT32: scalar_size = 4; break;
    case sensor_msgs::PointField::FLOAT64: scalar_size = 8; break;
    default: return false;
  }
  return field.count >= 1 && field.offset + scalar_size <= point_step;
}

struct CloudQuality {
  bool structurally_valid = true;
  uint64_t total_points = 0;
  uint64_t valid_points = 0;
  std::string error;
};

CloudQuality inspectPointCloud2(const sensor_msgs::PointCloud2& cloud,
                                double max_range) {
  CloudQuality result;
  result.total_points = static_cast<uint64_t>(cloud.width) * cloud.height;
  if (result.total_points == 0) return result;

  const sensor_msgs::PointField* x = findField(cloud, "x");
  const sensor_msgs::PointField* y = findField(cloud, "y");
  const sensor_msgs::PointField* z = findField(cloud, "z");
  if (!x || !y || !z || cloud.point_step == 0 ||
      !fieldFits(*x, cloud.point_step) || !fieldFits(*y, cloud.point_step) ||
      !fieldFits(*z, cloud.point_step)) {
    result.structurally_valid = false;
    result.error = "PointCloud2 is missing usable x/y/z fields";
    return result;
  }

  const uint64_t minimum_bytes = static_cast<uint64_t>(cloud.row_step) * cloud.height;
  if (cloud.row_step < cloud.point_step * cloud.width ||
      cloud.data.size() < minimum_bytes) {
    result.structurally_valid = false;
    result.error = "PointCloud2 data buffer is shorter than its dimensions";
    return result;
  }

  const double max_range_squared = max_range * max_range;
  const uint16_t endian_probe = 1;
  const bool host_is_big_endian =
      reinterpret_cast<const uint8_t*>(&endian_probe)[0] == 0;
  const bool swap_bytes = host_is_big_endian != cloud.is_bigendian;
  for (uint32_t row = 0; row < cloud.height; ++row) {
    for (uint32_t col = 0; col < cloud.width; ++col) {
      const uint64_t base = static_cast<uint64_t>(row) * cloud.row_step +
                            static_cast<uint64_t>(col) * cloud.point_step;
      double px = 0.0, py = 0.0, pz = 0.0;
      const bool decoded =
          readFieldAsDouble(cloud.data.data() + base + x->offset, x->datatype,
                            swap_bytes, px) &&
          readFieldAsDouble(cloud.data.data() + base + y->offset, y->datatype,
                            swap_bytes, py) &&
          readFieldAsDouble(cloud.data.data() + base + z->offset, z->datatype,
                            swap_bytes, pz);
      const double range_squared = px * px + py * py + pz * pz;
      if (decoded && std::isfinite(px) && std::isfinite(py) && std::isfinite(pz) &&
          range_squared > 1e-8 && range_squared <= max_range_squared) {
        ++result.valid_points;
      }
    }
  }
  return result;
}

}  // namespace

class FastLioHealthMonitor {
 public:
  FastLioHealthMonitor() : private_nh_("~") {
    private_nh_.param<std::string>("lidar_topic", lidar_topic_, "/livox/lidar");
    private_nh_.param<std::string>("lidar_visualization_topic",
        lidar_visualization_topic_, "lidar_raw_visualization");
    private_nh_.param<std::string>("lidar_visualization_frame",
        lidar_visualization_frame_, "livox_raw");
    private_nh_.param<std::string>("imu_topic", imu_topic_, "/livox/imu");
    // The node's default input is Livox, whose driver forwards acceleration in g.
    // SI IMUs must explicitly select mps2; do not infer units from LiDAR type.
    private_nh_.param<std::string>("imu_acceleration_unit", imu_acceleration_unit_, "g");
    if (imu_acceleration_unit_ == "g") {
      imu_acceleration_scale_ = STANDARD_GRAVITY_MPS2;
    } else if (imu_acceleration_unit_ == "mps2") {
      imu_acceleration_scale_ = 1.0;
    } else {
      const std::string error = "~imu_acceleration_unit must be 'g' or 'mps2'; got: " +
                                imu_acceleration_unit_;
      // Log before throwing: unwinding the last NodeHandle can shut ROS down,
      // making a message in main's catch handler invisible.
      ROS_FATAL_STREAM(error);
      throw std::invalid_argument(error);
    }
    private_nh_.param<std::string>("odometry_topic", odometry_topic_, "/Odometry");
    private_nh_.param<std::string>("lidar_type", lidar_type_, "livox_custom");
    private_nh_.param<std::string>("diagnostics_topic", diagnostics_topic_, "diagnostics");
    private_nh_.param<std::string>("status_topic", status_topic_, "status");
    private_nh_.param("publish_rate_hz", publish_rate_hz_, 2.0);
    private_nh_.param("lidar_stale_warning_sec", lidar_stale_warning_sec_, 0.5);
    private_nh_.param("lidar_stale_error_sec", lidar_stale_error_sec_, 1.5);
    private_nh_.param("imu_stale_warning_sec", imu_stale_warning_sec_, 0.15);
    private_nh_.param("imu_stale_error_sec", imu_stale_error_sec_, 0.6);
    private_nh_.param("odometry_stale_warning_sec", odometry_stale_warning_sec_, 0.3);
    private_nh_.param("odometry_stale_error_sec", odometry_stale_error_sec_, 1.0);
    private_nh_.param("minimum_lidar_rate_hz", minimum_lidar_rate_hz_, 5.0);
    private_nh_.param("minimum_imu_rate_hz", minimum_imu_rate_hz_, 50.0);
    private_nh_.param("minimum_lidar_points", minimum_lidar_points_, 100);
    private_nh_.param("minimum_valid_point_ratio", minimum_valid_point_ratio_, 0.8);
    private_nh_.param("maximum_point_range_m", maximum_point_range_m_, 500.0);
    private_nh_.param("maximum_acceleration_mps2", maximum_acceleration_mps2_, 100.0);
    private_nh_.param("maximum_angular_velocity_rps", maximum_angular_velocity_rps_, 35.0);
    private_nh_.param("odometry_warning_speed_mps", odometry_warning_speed_mps_, 15.0);
    private_nh_.param("odometry_error_speed_mps", odometry_error_speed_mps_, 50.0);
    private_nh_.param("maximum_pose_covariance", maximum_pose_covariance_, 100.0);
    private_nh_.param("rate_window_sec", rate_window_sec_, 5.0);

    if (publish_rate_hz_ <= 0.0) publish_rate_hz_ = 2.0;
    if (rate_window_sec_ <= 0.0) rate_window_sec_ = 5.0;
    if (minimum_valid_point_ratio_ < 0.0 || minimum_valid_point_ratio_ > 1.0) {
      minimum_valid_point_ratio_ = 0.8;
    }

    diagnostics_pub_ = private_nh_.advertise<diagnostic_msgs::DiagnosticArray>(
        diagnostics_topic_, 10, false);
    status_pub_ = private_nh_.advertise<std_msgs::UInt8>(status_topic_, 1, true);
    raw_cloud_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>(
        lidar_visualization_topic_, 1, false);

    if (lidar_type_ == "livox_custom") {
      livox_lidar_sub_ = nh_.subscribe(lidar_topic_, 20,
          &FastLioHealthMonitor::livoxLidarCallback, this);
    } else if (lidar_type_ == "pointcloud2") {
      pointcloud_lidar_sub_ = nh_.subscribe(lidar_topic_, 20,
          &FastLioHealthMonitor::pointCloudCallback, this);
    } else {
      lidar_state_.sample_level = HEALTH_ERROR;
      lidar_state_.sample_message = "lidar_type must be livox_custom or pointcloud2";
      ROS_ERROR_STREAM("Invalid ~lidar_type: " << lidar_type_);
    }
    imu_sub_ = nh_.subscribe(imu_topic_, 200,
        &FastLioHealthMonitor::imuCallback, this);
    odometry_sub_ = nh_.subscribe(odometry_topic_, 50,
        &FastLioHealthMonitor::odometryCallback, this);

    wall_timer_ = nh_.createWallTimer(ros::WallDuration(1.0 / publish_rate_hz_),
        &FastLioHealthMonitor::publishHealth, this);

    ROS_INFO_STREAM("FAST-LIO health monitor started: lidar=" << lidar_topic_
        << " (" << lidar_type_ << "), imu=" << imu_topic_
        << " (acceleration input=" << imu_acceleration_unit_
        << ", scale to m/s^2=" << imu_acceleration_scale_ << ")"
        << ", odometry=" << odometry_topic_);
  }

 private:
  void updateStream(StreamState& state, double stamp) {
    const double wall_now = ros::WallTime::now().toSec();
    const bool stamp_valid = std::isfinite(stamp) && stamp > 0.0;
    state.stamp_monotonic = !state.received ||
                            (stamp_valid && (!state.stamp_valid || stamp > state.last_stamp));
    state.stamp_valid = stamp_valid;
    if (stamp_valid) state.last_stamp = stamp;
    state.received = true;
    state.last_receive_wall = wall_now;
    ++state.message_count;
    state.receive_times.push_back(wall_now);
    while (!state.receive_times.empty() &&
           wall_now - state.receive_times.front() > rate_window_sec_) {
      state.receive_times.pop_front();
    }
  }

  void setSample(StreamState& state, uint8_t level, const std::string& message,
                 std::vector<diagnostic_msgs::KeyValue> values) {
    state.sample_level = level;
    state.sample_message = message;
    state.sample_values = std::move(values);
  }

  void livoxLidarCallback(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
    updateStream(lidar_state_, msg->header.stamp.toSec());
    const uint64_t total = msg->points.size();
    uint64_t valid = 0;
    for (const auto& point : msg->points) {
      const double x = point.x, y = point.y, z = point.z;
      const double range_squared = x * x + y * y + z * z;
      if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
          range_squared > 1e-8 && range_squared <= maximum_point_range_m_ * maximum_point_range_m_) {
        ++valid;
      }
    }

    const double ratio = total == 0 ? 0.0 : static_cast<double>(valid) / total;
    uint8_t level = HEALTH_OK;
    std::string message = "sample OK";
    if (total == 0) {
      level = HEALTH_ERROR;
      message = "Livox message contains no points";
    } else if (total < static_cast<uint64_t>(minimum_lidar_points_)) {
      level = HEALTH_WARNING;
      message = "LiDAR point count below configured minimum";
    }
    if (total > 0 && valid == 0) {
      level = HEALTH_ERROR;
      message = "LiDAR scan contains no valid points";
    } else if (ratio < minimum_valid_point_ratio_ && level < HEALTH_WARNING) {
      level = HEALTH_WARNING;
      message = "LiDAR valid-point ratio below configured minimum";
    }
    if (msg->point_num != msg->points.size() && level < HEALTH_WARNING) {
      level = HEALTH_WARNING;
      message = "Livox point_num differs from point array length";
    }
    if (!lidar_state_.stamp_valid) {
      level = HEALTH_ERROR;
      message = "LiDAR timestamp is zero or invalid";
    } else if (!lidar_state_.stamp_monotonic && level < HEALTH_WARNING) {
      level = HEALTH_WARNING;
      message = "LiDAR timestamp is not increasing";
    }

    setSample(lidar_state_, level, message,
        {makeValue("points_total", std::to_string(total)),
         makeValue("points_valid", std::to_string(valid)),
         makeValue("valid_ratio", number(ratio)),
         makeValue("driver_point_num", std::to_string(msg->point_num))});
    publishLivoxVisualizationCloud(*msg);
  }

  void publishLivoxVisualizationCloud(const livox_ros_driver::CustomMsg& msg) {
    sensor_msgs::PointCloud2 cloud;
    cloud.header = msg.header;
    if (!lidar_visualization_frame_.empty()) {
      cloud.header.frame_id = lidar_visualization_frame_;
    }
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(msg.points.size());
    cloud.is_bigendian = false;
    cloud.is_dense = true;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(4,
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(msg.points.size());

    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity(cloud, "intensity");
    for (const auto& point : msg.points) {
      *x = point.x;
      *y = point.y;
      *z = point.z;
      *intensity = point.reflectivity;
      cloud.is_dense = cloud.is_dense && std::isfinite(point.x) &&
                       std::isfinite(point.y) && std::isfinite(point.z);
      ++x; ++y; ++z; ++intensity;
    }
    raw_cloud_pub_.publish(cloud);
  }

  void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    updateStream(lidar_state_, msg->header.stamp.toSec());
    const CloudQuality quality = inspectPointCloud2(*msg, maximum_point_range_m_);
    const double ratio = quality.total_points == 0 ? 0.0 :
        static_cast<double>(quality.valid_points) / quality.total_points;
    uint8_t level = HEALTH_OK;
    std::string message = "sample OK";
    if (!quality.structurally_valid) {
      level = HEALTH_ERROR;
      message = quality.error;
    } else if (quality.total_points == 0 || quality.valid_points == 0) {
      level = HEALTH_ERROR;
      message = "PointCloud2 contains no valid points";
    } else if (quality.total_points < static_cast<uint64_t>(minimum_lidar_points_)) {
      level = HEALTH_WARNING;
      message = "LiDAR point count below configured minimum";
    } else if (ratio < minimum_valid_point_ratio_) {
      level = HEALTH_WARNING;
      message = "LiDAR valid-point ratio below configured minimum";
    }
    if (!lidar_state_.stamp_valid) {
      level = HEALTH_ERROR;
      message = "LiDAR timestamp is zero or invalid";
    } else if (!lidar_state_.stamp_monotonic && level < HEALTH_WARNING) {
      level = HEALTH_WARNING;
      message = "LiDAR timestamp is not increasing";
    }
    setSample(lidar_state_, level, message,
        {makeValue("points_total", std::to_string(quality.total_points)),
         makeValue("points_valid", std::to_string(quality.valid_points)),
         makeValue("valid_ratio", number(ratio)),
         makeValue("is_dense", msg->is_dense ? "true" : "false")});
    if (lidar_visualization_frame_.empty() ||
        lidar_visualization_frame_ == msg->header.frame_id) {
      raw_cloud_pub_.publish(msg);
    } else {
      sensor_msgs::PointCloud2 visualization_cloud = *msg;
      visualization_cloud.header.frame_id = lidar_visualization_frame_;
      raw_cloud_pub_.publish(visualization_cloud);
    }
  }

  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    updateStream(imu_state_, msg->header.stamp.toSec());
    const auto& a = msg->linear_acceleration;
    const auto& g = msg->angular_velocity;
    const bool finite = std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z) &&
                        std::isfinite(g.x) && std::isfinite(g.y) && std::isfinite(g.z);
    const double raw_accel_norm = std::hypot(a.x, std::hypot(a.y, a.z));
    const double accel_norm_mps2 = raw_accel_norm * imu_acceleration_scale_;
    const double gyro_norm = std::hypot(g.x, std::hypot(g.y, g.z));
    uint8_t level = HEALTH_OK;
    std::string message = "sample OK";
    if (!finite || !std::isfinite(accel_norm_mps2) || !std::isfinite(gyro_norm)) {
      level = HEALTH_ERROR;
      message = "IMU acceleration or angular velocity is non-finite";
    } else if (accel_norm_mps2 > maximum_acceleration_mps2_ ||
               gyro_norm > maximum_angular_velocity_rps_) {
      level = HEALTH_WARNING;
      message = "IMU magnitude exceeds configured physical range";
    }
    if (!imu_state_.stamp_valid) {
      level = HEALTH_ERROR;
      message = "IMU timestamp is zero or invalid";
    } else if (!imu_state_.stamp_monotonic && level < HEALTH_WARNING) {
      level = HEALTH_WARNING;
      message = "IMU timestamp is not increasing";
    }
    setSample(imu_state_, level, message,
        {makeValue("linear_acceleration_norm_mps2", number(accel_norm_mps2)),
         makeValue("linear_acceleration_norm_raw", number(raw_accel_norm)),
         makeValue("imu_acceleration_unit", imu_acceleration_unit_),
         makeValue("imu_acceleration_scale", number(imu_acceleration_scale_, 5)),
         makeValue("angular_velocity_norm_rps", number(gyro_norm))});
  }

  void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    updateStream(odometry_state_, msg->header.stamp.toSec());
    const auto& pose = msg->pose.pose;
    const double values[] = {pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w};
    bool finite = true;
    for (double value : values) finite = finite && std::isfinite(value);

    const double q_norm = std::sqrt(pose.orientation.x * pose.orientation.x +
        pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z +
        pose.orientation.w * pose.orientation.w);
    uint8_t level = HEALTH_OK;
    std::string message = "sample OK";
    if (!finite || !std::isfinite(q_norm) || q_norm < 1e-6) {
      level = HEALTH_ERROR;
      message = "Odometry pose contains invalid values or zero quaternion";
    } else if (std::abs(q_norm - 1.0) > 0.1) {
      level = HEALTH_ERROR;
      message = "Odometry quaternion norm is outside valid range";
    } else if (std::abs(q_norm - 1.0) > 0.02) {
      level = HEALTH_WARNING;
      message = "Odometry quaternion is not normalized";
    }

    double speed = 0.0;
    if (last_odom_position_valid_ && odometry_state_.stamp_valid &&
        odometry_state_.stamp_monotonic) {
      const double dt = msg->header.stamp.toSec() - last_odom_stamp_;
      if (dt > 1e-4) {
        const double dx = pose.position.x - last_odom_x_;
        const double dy = pose.position.y - last_odom_y_;
        const double dz = pose.position.z - last_odom_z_;
        speed = std::sqrt(dx * dx + dy * dy + dz * dz) / dt;
        if (speed > odometry_error_speed_mps_) {
          level = HEALTH_ERROR;
          message = "Odometry position jump exceeds error speed";
        } else if (speed > odometry_warning_speed_mps_ && level < HEALTH_WARNING) {
          level = HEALTH_WARNING;
          message = "Odometry position jump exceeds warning speed";
        }
      }
    }
    if (finite && odometry_state_.stamp_valid && odometry_state_.stamp_monotonic) {
      last_odom_x_ = pose.position.x;
      last_odom_y_ = pose.position.y;
      last_odom_z_ = pose.position.z;
      last_odom_stamp_ = msg->header.stamp.toSec();
      last_odom_position_valid_ = true;
    }

    double max_covariance = 0.0;
    bool covariance_valid = true;
    for (size_t i = 0; i < 36; ++i) {
      covariance_valid = covariance_valid && std::isfinite(msg->pose.covariance[i]);
    }
    const size_t diagonal_indices[] = {0, 7, 14, 21, 28, 35};
    for (size_t index : diagonal_indices) {
      const double covariance = msg->pose.covariance[index];
      if (covariance < 0.0) covariance_valid = false;
      max_covariance = std::max(max_covariance, covariance);
    }
    if (!covariance_valid) {
      level = HEALTH_ERROR;
      message = "Odometry covariance is non-finite or has a negative diagonal";
    } else if (max_covariance > maximum_pose_covariance_ && level < HEALTH_WARNING) {
      level = HEALTH_WARNING;
      message = "Odometry pose covariance exceeds configured maximum";
    }
    if (!odometry_state_.stamp_valid) {
      level = HEALTH_ERROR;
      message = "Odometry timestamp is zero or invalid";
    } else if (!odometry_state_.stamp_monotonic && level < HEALTH_WARNING) {
      level = HEALTH_WARNING;
      message = "Odometry timestamp is not increasing";
    }

    setSample(odometry_state_, level, message,
        {makeValue("position_x_m", number(pose.position.x)),
         makeValue("position_y_m", number(pose.position.y)),
         makeValue("position_z_m", number(pose.position.z)),
         makeValue("quaternion_norm", number(q_norm)),
         makeValue("linear_speed_mps", number(speed)),
         makeValue("maximum_pose_covariance", number(max_covariance))});
  }

  double rateHz(const StreamState& state) const {
    if (state.receive_times.size() < 2) return 0.0;
    const double duration = state.receive_times.back() - state.receive_times.front();
    if (duration <= 1e-6) return 0.0;
    return static_cast<double>(state.receive_times.size() - 1) / duration;
  }

  ComponentResult evaluate(const StreamState& state, const std::string& label,
      double stale_warning, double stale_error, double minimum_rate,
      double now_wall) const {
    ComponentResult result;
    result.level = HEALTH_OK;
    result.message = "OK";
    result.values = state.sample_values;
    if (!state.received) {
      result.level = HEALTH_ERROR;
      result.message = "No " + label + " messages received";
      result.values.push_back(makeValue("message_count", "0"));
      return result;
    }

    result.level = state.sample_level;
    result.message = state.sample_message;
    result.values.push_back(makeValue("sample_status", levelName(state.sample_level)));
    result.values.push_back(makeValue("sample_message", state.sample_message));
    const double age = now_wall - state.last_receive_wall;
    const double rate = rateHz(state);
    result.values.push_back(makeValue("message_count", std::to_string(state.message_count)));
    result.values.push_back(makeValue("age_sec", number(age)));
    result.values.push_back(makeValue("rate_hz", number(rate)));
    result.values.push_back(makeValue("last_stamp_sec", number(state.last_stamp, 6)));
    if (age > stale_error) {
      promote(HEALTH_ERROR, label + " data stale", result.level, result.message);
    } else if (age > stale_warning) {
      promote(HEALTH_WARNING, label + " data is getting stale", result.level, result.message);
    }
    if (state.receive_times.size() >= 3 && rate < minimum_rate) {
      promote(HEALTH_WARNING, label + " rate below configured minimum",
              result.level, result.message);
    }
    if (!state.stamp_monotonic) {
      promote(HEALTH_WARNING, label + " timestamp is not increasing",
              result.level, result.message);
    }
    return result;
  }

  diagnostic_msgs::DiagnosticStatus makeStatus(const std::string& name,
      const std::string& hardware_id, const ComponentResult& result) const {
    diagnostic_msgs::DiagnosticStatus status;
    status.name = name;
    status.hardware_id = hardware_id;
    status.level = diagnosticLevel(result.level);
    status.message = result.message.empty() ? levelName(result.level) : result.message;
    status.values = result.values;
    return status;
  }

  void publishHealth(const ros::WallTimerEvent&) {
    const double now_wall = ros::WallTime::now().toSec();
    ComponentResult lidar = evaluate(lidar_state_, "LiDAR", lidar_stale_warning_sec_,
        lidar_stale_error_sec_, minimum_lidar_rate_hz_, now_wall);
    ComponentResult imu = evaluate(imu_state_, "IMU", imu_stale_warning_sec_,
        imu_stale_error_sec_, minimum_imu_rate_hz_, now_wall);
    ComponentResult odometry = evaluate(odometry_state_, "Odometry",
        odometry_stale_warning_sec_, odometry_stale_error_sec_, 0.0, now_wall);
    lidar.values.push_back(makeValue("topic", lidar_topic_));
    imu.values.push_back(makeValue("topic", imu_topic_));
    odometry.values.push_back(makeValue("topic", odometry_topic_));

    const uint8_t overall = std::max(lidar.level, std::max(imu.level, odometry.level));
    ComponentResult summary;
    summary.level = overall;
    summary.message = std::string("overall health: ") + levelName(overall);
    summary.values = {makeValue("health_code", std::to_string(overall)),
                      makeValue("health_name", levelName(overall))};

    diagnostic_msgs::DiagnosticArray array;
    array.header.stamp = ros::Time::now();
    array.status.push_back(makeStatus("FAST-LIO health", "fastlio", summary));
    array.status.push_back(makeStatus("FAST-LIO LiDAR input", "lidar", lidar));
    array.status.push_back(makeStatus("FAST-LIO IMU input", "imu", imu));
    array.status.push_back(makeStatus("FAST-LIO localization output", "odometry", odometry));
    diagnostics_pub_.publish(array);

    std_msgs::UInt8 status;
    status.data = overall;
    status_pub_.publish(status);

    if (overall == HEALTH_ERROR) {
      ROS_WARN_THROTTLE(5.0, "FAST-LIO health ERROR: %s", summary.message.c_str());
    } else if (overall == HEALTH_WARNING) {
      ROS_WARN_THROTTLE(5.0, "FAST-LIO health WARNING");
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber livox_lidar_sub_;
  ros::Subscriber pointcloud_lidar_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber odometry_sub_;
  ros::Publisher diagnostics_pub_;
  ros::Publisher status_pub_;
  ros::Publisher raw_cloud_pub_;
  ros::WallTimer wall_timer_;

  std::string lidar_topic_, lidar_visualization_topic_, lidar_visualization_frame_;
  std::string imu_topic_, odometry_topic_, lidar_type_;
  std::string imu_acceleration_unit_;
  double imu_acceleration_scale_ = 1.0;
  std::string diagnostics_topic_, status_topic_;
  double publish_rate_hz_ = 2.0;
  double lidar_stale_warning_sec_ = 0.5, lidar_stale_error_sec_ = 1.5;
  double imu_stale_warning_sec_ = 0.15, imu_stale_error_sec_ = 0.6;
  double odometry_stale_warning_sec_ = 0.3, odometry_stale_error_sec_ = 1.0;
  double minimum_lidar_rate_hz_ = 5.0, minimum_imu_rate_hz_ = 50.0;
  int minimum_lidar_points_ = 100;
  double minimum_valid_point_ratio_ = 0.8, maximum_point_range_m_ = 500.0;
  double maximum_acceleration_mps2_ = 100.0, maximum_angular_velocity_rps_ = 35.0;
  double odometry_warning_speed_mps_ = 15.0, odometry_error_speed_mps_ = 50.0;
  double maximum_pose_covariance_ = 100.0, rate_window_sec_ = 5.0;

  StreamState lidar_state_, imu_state_, odometry_state_;
  bool last_odom_position_valid_ = false;
  double last_odom_x_ = 0.0, last_odom_y_ = 0.0, last_odom_z_ = 0.0;
  double last_odom_stamp_ = 0.0;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "fastlio_health_monitor");
  try {
    FastLioHealthMonitor monitor;
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL_STREAM("FAST-LIO health monitor startup/runtime failure: " << error.what());
    return 1;
  }
  return 0;
}
