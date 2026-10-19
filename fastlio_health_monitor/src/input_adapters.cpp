/**
 * 输入数据流：ROS订阅回调 -> 解包/边界检查/单位转换 -> Sample -> SampleSink。
 * 可选的业务阈值由配置决定，结构不可解析与非有限量保持基础守卫，防止错误计算。
 */
#include <diagnostic_msgs/DiagnosticArray.h>
#include <fastlio_health_monitor/TaskState.h>
#include <fastlio_health_monitor/input_adapters.h>
#include <geometry_msgs/PoseStamped.h>
#ifdef HEALTH_WITH_LIVOX
#include <livox_ros_driver/CustomMsg.h>
#endif
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>

namespace robot_health {
namespace {
template <class T>
// 从可选YAML项读取值；未写/null采用运行默认值，不推断设备型号或单位。
T get(const YAML::Node& n, const char* k, T d) {
  return n && n[k] && !n[k].IsNull() ? n[k].as<T>() : d;
}
// 将适配器发现的结构/数值问题写入Sample；ERROR同时标记该样本不可用于计算。
void finding(Sample& s, const std::string& id, const std::string& reason,
             int level, double value = 0, double limit = 0, bool hard = false) {
  s.findings.push_back(Finding{id, reason, id, "", level, value, limit, hard});
  if (level == ERROR) s.valid = false;
}
// 生成通过/失败二值规则；只处理基础有效性，不在这里拼装总体健康。
void binary(Sample& s, const std::string& id, bool valid,
            const std::string& reason, double value = 0) {
  finding(s, id, valid ? "valid" : reason, valid ? OK : ERROR, value, 0,
          !valid);
}
// 检查ROS连接是否为latched；旧缓存消息不能被当作新的控制/定位样本。
bool latched(const ros::M_string& header) {
  auto it = header.find("latching");
  return it != header.end() && it->second == "1";
}
class AdapterBase : public IInputAdapter {
 public:
  // 保存ROS句柄、模块配置和Sample出口，统一管理订阅与点云显示。
  AdapterBase(ros::NodeHandle& nh, const ModuleConfig& c, SampleSink sink)
      : nh_(nh), config_(c), sink_(std::move(sink)) {}
  // 默认流式适配器无需额外轮询；任务驱动规划适配器会覆盖此函数。
  void poll(double) override {}

 protected:
  ros::NodeHandle nh_;
  ModuleConfig config_;
  SampleSink sink_;
  ros::Subscriber subscriber_;
  ros::Publisher visualization_;
  // 把ROS Header转换为统一Sample，保留精确源纳秒和单调接收时间供后续日志/探针。
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
// 判断点XYZ有限且距离落在选定过滤范围；仅统计，不修改原始点云。
bool validPoint(double x, double y, double z, double min, double max) {
  double r = std::hypot(x, std::hypot(y, z));
  return std::isfinite(r) && r > min && r <= max;
}
#ifdef HEALTH_WITH_LIVOX
class LivoxAdapter : public AdapterBase {
 public:
  // 创建Livox CustomMsg订阅和可选可视化输出；核心不直接依赖该消息类型。
  LivoxAdapter(ros::NodeHandle& n, const ModuleConfig& c, SampleSink s)
      : AdapterBase(n, c, std::move(s)) {
    if (!c.visual_topic.empty())
      visualization_ =
          nh_.advertise<sensor_msgs::PointCloud2>(c.visual_topic, 1);
    reconnect();
  }
  // 关闭旧Livox订阅并重建，可信实例重置后不沿用旧消息队列。
  void reconnect() override {
    subscriber_.shutdown();
    subscriber_ =
        nh_.subscribe(config_.topic, 20, &LivoxAdapter::callback, this);
  }

 private:
  // 校验Livox点数结构 -> 统计有限/范围内点 -> 输出Sample -> 按需桥接RViz点云。
  void callback(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
    Sample s = sample(msg->header);
    s.total = msg->points.size();
    binary(s, "LIDAR_STRUCTURE", msg->point_num == s.total,
           "point_num does not match array length", msg->point_num);
    double min = (ruleEnabled(config_.quality["range_m"]) ? get<double>(config_.quality["range_m"], "minimum", 0) : 0),
           max = (ruleEnabled(config_.quality["range_m"]) ? get<double>(config_.quality["range_m"], "maximum", std::numeric_limits<double>::infinity()) : std::numeric_limits<double>::infinity());
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
// 根据PointField数据类型求单元素字节数，用于校验字段和缓冲区边界。
#endif  // HEALTH_WITH_LIVOX
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
// 按消息端序安全读取一个标量；使用memcpy避免未对齐内存访问。
double read(const uint8_t* p, bool swap) {
  T v;
  uint8_t b[sizeof(T)];
  for (size_t i = 0; i < sizeof(T); ++i) b[i] = p[swap ? sizeof(T) - 1 - i : i];
  std::memcpy(&v, b, sizeof(T));
  return v;
}
// 把PointField支持的整数/浮点类型统一读成double，兼容不同雷达布局。
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
  // 接入标准PointCloud2，兼容不同字段类型/端序/组织形式的雷达。
  CloudAdapter(ros::NodeHandle& n, const ModuleConfig& c, SampleSink s)
      : AdapterBase(n, c, std::move(s)) {
    if (!c.visual_topic.empty())
      visualization_ =
          nh_.advertise<sensor_msgs::PointCloud2>(c.visual_topic, 1);
    reconnect();
  }
  // 重建PointCloud2订阅以清除旧队列。
  void reconnect() override {
    subscriber_.shutdown();
    subscriber_ =
        nh_.subscribe(config_.topic, 20, &CloudAdapter::callback, this);
  }

 private:
  // 先验证维度、字段、缓冲区边界，再按端序读取XYZ统计有效点；异常结构不越界读取。
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
      double min = (ruleEnabled(config_.quality["range_m"]) ? get<double>(config_.quality["range_m"], "minimum", 0) : 0),
             max = (ruleEnabled(config_.quality["range_m"]) ? get<double>(config_.quality["range_m"], "maximum", std::numeric_limits<double>::infinity()) : std::numeric_limits<double>::infinity());
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
  // 订阅sensor_msgs/Imu；在接入层显式做单位转换，规则层统一使用SI。
  ImuAdapter(ros::NodeHandle& n, const ModuleConfig& c, SampleSink s)
      : AdapterBase(n, c, std::move(s)) {
    reconnect();
  }
  // 重建IMU订阅，不在实例切换后保留旧队列。
  void reconnect() override {
    subscriber_.shutdown();
    subscriber_ =
        nh_.subscribe(config_.topic, 1000, &ImuAdapter::callback, this);
  }

 private:
  template <class Array>
  // 按未知/不可用约定处理IMU协方差，必要时换算g²并检查对称/半正定。
  bool covariance(Sample& s, const Array& a, const std::string& name,
                  double scale) {
    auto c = config_.quality["covariance"];
    if (!ruleEnabled(c)) {
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
                                    get<bool>(c, "require_symmetric", false),
                                    get<bool>(c, "require_psd", false)),
           "nonfinite/asymmetric/non-PSD IMU covariance");
    return true;
  }
  // 原始IMU -> 单位换算 -> 可用性/有限值/选定协方差 -> 姿态四元数 -> 统一Sample。
  void callback(const sensor_msgs::Imu::ConstPtr& msg) {
    Sample s = sample(msg->header);
    auto q = config_.quality;
    auto req = q["required_measurements"];
    double scale =
        get<std::string>(q, "acceleration_unit", "mps2") == "g" ? 9.80665 : 1;
    const auto& a = msg->linear_acceleration;
    const auto& g = msg->angular_velocity;
    s.acceleration = Eigen::Vector3d(a.x, a.y, a.z) * scale;
    s.gyro = Eigen::Vector3d(g.x, g.y, g.z);
    s.values["imu_acceleration_unit"] =
        get<std::string>(q, "acceleration_unit", "mps2");
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
           accel_available || !get<bool>(req, "acceleration", false),
           "required acceleration estimate unavailable");
    binary(s, "IMU_GYRO_AVAILABLE",
           gyro_available || !get<bool>(req, "angular_velocity", false),
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
    s.orientation_available = msg->orientation_covariance[0] != -1;
    if ((get<bool>(req, "orientation", false) ||
         ruleEnabled(q["attitude"])) && s.orientation_available) {
      const auto& o = msg->orientation;
      Eigen::Quaterniond quat(o.w, o.x, o.y, o.z);
      s.orientation = quat;
      binary(s, "IMU_ORIENTATION_FINITE",
             quat.coeffs().allFinite() && std::isfinite(quat.coeffs().stableNorm()) &&
             quat.coeffs().stableNorm() > 1e-12 &&
             (ruleEnabled(q["attitude"]) || std::abs(quat.norm() - 1) < .1),
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
  // 选择Odometry或PoseStamped订阅；两者统一成为定位Sample。
  OdomAdapter(ros::NodeHandle& n, const ModuleConfig& c, SampleSink s)
      : AdapterBase(n, c, std::move(s)) {
    reconnect();
  }
  // 依据adapter重建定位订阅，丢弃旧实例连接。
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
  // 读取位置/四元数，执行基础有限性和可选归一化误差/期望坐标系检查。
  void populate(Sample& s, const geometry_msgs::Pose& p) {
    s.position = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
    s.orientation = Eigen::Quaterniond(p.orientation.w, p.orientation.x,
                                       p.orientation.y, p.orientation.z);
    s.orientation_available = true;
    binary(s, "LOC_POSE_FINITE",
           s.position.allFinite() && s.orientation.coeffs().allFinite(),
           "nonfinite pose");
    double norm = s.orientation.coeffs().stableNorm();
    binary(s, "LOC_QUATERNION_ZERO", std::isfinite(norm) && norm > 1e-12,
           "zero/nonfinite quaternion", norm);
    if (std::isfinite(norm) && norm > 1e-12) {
      auto q = config_.quality["quaternion_norm_error"];
      Bounds b = configuredBounds(q);
      if (b.enabled())
        finding(s, "LOC_QUATERNION_NORM", "quaternion normalization error",
                b.level(std::abs(norm - 1)), std::abs(norm - 1),
                b.level(std::abs(norm - 1)) == ERROR ? b.error : b.warning);

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
  // 拒绝latched旧定位，填充pose、child-frame twist与6x6协方差，再提交Sample。
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
  // 接入只有位姿的PoseStamped，明确标记协方差不可用，不伪造零噪声。
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
class RangeAdapter : public AdapterBase {
 public:
  // 订阅sensor_msgs/Range；默认可以直接接/mavros/rangefinder/rangefinder。
  RangeAdapter(ros::NodeHandle& n, const ModuleConfig& c, SampleSink s)
      : AdapterBase(n, c, std::move(s)) { reconnect(); }
  // 重建测距订阅；可选模块初始无数据不阻塞其他模块。
  void reconnect() override {
    subscriber_.shutdown();
    subscriber_ = nh_.subscribe(config_.topic, 20, &RangeAdapter::callback, this);
  }

 private:
  // 保留原始测距/量程/视场角与时间；明确这是波束距离，尚未做垂直投影或地形估计。
  void callback(const ros::MessageEvent<sensor_msgs::Range const>& event) {
    if (latched(*event.getConnectionHeaderPtr())) return;
    const auto m = event.getMessage();
    Sample s = sample(m->header);
    s.range_m = m->range;
    s.range_min_m = m->min_range;
    s.range_max_m = m->max_range;
    s.values["range_raw_m"] = std::isnan(m->range) ? "NaN" :
        std::isinf(m->range) ? (m->range > 0 ? "+Inf" : "-Inf") : number(m->range);
    s.values["range_sensor_min_m"] = number(m->min_range);
    s.values["range_sensor_max_m"] = number(m->max_range);
    s.values["range_field_of_view_rad"] = number(m->field_of_view);
    s.values["range_radiation_type"] = std::to_string(m->radiation_type);
    s.values["range_height_interpretation"] = "beam distance; vertical projection not calibrated";
    binary(s, "RANGE_FOV", std::isfinite(m->field_of_view) &&
        m->field_of_view >= 0 && m->field_of_view <= 6.283185307179586,
        "invalid field of view");
    sink_(s);
  }
};
class DiagnosticAdapter : public AdapterBase {
 public:
  // 接入感知等现有DiagnosticArray，用标准诊断扩展健康监测。
  DiagnosticAdapter(ros::NodeHandle& n, const ModuleConfig& c, SampleSink s)
      : AdapterBase(n, c, std::move(s)) {
    reconnect();
  }
  // 重建诊断输入订阅。
  void reconnect() override {
    subscriber_.shutdown();
    subscriber_ =
        nh_.subscribe(config_.topic, 100, &DiagnosticAdapter::callback, this);
  }

 private:
  // 校验诊断非空和等级合法，将上游异常转成统一Sample；不自行评价感知识别精度。
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
  // 同时接入nav_msgs/Path与任务状态；无目标时路径需求可暂停。
  PathAdapter(ros::NodeHandle& n, const ModuleConfig& c, SampleSink s)
      : AdapterBase(n, c, std::move(s)) {
    reconnect();
  }
  // 重建路径/任务订阅并清空旧目标与路径就绪状态。
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
  // 用单调时间探针检查所选任务心跳与有目标时的路径期限；即使路径没再发也能报警。
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
            ruleValue(config_.quality["task_state_timeout_sec"], 2);
    if (ruleEnabled(config_.quality["task_state_timeout_sec"]))
    binary(s, "PLANNING_TASK_FRESH", current, "task state missing or stale");
    if (task_receive_ > 0)
      binary(s, "PLANNING_GOAL_ID",
             !active_ || (!goal_.empty() && task_stamp_ > 0),
             "active goal requires nonempty goal_id and valid goal stamp");
    if (active_ && ruleEnabled(config_.quality["result_deadline_sec"]))
      binary(s, "PLANNING_DEADLINE",
             now - path_receive_ <=
                 ruleValue(config_.quality["result_deadline_sec"], 2),
             "active goal has no fresh path");
    else
      finding(s, "PLANNING_DEADLINE", "no active goal; path demand suspended",
              OK);
    s.values["synthetic_status"] = "true";
    s.values["required_checks_available"] =
        active_ && ruleEnabled(config_.quality["result_deadline_sec"]) && (!have_path_ ||
                    now - path_receive_ >
                        ruleValue(config_.quality["result_deadline_sec"], 2))
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
  // 接收任务目标和活动状态；目标变化后丢弃旧路径就绪证据。
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
  // 检查路径结构、有限位姿、参考系、选定段长及目标时间关系；不宣称完成碰撞安全检测。
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
                         ruleValue(config_.quality["maximum_segment_m"], 2);
      first = false;
      previous = pos;
    }
    binary(s, "PLANNING_PATH_FINITE", finite, "invalid path pose");
    binary(s, "PLANNING_PATH_FRAME", frames, "inconsistent path frames");
    if (ruleEnabled(config_.quality["maximum_segment_m"]))
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
// 依据adapter创建ROS订阅器；回调输出统一Sample，因此核心规则不依赖FAST-LIO。
std::unique_ptr<IInputAdapter> AdapterFactory::create(
    ros::NodeHandle& n, const ModuleConfig& c, IInputAdapter::SampleSink sink) {
  if (c.adapter == "LivoxCustomAdapter") {
#ifdef HEALTH_WITH_LIVOX
    return std::unique_ptr<IInputAdapter>(new LivoxAdapter(n, c, std::move(sink)));
#else
    throw std::invalid_argument("LivoxCustomAdapter requires BUILD_HEALTH_LIVOX_ADAPTER=ON and livox_ros_driver");
#endif
  }
  if (c.adapter == "PointCloud2Adapter")
    return std::unique_ptr<IInputAdapter>(
        new CloudAdapter(n, c, std::move(sink)));
  if (c.adapter == "RosImuAdapter")
    return std::unique_ptr<IInputAdapter>(
        new ImuAdapter(n, c, std::move(sink)));
  if (c.adapter == "RosRangeAdapter")
    return std::unique_ptr<IInputAdapter>(new RangeAdapter(n, c, std::move(sink)));
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
