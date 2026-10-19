/**
 * 与ROS传输层无关的健康检测核心接口和数据类型。
 *
 * 主要数据流：ConfigLoader读取YAML -> ModuleConfig创建ModuleMonitor -> ROS适配器
 * 把输入消息转换为Sample -> StreamTracker检查接收年龄/频率/时间戳，Checker检查
 * 单条数据质量 -> RuleLatch管理告警持续和恢复 -> ModuleReport供ROS节点发布、
 * RViz显示和EventRecorder记录。
 *
 * 这里不订阅ROS Topic，也不负责绘图或写文件。Adapter负责消息格式和单位转换；
 * 生命周期跟踪器决定当前是否应该收到数据；流检查与单样本质量检查分别运行。
 */
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

// 对外统一的健康等级数值；数值越大表示问题越严重。
enum Level { OK = 0, WARNING = 1, ERROR = 2 };
// 诊断、日志和界面共用的键值数据；值统一转成文本，便于不同输出复用。
using Values = std::map<std::string, std::string>;
// 返回单调时钟秒数；用于计算接收年龄、保持时间和steady_elapsed_sec，不受系统校时影响。
double steadyNow();
// 将double转成精度足够的文本；非有限值的处理由各输出格式负责。
std::string number(double value);
// 创建本次运行/命令所需的唯一标识；内部关联使用，不等同于传感器数据时间。
std::string uniqueId();
// 把Level枚举转换成OK/WARNING/ERROR文本，供诊断和日志使用。
const char* levelName(int level);

/**
 * 一条规则在当前检查周期的结果。
 * checker或StreamTracker生成Finding，RuleLatch使用id跟踪该规则的告警状态，
 * ModuleReport再把当前等级和原因提供给ROS发布与日志层。hard=true代表不可被
 * 普通软告警保持时间延迟的结构/数据错误。metric仅留作内部兼容接口，不写入新版日志。
 */
struct Finding {
  std::string id;      // 稳定规则编号，如RECEIVE_AGE、IMU_ACCELERATION。
  std::string reason;  // 面向诊断/日志阅读者的本次判断说明。
  std::string metric;  // 内部兼容字段；新版日志不输出该字段。
  std::string unit;    // value和limit对应的单位，如s、Hz、m/s²。
  int level = OK;      // 本次Finding等级，不含其他规则的聚合结果。
  double value = 0;    // 当前观测值；无法表示的数值可为NaN/Inf，由输出层编码。
  double limit = 0;    // 触发判断所用阈值，供报告和日志定位问题。
  bool hard = false;   // 是否按硬错误处理，例如无效结构或非法时间戳。
};
// 同一时刻由一个流或Checker产生的规则结果集合。
using Findings = std::vector<Finding>;
/**
 * 一个数值范围规则的两级阈值。
 * lower=false表示数值高于阈值告警；lower=true表示数值低于阈值告警。
 * WARNING和ERROR可独立启停；未配置某一级时不为该级补默认阈值。
 */
struct Bounds {
  double warning = 0;  // WARNING边界；比较使用严格大于/小于。
  double error = 0;    // ERROR边界；比较使用严格大于/小于。
  bool lower = false;  // false为上限检查，true为下限检查。
  // 两级阈值独立选择：只填写WARNING不会自动启用默认ERROR，反之亦然。
  bool warning_enabled = true, error_enabled = true;
  // 判断至少有一级阈值生效；这是配置状态查询，不执行数值检查。
  bool enabled() const { return warning_enabled || error_enabled; }
  // 按当前方向和已启用阈值返回OK/WARNING/ERROR；ERROR优先。
  int level(double value) const;
};
// 判断YAML规则块是否开启；缺失、null或enabled:false关闭，其他存在块默认开启。
bool ruleEnabled(const YAML::Node& node);
// 读取warning_below/above和error_below/above；lower指定阈值的比较方向。
// 未写的档位会关闭，不会填入默认阈值；已关闭规则保留的数字不会触发告警。
Bounds configuredBounds(const YAML::Node& node, bool lower = false);
// 读取单值配置，兼容标量和{enabled, value}结构；不存在或关闭时返回fallback。
double ruleValue(const YAML::Node& node, double fallback);
/**
 * 单路输入流使用的时间与频率规则设置。
 * age表示steady接收年龄；low_rate/high_rate检查频率；gap/duplicate/regression
 * 基于消息源时间戳。单位与频率基准由ConfigLoader从YAML解析并校验。
 */
struct StreamConfig {
  std::string frequency_basis = "source";       // source或receive，决定频率统计使用哪种时间。
  std::string stamp_domain = "device_relative"; // 时间戳时钟域说明，用于诊断解释，不负责时钟同步。
  // receive_age阈值、频率下限、频率上限和源时间戳间隔阈值。
  Bounds age{0.5, 1.5, false}, low_rate{5, 1, true}, high_rate{15, 30, false};
  Bounds gap{0.3, 1.0, false};
  int duplicate_level = WARNING;  // 重复源时间戳的告警级别；OK表示关闭。
  int regression_level = ERROR;   // 源时间戳倒退的告警级别；OK表示关闭。
  bool require_positive = true;   // 是否要求源时间戳为有限正数。
  bool stamp_checks_enabled = true;  // 源时间戳规则总开关；不关闭接收年龄探针。
};
// 单模块的生命周期时限和RuntimeState绑定名；这些参数控制等待过程，不是数据质量阈值。
struct LifecycleConfig {
  std::string binding;          // 控制器组名；相同binding的模块共享同一进程状态。
  double startup = 10, stop = 5; // 启动就绪和停止确认期限，单位秒。
  unsigned ready_samples = 10;  // 连续健康样本数；达到后才允许进入就绪状态。
};
/**
 * YAML中一个监控模块解析后的完整配置。
 * adapter负责消息到Sample的映射；checker负责Sample质量判断；stream负责流行为；
 * quality保存checker专属YAML规则。dependencies用于校验模块之间的先后/数据依赖。
 */
struct ModuleConfig {
  // 模块ID、C++检查器名、ROS适配器名和输入Topic等接入信息。
  std::string id, checker, adapter, topic, frame, child_frame, continuity;
  // 可选可视化输出、可视化坐标系和任务状态Topic。
  std::string visual_topic, visual_frame, task_topic;
  std::string missing_data_policy = "fault";  // 可选模块首帧前如何处理无数据。
  bool required = true;                       // 是否必须参与系统总体健康等级。
  // 仅对非必需模块且策略为no_data_until_first_sample时返回true。
  // 该策略只豁免首帧前NO_DATA；首帧收到后断流仍按常规规则判断。
  bool allowsInitialNoData() const {
    return !required && missing_data_policy == "no_data_until_first_sample";
  }
  std::vector<std::string> dependencies;  // 本模块要求存在/可用的其他模块ID。
  StreamConfig stream;                    // 接收、频率和源时间戳规则。
  LifecycleConfig lifecycle;              // 本模块运行/停止状态跟踪配置。
  YAML::Node quality;                     // checker专用阈值；由对应Checker解释。
};
/**
 * 运行一次健康监控所需的完整有效配置。
 * ConfigLoader将YAML中的可选项、默认值和模块设置汇总到这里，节点据此创建各模块、
 * 诊断发布器和日志器。source_text/effective_text用于记录用户原配置与本次生效配置。
 */
struct MonitorConfig {
  // 诊断发布频率及流统计窗口、最少样本/覆盖时间要求。
  double publish_rate = 5, window = 5, frequency_coverage = 1;
  unsigned frequency_samples = 3;
  // 软告警保持时间和恢复保持时间，单位秒。
  double soft_hold = 0.5, recovery_hold = 1;
  // RuntimeState控制要求和心跳超时阈值。
  bool strict_control = false;
  double heartbeat_warning = 1, heartbeat_error = 3;
  // 控制状态、标准诊断和简化等级三个ROS输出/输入Topic。
  std::string state_topic = "/robot_runtime/module_state";
  std::string diagnostics_topic = "/fastlio_health_monitor/diagnostics";
  std::string status_topic = "/fastlio_health_monitor/status";
  // 日志开关、目录、CSV采样/flush周期、队列及文件轮转/总预算限制。
  bool logging = true;
  std::string log_directory;
  double metrics_period = 1, flush_period = 1;
  size_t metrics_capacity = 4096, event_capacity = 512,
         rotate_bytes = 20 * 1024 * 1024;
  unsigned rotate_count = 10;
  size_t budget_bytes = 512 * 1024 * 1024;
  std::vector<ModuleConfig> modules;  // 通过校验并启用的模块列表。
  std::string source_path, source_text, effective_text;  // 配置追溯信息。
  std::string data_reference;  // 用户提供的数据集/运行来源说明，不代替时间戳。
};
/**
 * YAML配置加载与校验入口。
 * load()读取文件、验证允许的字段/单位/阈值/模块依赖、应用文档定义的运行默认值，
 * 并返回MonitorConfig。配置错误通过std::exception派生异常报告，节点应在启动时显示错误并退出。
 */
class ConfigLoader {
 public:
  // 加载指定YAML路径；成功时返回完整有效配置，失败时抛出带字段/原因的异常。
  static MonitorConfig load(const std::string& path);
};

/**
 * 所有消息适配器输出给健康核心的统一样本。
 * stamp是消息源时间，received是本机steadyNow()接收时间；两者不能混用。
 * 物理量在适配器中统一为SI单位。结构化字段供Checker计算，values供诊断/RViz/日志显示，
 * findings供适配器报告无法由Checker恢复的消息结构问题。
 */
struct Sample {
  double stamp = 0;     // ROS Header源时间戳，秒；用于stamp gap、频率和日志回溯。
  double received = 0;  // 单调时钟接收时间，秒；用于receive age与断流探针。
  bool valid = true;    // 适配器对消息整体结构/基础数值的初步判断。
  bool covariance_available = true;  // 定位消息是否实际提供协方差。
  std::string frame, child_frame;    // 消息声明的参考坐标系，不在核心内自动变换。
  uint64_t total = 0, valid_points = 0;  // 点云总点数和有限/有效点数。
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();  // 加速度，m/s²。
  Eigen::Vector3d gyro = Eigen::Vector3d::Zero();          // 角速度，rad/s。
  Eigen::Vector3d position = Eigen::Vector3d::Zero();       // 定位位置，m。
  Eigen::Vector3d twist = Eigen::Vector3d::Zero();          // 线速度或适配器约定的运动量。
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();  // 姿态四元数。
  bool orientation_available = false;  // 防止把默认单位四元数误当成真实姿态。
  double range_m = 0, range_min_m = 0, range_max_m = 0;  // 测距值及消息声明范围，m。
  Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Zero();
  Findings findings;  // 适配器发现的消息层问题，进入模块规则聚合。
  Values values;      // 可读指标/原始单位等展示字段，供诊断、RViz和日志复用。
};

/**
 * 单路输入消息流的时间和频率统计器。
 * receive()由消息回调调用，立即检查源时间戳重复/倒退/间隔并记录接收时间；
 * evaluate()由健康节点定时调用，即使没有新消息也能检查receive age并更新频率就绪状态。
 * 因此STAMP_GAP等源时间戳间隔问题要等下一条消息到达，而RECEIVE_AGE由定时探针发现断流。
 */
class StreamTracker {
 public:
  // 使用模块流规则和全局统计窗口构造；now使用单调时钟秒数。
  StreamTracker(const StreamConfig& config, const MonitorConfig& monitor);
  // 记录一个新消息的源stamp和本机接收时间；返回本次可立即确认的源时间戳问题。
  Findings receive(double stamp, double received);
  // 定时更新年龄/频率诊断数值和ready；没有收到新消息时也必须持续调用。
  Findings evaluate(double now, Values& values, bool& ready);
  // 清空窗口与时间基线，用now作为新一轮启动时间。
  void reset(double now);
  // 查询自上次reset以来是否至少接收过一条样本。
  bool received() const { return count_ > 0; }
  // 最近一次接收的steady时间；没有数据时为0，供生命周期/告警展示使用。
  double lastReceive() const { return last_receive_; }
  // 最近一个有效且递增的源时间戳；不能用它计算本机接收年龄。
  double lastStamp() const { return last_stamp_; }

 private:
  StreamConfig config_;
  double window_, coverage_, started_ = 0, last_receive_ = 0, last_stamp_ = 0;
  unsigned min_samples_;
  uint64_t count_ = 0;
  std::deque<double> receipts_;
  std::deque<std::pair<double, double>> sources_;
};

/**
 * 按规则ID分别管理告警状态的保持器。
 * apply()收到当前规则结果后，软告警可按soft_hold延迟确认、恢复可按recovery_hold延迟；
 * 硬错误会保留到至少一次诊断发布，避免瞬时结构错误被发布周期吞掉。保持器不修改Sample，
 * 仅改变规则状态的生效时刻，并通过EventSink报告等级变化。
 */
class RuleLatch {
 public:
  using EventSink =
      std::function<void(const std::string&, const Finding&, int, int)>;
  // 创建规则保持器；sink在某条规则生效等级变化时接收事件回调。
  RuleLatch(double soft_hold, double recovery_hold, EventSink sink);
  // 提交本周期全部finding并更新时间状态；now应使用单调时钟。
  void apply(const Findings& findings, double now);
  // 返回当前所有规则中的最高生效等级。
  int level() const;
  // 返回当前发布周期需要保留的最高等级，包含尚未发布过的瞬时硬错误。
  int periodLevel() const;
  // 汇总当前活动规则原因，供ModuleReport.message使用。
  std::string reasons() const;
  // 清空所有规则保持状态；实例重启/数据代际切换时使用。
  void reset();
  // 通知保持器当前周期已发布，清除只需保留一个发布周期的硬错误标记。
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

/**
 * 传感器/算法模块质量检查器基类。
 * Checker接收已完成消息适配的Sample，生成Finding和人类可读Values；它不订阅ROS，
 * 也不自己实现输入频率/接收年龄探针。派生类按模块类型实现check()。
 */
class IHealthChecker {
 public:
  // 虚析构确保由工厂创建并通过基类unique_ptr管理的Checker正确释放。
  virtual ~IHealthChecker() = default;
  // 检查当前样本；返回本帧质量问题并向values追加用于诊断/日志的数值。
  virtual Findings check(const Sample& sample, Values& values) = 0;
  // 清除跨帧计算所需历史；无历史状态的Checker可保持默认空实现。
  virtual void reset() {}
};
// 检查总点数、有效点数和有效比例；消息具体解包已由PointCloud适配器完成。
class LidarChecker : public IHealthChecker {
 public:
  // 保存quality.lidar规则；创建阶段读取阈值，后续每帧无需访问ROS参数服务器。
  explicit LidarChecker(const YAML::Node& quality);
  // 计算并检查当前点云计数，返回触发规则和实际计数。
  Findings check(const Sample& sample, Values& values) override;

 private:
  Bounds total_, valid_, ratio_;
};
// Shared by IMU and localization; no ROS or driver-specific dependency.
/**
 * 跨消息姿态质量跟踪器，供IMU与定位Checker共用。
 * 检查四元数有效性、roll/pitch范围、相邻姿态变化及配置允许的坐标系条件。
 * IMU与定位只有在参考系可比时才能做差异判断；本类不会擅自将坐标系不一致的角度相减。
 */
class AttitudeTracker {
 public:
  // quality是该输入的姿态规则，prefix区分输出指标名称（例如IMU/LOC）。
  AttitudeTracker(const YAML::Node& quality, const std::string& prefix);
  // 检查一帧姿态并维护上一有效姿态；缺少姿态时按配置报告不可用/跳过。
  Findings update(const Sample& sample, Values& values);
  // 清除姿态差分基线；重启后下一帧只建立新基线，不跨运行比较。
  void reset() { have_previous_ = false; }

 private:
  YAML::Node config_;
  std::string prefix_;
  bool have_previous_ = false;
  double stamp_ = 0;
  Eigen::Quaterniond previous_ = Eigen::Quaterniond::Identity();
  std::string frame_, child_frame_;
};
// 检查SI单位下的加速度/角速度幅值并组合姿态规则；单位转换只在适配器进行一次。
class ImuChecker : public IHealthChecker {
 public:
  // 读取IMU质量规则并建立姿态跟踪器。
  explicit ImuChecker(const YAML::Node& quality);
  // 检查当前加速度、角速度和可用姿态，输出幅值/姿态指标及异常规则。
  Findings check(const Sample& sample, Values& values) override;
  // 清除姿态历史；幅值规则本身不依赖上一帧。
  void reset() override { attitude_.reset(); }

 private:
  Bounds acceleration_, gyro_;
  AttitudeTracker attitude_;
};
/**
 * 对Range消息的原始距离进行有效性、范围、变化率和连续性检查。
 * 输出代表传感器沿自身测量方向的距离；没有安装姿态/外参标定时不能直接解释为垂直离地高度。
 */
class RangeChecker : public IHealthChecker {
 public:
  // 保存测距质量规则；阈值在check()中按本项目配置格式读取。
  explicit RangeChecker(const YAML::Node& quality) : quality_(quality) {}
  // 检查本帧测距值并与上一有效样本比较连续性。
  Findings check(const Sample& sample, Values& values) override;
  // 清除距离差分基线，避免跨定位/传感器重启计算跳变。
  void reset() override { have_previous_ = false; }

 private:
  YAML::Node quality_;
  bool have_previous_ = false;
  double stamp_ = 0, range_ = 0;
  std::string frame_;
};
/**
 * 定位/IMU协方差矩阵结构检查工具。
 * 可检查矩阵元素是否有限、是否近似对称、是否满足半正定；返回值只说明矩阵数值合法，
 * 不代表传感器估计一定准确或当前定位可用。
 */
class CovarianceChecker {
 public:
  // tolerance为浮点比较容差；symmetry/psd分别选择对称性和半正定性检查。
  static bool valid(const Eigen::MatrixXd& matrix, double tolerance,
                    bool symmetry, bool psd);
};
/**
 * 连续位姿/速度跟踪器。
 * 根据相邻有效样本和时间间隔估计速度、加速度与角速度，检查运动上限和位姿跳变。
 * 时间戳不递增、位置坐标系变化或姿态无效时会重建/拒绝差分，避免把不相容状态作比较。
 */
class KinematicsTracker {
 public:
  // 检查当前定位样本并更新运动学基线；将计算指标写入values。
  Findings update(const Sample& sample, const YAML::Node& quality,
                  Values& values);
  // 清除位置、速度、姿态和源时间基线。
  void reset();

 private:
  bool have_pose_ = false, have_velocity_ = false;
  double stamp_ = 0, midpoint_ = 0;
  std::string frame_;
  Eigen::Vector3d position_ = Eigen::Vector3d::Zero(),
                  velocity_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
};
/**
 * Odometry/PoseStamped定位输出的综合质量检查器。
 * 每帧可检查位置和姿态有效性、速度/加速度/角速度、相邻跳变及协方差；实际启用项由quality决定。
 * 它只评估定位消息本身，不启动或管理被监控的定位进程。
 */
class LocalizationChecker : public IHealthChecker {
 public:
  // 保存定位质量规则，分别创建运动学跟踪器和定位姿态跟踪器。
  explicit LocalizationChecker(const YAML::Node& quality)
      : quality_(quality), attitude_(quality["attitude"], "LOC") {}
  // 检查一条定位输出，返回异常并生成位置、姿态和运动学指标。
  Findings check(const Sample& sample, Values& values) override;
  // 定位节点重启或数据代际切换后清除所有跨帧基线。
  void reset() override { motion_.reset(); attitude_.reset(); }

 private:
  YAML::Node quality_;
  KinematicsTracker motion_;
  AttitudeTracker attitude_;
};
// 转发适配器已生成的Finding/Values；用于接入上游已计算的诊断和规划任务规则。
class RelayChecker : public IHealthChecker {
 public:
  // 不重复计算阈值，直接把Sample携带的上游结果与显示值交给模块聚合层。
  Findings check(const Sample& sample, Values& values) override {
    values = sample.values;
    return sample.findings;
  }
};
/**
 * 质量检查器工厂：把ModuleConfig.checker字符串转换成对应C++对象。
 * 支持范围集中在此处；不支持的名称作为配置/启动错误处理，避免悄悄退回错误规则。
 */
class CheckerFactory {
 public:
  // 创建checker并读取其quality参数；返回对象由ModuleMonitor持有。
  static std::unique_ptr<IHealthChecker> create(const ModuleConfig& config);
};

/**
 * 控制器报告的单次运行事实。
 * LifecycleTracker使用controller/instance/sequence/epoch拒绝旧进程或乱序心跳；
 * 这是内部控制协议。它与传感器健康Finding分开处理，日志字段精简不改变校验行为。
 */
struct RuntimeUpdate {
  std::string controller;  // 发布控制状态的控制器名称。
  std::string instance;    // 当前被管理进程实例ID，用于排除旧进程消息。
  std::string command;     // 本次start/stop/pause操作ID。
  std::string desired;     // 控制器请求状态，如RUN或STOP。
  std::string observed;    // 控制器观察到的进程/ROS节点状态。
  std::string reason;      // 状态变化原因，供诊断说明。
  uint64_t sequence = 0;   // 同一控制器内递增序号，拒绝乱序旧心跳。
  uint64_t frame_epoch = 0; // 数据代际；节点重启后变化，防止跨代沿用旧状态。
  bool trace_healthy = true; // 控制器日志/追溯链是否正常。
  std::string controller_run_id, controller_log_directory, data_reference,
      data_sha256;
};
// 当前模块的进程/控制生命周期结论；与数据质量等级分开，最终由ModuleMonitor组合。
struct LifecycleReport {
  std::string state;  // 如STARTING、RUNNING、PAUSED、STOPPED、UNKNOWN或FAULT。
  bool expected_active = true, valid_state = false;  // 当前是否预期有输入、状态消息是否合法。
  Findings findings;  // 控制心跳或启停时限相关的规则结果。
};
/**
 * 处理RuntimeState的生命周期状态机。
 * 根据控制意图、观察状态和心跳时间判断模块是否RUNNING/PAUSED/STOPPED等；只有收到
 * 有效控制意图后才把没有数据解释成计划停止。请求RUN后允许启动等待期，成功收到数据
 * 后要求连续就绪样本；这样定位节点正常关闭再重启不会被简单当成永久断流或立即报错。
 */
class LifecycleTracker {
 public:
  // 用模块生命周期配置、全局心跳策略和本轮steady时间初始化状态机。
  LifecycleTracker(const LifecycleConfig& config, const MonitorConfig& monitor,
                   double now);
  // 接受并校验新的控制事实；返回该控制更新是否通过身份/序号/状态校验。
  bool control(const RuntimeUpdate& update, double now);
  // 查询最近一次control()是否接受了控制消息。
  bool controlAccepted() const { return control_accepted_; }
  // 告诉状态机当前样本质量是否良好，用于连续就绪计数。
  void sample(bool good, double now);
  // 标记新一轮输入启动，重置本轮启动等待计时。
  void beginInput(double now);
  // 依据当前时间、流统计准备状态和接收年龄计算生命周期报告。
  // wait_for_first_sample用于可选输入首帧前维持NO_DATA，而不启动断流倒计时。
  LifecycleReport evaluate(double now, bool stream_ready, double receive_age,
                           bool wait_for_first_sample = false);
  // 获取最近控制器确认的实例身份，用于诊断和拒绝旧实例消息。
  std::string instance() const { return update_.instance; }
  // 获取当前操作命令ID，用于把请求、状态变化和结束事件关联起来。
  std::string command() const { return update_.command; }
  // 获取当前数据代际；内部控制/RViz使用，不作为新版日志字段。
  uint64_t epoch() const { return epoch_; }
  // 获取最近接受的完整控制事实；只读，不会推进状态机。
  const RuntimeUpdate& runtime() const { return update_; }
  // 当前处于运行/故障/未知状态时要求持续输入；PAUSED/STOPPED不按断流告警。
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

/**
 * ModuleMonitor每次evaluate()生成的模块快照。
 * 同一对象供DiagnosticArray、简化状态汇总、RViz和日志层使用，避免各输出各自重算状态。
 * level是生命周期与数据质量合并后的模块等级；data_level只表示输入质量；required和
 * contributes_to_overall说明该模块是否参与系统总体等级。
 */
struct ModuleReport {
  std::string id;       // 模块唯一ID，与YAML配置一致。
  std::string checker;  // 使用的C++检查器名称。
  std::string topic;    // 输入Topic，帮助定位数据来源。
  std::string state;    // 生命周期状态，如RUNNING、PAUSED或STOPPED。
  std::string instance, command;  // 最近接受的进程实例和控制命令标识。
  std::string message;  // 面向人阅读的状态/规则原因摘要。
  uint64_t epoch = 1;   // 当前数据代际；用于内部状态和界面，不写入新版日志。
  int level = WARNING;  // 合并数据与生命周期后的模块等级。
  int data_level = OK;  // 仅传感器/算法输出质量等级。
  bool required = true;  // 是否为总体健康必需模块。
  bool has_data = false; // 本轮生命周期是否收到过数据。
  bool valid = false;    // 当前报告对应数据是否有效。
  bool expected_active = true;  // 当前状态是否预期持续产生输入。
  bool contributes_to_overall = true;  // 是否计入系统总体健康汇总。
  Values values;  // 本模块当前指标，供诊断、面板和日志共同使用。
};
/**
 * 单模块健康检测的协调器。
 * receive()把新Sample送到时间跟踪器和checker；control()处理RuntimeState；evaluate()
 * 周期性检查接收年龄/频率、生命周期、规则保持状态并形成ModuleReport。
 * 一般一个配置模块对应一个ModuleMonitor，由RobotHealthMonitor节点持有。
 */
class ModuleMonitor {
 public:
  using EventSink = RuleLatch::EventSink;
  // 创建协调器和其Checker/生命周期/规则保持器；sink接收规则等级变化事件。
  ModuleMonitor(const ModuleConfig& config, const MonitorConfig& monitor,
                EventSink sink, double now);
  // 处理一条已经由适配器统一单位和格式的输入样本。
  void receive(const Sample& sample);
  // 把控制器状态交给生命周期跟踪器；返回更新是否被接受。
  bool control(const RuntimeUpdate& update, double now);
  // 定时评估当前模块并生成可发布/记录的快照；即使没有新消息也必须调用。
  ModuleReport evaluate(double now);
  // 读取本模块启动时加载的有效配置；修改YAML需重启节点才会生效。
  const ModuleConfig& config() const { return config_; }
  // 读取最近样本的原始/派生显示值，供规则事件添加可追溯上下文。
  const Values& sampleValues() const { return values_; }
  // 必需模块始终计入总体；可选且允许首帧NO_DATA的模块从首帧到达后开始计入。
  bool contributesToOverall() const {
    return config_.required || (config_.allowsInitialNoData() && have_data_);
  }
  // 获取控制器确认的模块实例ID；用于诊断和区分定位进程重启前后。
  std::string instance() const { return lifecycle_.instance(); }
  // 获取关联本轮启停操作的命令ID。
  std::string command() const { return lifecycle_.command(); }
  // 获取数据代际，仅供内部控制校验和界面展示使用。
  uint64_t epoch() const { return lifecycle_.epoch(); }
  // 获取最近接受的RuntimeState事实；未接受控制时内容为默认值。
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
