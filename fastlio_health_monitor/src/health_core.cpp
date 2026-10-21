/**
 * 健康检测的C++业务核心实现，不依赖ROS订阅、RViz控件或具体传感器驱动。
 *
 * 初始化数据流：YAML文本 -> ConfigLoader检查/规范化 -> ModuleConfig -> CheckerFactory。
 * 运行数据流：ROS适配器生成Sample -> StreamTracker检查消息流 -> IHealthChecker检查
 * 单样本/跨样本质量 -> RuleLatch管理告警保持 -> LifecycleTracker解释运行意图
 * -> ModuleMonitor生成统一ModuleReport。
 *
 * 时间必须分开理解：消息头stamp用于源时间戳重复、倒退、间隔及按源时间计算的频率；
 * steady接收时间用于receive age和心跳超时。前者要在消息到来时比较，后者由定时
 * evaluate()持续检查，所以在没有下一条消息时也能及时报告断流。
 */
#include <fastlio_health_monitor/health_core.h>
#include <unistd.h>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <limits>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>

namespace robot_health {
namespace {
/**
 * 检查一个YAML映射块的形状和字段白名单。
 * 先确认节点是mapping，再检查enabled是否能转换为bool，最后逐个拒绝未知键。
 * 这样用户拼错字段时会在启动加载阶段得到异常，而不是程序静默采用默认值。
 */
void keys(const YAML::Node& node, const std::string& path,
          std::initializer_list<const char*> allowed) {
  if (!node || node.IsNull()) return;
  if (!node.IsMap())
    throw std::invalid_argument(path + " must be a mapping");
  std::set<std::string> names;
  for (const auto* s : allowed) names.insert(s);
  if (names.count("enabled") && node["enabled"] && !node["enabled"].IsNull()) node["enabled"].as<bool>();
  for (const auto& entry : node) {
    if (!names.count(entry.first.as<std::string>()))
      throw std::invalid_argument(path + ": unsupported field " +
                                  entry.first.as<std::string>());
  }
}
template <class T>
// 读取可选标量：字段缺失或显式null时返回fallback；字段存在但类型错误时让YAML转换异常上报。
// fallback是程序运行参数默认值，不用于推断传感器型号、坐标系或物理单位。
T get(const YAML::Node& node, const char* key, T fallback) {
  return node && node[key] && !node[key].IsNull() ? node[key].as<T>() : fallback;
}
// 读取必须有限的运行数值；zero=true允许0，否则必须大于0。
// 统一在加载配置时拒绝NaN/Inf/负数，避免错误的时间窗、周期或容量进入运行循环。
double positive(const YAML::Node& n, const char* key, double fallback,
                bool zero = false) {
  double v = get<double>(n, key, fallback);
  if (!std::isfinite(v) || v < 0 || (!zero && v == 0))
    throw std::invalid_argument(std::string(key) + " must be finite and " +
                                (zero ? "nonnegative" : "positive"));
  return v;
}
/**
 * 把YAML数值规则解析成Bounds。
 * 方向由lower决定：上限类只接受warning_above/error_above，下限类只接受
 * warning_below/error_below。只有用户实际写出的档位才启用；enabled:false关闭整块，
 * 但保留阈值数字供文档/配置查看。双档同时启用时校验WARNING与ERROR顺序。
 * defaults只提供未使用字段的内部数值，不会因此打开缺失的告警级别。
 */
Bounds bounds(const YAML::Node& n, bool lower, Bounds defaults) {
  Bounds b = defaults;
  b.lower = lower;
  b.warning_enabled = b.error_enabled = false;
  if (!n || n.IsNull()) return b;
  keys(n, "bounds", lower
      ? std::initializer_list<const char*>{"enabled", "warning_below", "error_below"}
      : std::initializer_list<const char*>{"enabled", "warning_above", "error_above"});
  const char* warn = lower ? "warning_below" : "warning_above";
  const char* error = lower ? "error_below" : "error_above";
  const bool enabled = get<bool>(n, "enabled", true);
  b.warning_enabled = enabled && n[warn] && !n[warn].IsNull();
  b.error_enabled = enabled && n[error] && !n[error].IsNull();
  if (n[warn] && !n[warn].IsNull()) b.warning = positive(n, warn, defaults.warning, true);
  if (n[error] && !n[error].IsNull()) b.error = positive(n, error, defaults.error, true);
  if (b.warning_enabled && b.error_enabled &&
      ((lower && b.error >= b.warning) || (!lower && b.warning >= b.error)))
    throw std::invalid_argument("threshold ordering invalid");
  return b;
}
/**
 * 解析只有一个等级的枚举规则，例如时间戳重复或Range无回波。
 * 支持旧字符串写法和新映射写法{enabled, level}；关闭映射返回OK，未知等级报配置错误。
 */
int parseLevel(const YAML::Node& n, const char* key, const char* def) {
  auto value = n && n[key] ? n[key] : YAML::Node();
  std::string text = def;
  if (value && !value.IsNull()) {
    if (value.IsMap()) {
      keys(value, key, {"enabled", "level"});
      if (!get<bool>(value, "enabled", true)) return OK;
      text = get<std::string>(value, "level", def);
    } else text = value.as<std::string>();
  }
  if (text == "WARNING") return WARNING;
  if (text == "ERROR") return ERROR;
  throw std::invalid_argument(std::string(key) + " must be WARNING or ERROR");
}
/**
 * 递归合并共享默认配置与单模块覆盖。
 * 两边都是mapping时逐字段合并，其他情况由模块值整体覆盖默认值；显式null也作为
 * 覆盖值保留，之后由规则解析器解释为关闭/无配置，不能再悄悄继承回检测阈值。
 */
YAML::Node merge(const YAML::Node& base, const YAML::Node& override) {
  YAML::Node out = YAML::Clone(base);
  if (!out || out.IsNull()) out = YAML::Node(YAML::NodeType::Map);
  if (override) {
    if (!override.IsMap())
      throw std::invalid_argument("override must be a mapping");
    for (const auto& item : override) {
      auto name = item.first.as<std::string>();
      if (out[name] && out[name].IsMap() && item.second.IsMap())
        out[name] = merge(out[name], item.second);
      else
        out[name] = YAML::Clone(item.second);
    }
  }
  return out;
}
// 统一创建Finding，避免各Checker遗漏规则编号、可读原因、单位或硬错误标记。
// hard用于表示该结果不经过普通软告警保持时间，例如消息结构损坏或非法时间戳。
void add(Findings& out, const std::string& id, const std::string& reason,
         int level, double value = 0, double limit = 0,
         const std::string& unit = "", bool hard = false) {
  out.push_back(Finding{id, reason, id, unit, level, value, limit, hard});
}
// 对一个Bounds执行数值比较。未选择任何阈值时不生成Finding；非有限观测值按ERROR处理。
// 生成的Finding同时保存当前值、命中的档位、单位与对应阈值，供诊断和日志定位。
void threshold(Findings& out, const std::string& id, double value,
               const Bounds& b, const std::string& unit) {
  if (!b.enabled()) return;
  int l = std::isfinite(value) ? b.level(value) : ERROR;
  add(out, id, l ? "outside configured range" : "within range", l, value,
      l == ERROR ? b.error : b.warning, unit, !std::isfinite(value));
}
// 查找一组Finding中是否存在ERROR；用于阻止坏时间戳/坏样本污染跨帧基线和启动就绪计数。
bool anyError(const Findings& f) {
  return std::any_of(f.begin(), f.end(),
                     [](const Finding& v) { return v.level == ERROR; });
}
/**
 * 校验IMU或定位协方差规则的字段及取值。
 * IMU配置包含加速度协方差单位与未知值策略；定位配置包含零矩阵策略及位置/姿态方差阈值。
 * 本函数只检查规则定义是否自洽，不接触Sample，也不判断某条协方差是否有效。
 */
void validateCovariance(const YAML::Node& c, bool imu) {
  if (!c || c.IsNull()) return;
  if (imu)
    keys(
        c, "covariance",
        {"enabled", "required_for_valid", "unknown_policy", "acceleration_unit",
         "require_symmetric", "require_psd", "numeric_tolerance"});
  else
    keys(c, "covariance",
         {"enabled", "required_for_valid", "zero_matrix_policy",
          "require_symmetric", "require_psd", "numeric_tolerance",
          "position_variance_m2", "orientation_variance_rad2"});
  positive(c, "numeric_tolerance", 1e-9);
  for (const char* name :
       {"enabled", "required_for_valid", "require_symmetric", "require_psd"})
    if (c[name] && !c[name].IsNull()) c[name].as<bool>();
  if (imu) {
    const auto unit = get<std::string>(c, "acceleration_unit", "input_squared");
    if (unit != "input_squared" && unit != "si_squared")
      throw std::invalid_argument("covariance.acceleration_unit invalid");
    const auto p = get<std::string>(c, "unknown_policy", "report_only");
    if (p != "report_only" && p != "warning" && p != "error")
      throw std::invalid_argument("unknown_policy invalid");
  } else {
    const auto p = get<std::string>(c, "zero_matrix_policy", "unknown_warning");
    if (p != "unknown_warning" && p != "report_only" && p != "valid")
      throw std::invalid_argument("zero_matrix_policy invalid");
    bounds(c["position_variance_m2"], false, {1, 4, false});
    bounds(c["orientation_variance_rad2"], false, {.1, .5, false});
  }
}
/**
 * 校验通用姿态规则的字段、角度域和差分时间窗。
 * roll/pitch/rotation_jump/angular_speed配置使用度制；差分时间使用秒。这里限制Euler
 * 角度可达范围并保证dt_min小于dt_max，避免把弧度误填进角度字段或产生无效差分窗。
 */
void validateAttitude(const YAML::Node& a) {
  if (!a || a.IsNull()) return;
  keys(a, "attitude", {"enabled", "require_available", "quaternion_norm_error",
                       "roll_abs_deg", "pitch_abs_deg", "rotation_jump_deg",
                       "angular_speed_dps", "dt_min_sec", "dt_max_sec"});
  for (const auto* k : {"enabled", "require_available"})
    if (a[k] && !a[k].IsNull()) a[k].as<bool>();
  bounds(a["quaternion_norm_error"], false, {.02, .1, false});
  const auto roll = bounds(a["roll_abs_deg"], false, {30, 45, false});
  const auto pitch = bounds(a["pitch_abs_deg"], false, {30, 45, false});
  if (roll.error > 180 || pitch.error > 90)
    throw std::invalid_argument("attitude angle bounds exceed Euler angle domain");
  const auto jump = bounds(a["rotation_jump_deg"], false, {30, 60, false});
  if (jump.error > 180)
    throw std::invalid_argument("attitude rotation jump exceeds 180 degrees");
  bounds(a["angular_speed_dps"], false, {180, 360, false});
  if (positive(a, "dt_min_sec", .0001) >= positive(a, "dt_max_sec", 1))
    throw std::invalid_argument("attitude dt bounds invalid");
}
/**
 * 按Checker类型验证quality字段、量纲、阈值关系及适配器组合。
 * 每个分支只检查该Checker认识的字段；可省略业务检测块，但不能把IMU规则配给点云
 * 消息适配器。函数在创建运行对象前执行，错误会阻止节点以不匹配的配置启动。
 */
void qualityValidate(const ModuleConfig& m) {
  const auto q = m.quality;
  if (m.checker == "LidarChecker") {
    keys(q, m.id + ".quality",
         {"points_total", "points_valid", "valid_ratio", "range_m"});
    bounds(q["points_total"], true, {100, 1, true});
    bounds(q["points_valid"], true, {100, 1, true});
    auto b = bounds(q["valid_ratio"], true, {.8, .5, true});
    if ((b.warning_enabled && b.warning > 1) || (b.error_enabled && b.error > 1))
      throw std::invalid_argument("valid_ratio exceeds 1");
    auto range = q["range_m"];
    keys(range, "range_m", {"enabled", "minimum", "maximum"});
    if (positive(range, "minimum", .0001, true) >=
        positive(range, "maximum", std::numeric_limits<double>::max()))
      throw std::invalid_argument("range order invalid");
    if (m.adapter != "LivoxCustomAdapter" && m.adapter != "PointCloud2Adapter")
      throw std::invalid_argument("unsupported lidar adapter");
  } else if (m.checker == "ImuChecker") {
    keys(q, m.id + ".quality",
         {"acceleration_unit", "angular_velocity_unit", "acceleration_content",
          "required_measurements", "acceleration_norm_mps2",
          "angular_velocity_norm_rps", "covariance", "attitude"});
    const auto u = get<std::string>(q, "acceleration_unit", "mps2");
    if (u != "g" && u != "mps2")
      throw std::invalid_argument("acceleration_unit must be g or mps2");
    if (get<std::string>(q, "angular_velocity_unit", "radps") != "radps")
      throw std::invalid_argument("only radps is supported");
    auto content =
        get<std::string>(q, "acceleration_content", "specific_force");
    if (content != "specific_force" && content != "gravity_removed")
      throw std::invalid_argument("invalid acceleration_content");
    if (q["required_measurements"] && !q["required_measurements"].IsNull())
      keys(q["required_measurements"], "required_measurements",
           {"acceleration", "angular_velocity", "orientation"});
    if (q["required_measurements"] && !q["required_measurements"].IsNull())
      for (const auto& field : q["required_measurements"])
        field.second.as<bool>();
    bounds(q["acceleration_norm_mps2"], false, {100, 200, false});
    bounds(q["angular_velocity_norm_rps"], false, {35, 70, false});
    validateCovariance(q["covariance"], true);
    validateAttitude(q["attitude"]);
    if (m.adapter != "RosImuAdapter")
      throw std::invalid_argument("unsupported imu adapter");
  } else if (m.checker == "LocalizationChecker") {
    keys(q, m.id + ".quality",
         {"expected_pose_frame", "expected_child_frame", "continuity",
          "quaternion_norm_error", "kinematics", "covariance", "jump", "attitude"});
    validateAttitude(q["attitude"]);
    if (m.adapter != "OdometryAdapter" && m.adapter != "PoseStampedAdapter")
      throw std::invalid_argument("unsupported localization adapter");
    auto k = q["kinematics"] ? YAML::Clone(q["kinematics"]) : YAML::Node(YAML::NodeType::Map);
    if (!k || k.IsNull()) k = YAML::Node(YAML::NodeType::Map);
    keys(k, "kinematics",
         {"enabled", "velocity_source", "dt_min_sec", "dt_max_sec", "speed_mps",
          "acceleration_mps2", "angular_speed_rps"});
    const auto source =
        get<std::string>(k, "velocity_source", "position_difference");
    if (source != "position_difference" && source != "twist")
      throw std::invalid_argument("velocity_source invalid");
    if (m.adapter == "PoseStampedAdapter" && source == "twist")
      throw std::invalid_argument("PoseStamped has no twist");
    auto continuity = get<std::string>(q, "continuity", "continuous_odometry");
    if (continuity != "continuous_odometry" &&
        continuity != "global_localization")
      throw std::invalid_argument("continuity invalid");
    if (continuity == "global_localization" && source == "position_difference")
      throw std::invalid_argument(
          "global_localization requires trusted twist for physical motion; "
          "position differencing unsupported");
    if (positive(k, "dt_min_sec", .001) >= positive(k, "dt_max_sec", 1))
      throw std::invalid_argument("dt bounds invalid");
    bounds(q["quaternion_norm_error"], false, {.02, .1, false});
    bounds(ruleEnabled(k) ? k["speed_mps"] : YAML::Node(), false, {15, 50, false});
    bounds(ruleEnabled(k) ? k["acceleration_mps2"] : YAML::Node(), false, {10, 30, false});
    bounds(ruleEnabled(k) ? k["angular_speed_rps"] : YAML::Node(), false, {3, 10, false});
    validateCovariance(q["covariance"], false);
    auto jump = q["jump"];
    keys(jump, "jump", {"enabled", "residual_m", "rotation_rad"});
    if (jump && !jump.IsNull()) {
      bounds(jump["residual_m"], false, {.5, 2, false});
      bounds(jump["rotation_rad"], false, {.3, 1, false});
    }
  } else if (m.checker == "RangeChecker") {
    keys(q, m.id + ".quality", {"valid_range_m", "no_return_level",
                                "height_limits", "continuity"});
    if (m.adapter != "RosRangeAdapter")
      throw std::invalid_argument("RangeChecker requires RosRangeAdapter");
    keys(q["valid_range_m"], "valid_range_m", {"enabled", "minimum", "maximum"});
    if (positive(q["valid_range_m"], "minimum", 0, true) >=
        positive(q["valid_range_m"], "maximum", std::numeric_limits<double>::max()))
      throw std::invalid_argument("valid_range_m ordering invalid");
    parseLevel(q, "no_return_level", "WARNING");
    auto h = q["height_limits"];
    if (h && !h.IsNull()) {
      keys(h, "height_limits", {"enabled", "minimum_m", "maximum_m"});
      if (h["enabled"] && !h["enabled"].IsNull()) h["enabled"].as<bool>();
      auto lo = bounds(h["minimum_m"], true, {.2, .1, true});
      auto hi = bounds(h["maximum_m"], false, {20, 30, false});
      if (lo.warning >= hi.warning)
        throw std::invalid_argument("height limit ranges overlap");
    }
    auto c = q["continuity"];
    if (c && !c.IsNull()) {
      keys(c, "range continuity", {"enabled", "dt_min_sec", "dt_max_sec",
                                   "jump_m", "rate_abs_mps"});
      if (c["enabled"] && !c["enabled"].IsNull()) c["enabled"].as<bool>();
      if (positive(c, "dt_min_sec", .01) >= positive(c, "dt_max_sec", 1.2))
        throw std::invalid_argument("range continuity dt bounds invalid");
      bounds(c["jump_m"], false, {1, 3, false});
      bounds(c["rate_abs_mps"], false, {3, 8, false});
    }
  } else if (m.checker == "RelayChecker") {
    keys(q, m.id + ".quality", {});
    if (m.adapter != "DiagnosticRelayAdapter")
      throw std::invalid_argument(
          "RelayChecker requires DiagnosticRelayAdapter");
  } else if (m.checker == "PlanningChecker") {
    keys(q, m.id + ".quality",
         {"task_state_topic", "task_state_timeout_sec", "result_deadline_sec",
          "maximum_segment_m"});
    for (const char* name :
         {"task_state_timeout_sec", "result_deadline_sec", "maximum_segment_m"})
      if (q[name] && !q[name].IsNull()) {
        const auto r = q[name];
        if (r.IsMap()) keys(r, name, {"enabled", "value"});
        if (!std::isfinite(ruleValue(r, 2)) || ruleValue(r, 2) <= 0)
          throw std::invalid_argument(std::string(name) + " must be positive");
      }
    if (!q["task_state_topic"] ||
        q["task_state_topic"].as<std::string>().empty())
      throw std::invalid_argument("planning requires task_state_topic");
    if (m.adapter != "PathAndTaskStateAdapter")
      throw std::invalid_argument("unsupported planning adapter");
  } else
    throw std::invalid_argument("checker not registered: " + m.checker);
}
}  // namespace

// 统一判定规则是否被用户选择；不存在/null/显式false关闭，存在时默认启用。
bool ruleEnabled(const YAML::Node& n) {
  return n && !n.IsNull() && (!n.IsMap() || get<bool>(n, "enabled", true));
}
// 为ROS适配器提供与核心一致的可选阈值解析，避免适配器另藏一套默认告警。
Bounds configuredBounds(const YAML::Node& n, bool lower) {
  return bounds(n, lower, {0, 0, lower});
}
// 读取可选标量规则的数值，兼容旧数值写法与{enabled,value}新写法。
double ruleValue(const YAML::Node& n, double fallback) {
  if (!n || n.IsNull()) return fallback;
  return n.IsMap() ? get<double>(n, "value", fallback) : n.as<double>();
}

// 读取单调时钟秒数；不受系统时间校正或bag暂停影响，供接收年龄探针使用。
double steadyNow() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
// 将有限数值转换成可读文本；非有限值显示N/A，不伪造成0。
std::string number(double value) {
  if (!std::isfinite(value)) return "N/A";
  std::ostringstream out;
  out << std::setprecision(12) << value;
  return out.str();
}
// 生成本次运行/命令/实例标识；用于内部关联和防止旧控制心跳被误接受。
std::string uniqueId() {
  std::random_device rd;
  std::ostringstream out;
  out << std::hex << getpid() << '-'
      << std::chrono::high_resolution_clock::now().time_since_epoch().count()
      << '-' << rd();
  return out.str();
}
// 把0/1/2转换成OK/WARNING/ERROR，便于诊断和事件阅读。
const char* levelName(int l) {
  return l == OK ? "OK" : l == WARNING ? "WARNING" : "ERROR";
}
// 仅比较已经启用的阈值档；严格大于/小于才触发，ERROR优先，省略档不补默认值。
int Bounds::level(double v) const {
  if (error_enabled && (lower ? v < error : v > error)) return ERROR;
  if (warning_enabled && (lower ? v < warning : v > warning)) return WARNING;
  return OK;
}

/**
 * 加载并规范化一份V2健康监控配置。
 * 阶段顺序：保留原始文件文本 -> 校验顶层/运行参数 -> 逐模块过滤enabled:false
 * -> 合并共享设置和模块覆盖 -> 校验checker/adapter/quality -> 检查依赖图
 * -> 保存effective_text。调用方把返回值交给监控器、适配器和日志器。
 * 缺失的运行参数采用代码中明确的运行默认值；缺失的检测规则保持关闭，不能把
 * 运行参数默认值误解成自动打开的业务阈值。
 */
MonitorConfig ConfigLoader::load(const std::string& path) {
  MonitorConfig c;
  c.source_path = path;
  std::ifstream in(path);
  if (!in) throw std::invalid_argument("cannot open config: " + path);
  // 保留配置文件逐字节文本用于本次run追溯；YAML解析使用同一份内容，避免读到两版文件。
  c.source_text.assign(std::istreambuf_iterator<char>(in), {});
  YAML::Node root = YAML::Load(c.source_text);
  // 先限制顶层结构，schema_version之后再逐块解析；未知项直接报错。
  keys(root, "root",
       {"schema_version", "monitor", "control", "stream_defaults",
        "lifecycle_defaults", "logging", "modules"});
  if (get<int>(root, "schema_version", 2) != 2)
    throw std::invalid_argument("schema_version must be 2");
  // 读取发布/统计参数。frequency_coverage不能超过统计窗口，样本数也需要处于可用范围。
  auto n = root["monitor"];
  keys(n, "monitor",
       {"publish_rate_hz", "statistics_window_sec", "min_frequency_samples",
        "min_frequency_coverage_sec", "soft_warning_hold_sec",
        "recovery_hold_sec", "diagnostics_topic", "status_topic",
        "data_reference"});
  c.data_reference = get<std::string>(n, "data_reference", "");
  c.publish_rate = positive(n, "publish_rate_hz", 5);
  c.window = positive(n, "statistics_window_sec", 5);
  c.frequency_samples = get<unsigned>(n, "min_frequency_samples", 3);
  if (c.frequency_samples < 2 || c.frequency_samples > 100000)
    throw std::invalid_argument("min_frequency_samples out of bounds");
  c.frequency_coverage = positive(n, "min_frequency_coverage_sec", 1);
  if (c.frequency_coverage > c.window)
    throw std::invalid_argument("frequency coverage exceeds window");
  c.soft_hold = positive(n, "soft_warning_hold_sec", .5, true);
  c.recovery_hold = positive(n, "recovery_hold_sec", 1, true);
  c.diagnostics_topic =
      get<std::string>(n, "diagnostics_topic", c.diagnostics_topic);
  c.status_topic = get<std::string>(n, "status_topic", c.status_topic);
  // 控制心跳输入与超时策略和数据Topic分开配置；ERROR超时必须比WARNING更大。
  n.reset(root["control"]);
  keys(n, "control",
       {"strict_intent", "state_topic", "heartbeat_warning_sec",
        "heartbeat_error_sec"});
  c.strict_control = get<bool>(n, "strict_intent", false);
  c.state_topic = get<std::string>(n, "state_topic", c.state_topic);
  c.heartbeat_warning = positive(n, "heartbeat_warning_sec", 1);
  c.heartbeat_error = positive(n, "heartbeat_error_sec", 3);
  if (c.heartbeat_warning >= c.heartbeat_error)
    throw std::invalid_argument("heartbeat bounds invalid");
  // 建立日志运行参数。容量必须为整数，预算必须能容纳至少一个轮转文件。
  n.reset(root["logging"]);
  keys(n, "logging",
       {"enabled", "directory", "metrics_period_sec", "metrics_queue_capacity",
        "critical_event_queue_capacity", "rotate_size_mb", "rotate_file_count",
        "total_budget_mb", "flush_period_sec", "events"});
  c.logging = get<bool>(n, "enabled", true);
  // 独立解析事件文件策略；删除/注释某个列表、写null或[]均表示该项不筛选。
  // 整个logging.events缺失时仍记录全部事件，兼容旧配置。
  const YAML::Node events = n && n["events"] ? n["events"] : YAML::Node();
  keys(events, "logging.events",
       {"include_event_types", "exclude_rule_ids", "exclude_module_ids"});
  auto event_names = [&](const char* key) {
    std::set<std::string> result;
    const auto list = events && events[key] ? events[key] : YAML::Node();
    const std::string path = std::string("logging.events.") + key;
    if (!list || list.IsNull()) return result;
    if (!list.IsSequence())
      throw std::invalid_argument(path + " must be a list, e.g. [] or [RULE_TRIGGERED]");
    for (const auto& item : list) {
      if (!item.IsScalar())
        throw std::invalid_argument(path + " entries must be nonempty names");
      const auto name = item.as<std::string>();
      if (name.empty() || name.find_first_of("*?[]") != std::string::npos ||
          std::any_of(name.begin(), name.end(), [](unsigned char ch) {
            return std::isspace(ch) || std::iscntrl(ch);
          }))
        throw std::invalid_argument(path + " entries require exact names without whitespace or wildcards");
      result.insert(name);  // 重复名称只保留一份，匹配结果不受顺序影响。
    }
    return result;
  };
  c.event_filter.include_event_types = event_names("include_event_types");
  c.event_filter.exclude_rule_ids = event_names("exclude_rule_ids");
  c.event_filter.exclude_module_ids = event_names("exclude_module_ids");
  const char* home = std::getenv("HOME");
  const std::string default_log = std::string(home ? home : "/tmp") + "/.ros/robot_health/health";
  c.log_directory = get<std::string>(n, "directory", default_log);
  if (c.log_directory.compare(0, 2, "~/") == 0) {
    if (!home) throw std::invalid_argument("HOME is unavailable for ~/ log directory");
    c.log_directory = std::string(home) + c.log_directory.substr(1);
  }
  if (c.logging && c.log_directory.empty())
    throw std::invalid_argument("logging.directory required");
  c.metrics_period = positive(n, "metrics_period_sec", 1);
  c.flush_period = positive(n, "flush_period_sec", 1);
  auto capacity = [&](const char* name, size_t def) {
    double v = positive(n, name, def);
    if (v > 1000000 || std::floor(v) != v)
      throw std::invalid_argument(std::string(name) + " invalid capacity");
    return static_cast<size_t>(v);
  };
  c.metrics_capacity = capacity("metrics_queue_capacity", 4096);
  c.event_capacity = capacity("critical_event_queue_capacity", 512);
  c.rotate_bytes =
      static_cast<size_t>(positive(n, "rotate_size_mb", 20) * 1024 * 1024);
  c.budget_bytes =
      static_cast<size_t>(positive(n, "total_budget_mb", 512) * 1024 * 1024);
  c.rotate_count = capacity("rotate_file_count", 10);
  if (c.rotate_bytes < 1024 || c.budget_bytes < c.rotate_bytes)
    throw std::invalid_argument("log budget invalid");
  // modules必须是非空列表；关闭模块在此跳过，后续不创建订阅和Checker。
  if (!root["modules"] || !root["modules"].IsSequence() ||
      root["modules"].size() == 0)
    throw std::invalid_argument("modules must be a nonempty list");
  std::set<std::string> ids;
  for (size_t i = 0; i < root["modules"].size(); ++i) {
    auto m = root["modules"][i];
    // 先检查键集合，再读取模块身份；重复ID会使诊断、日志和依赖无法唯一对应。
    keys(m, "module",
         {"enabled", "id", "checker", "adapter", "topic", "required", "depends_on",
          "stream", "lifecycle", "quality", "visualization_topic",
          "visualization_frame", "missing_data_policy"});
    // 模块级开关false代表整模块不实例化；它与模块内单条规则开关是两个层级。
    if (!get<bool>(m, "enabled", true)) continue;
    ModuleConfig mc;
    mc.id = get<std::string>(m, "id", "");
    mc.checker = get<std::string>(m, "checker", "");
    mc.adapter = get<std::string>(m, "adapter", "");
    mc.topic = get<std::string>(m, "topic", "");
    if (mc.id.empty() ||
        mc.id.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRST"
                                "UVWXYZ0123456789_-") != std::string::npos ||
        !ids.insert(mc.id).second)
      throw std::invalid_argument("invalid/duplicate module id");
    if (mc.topic.empty()) throw std::invalid_argument("module topic required");
    mc.required = get<bool>(m, "required", true);
    mc.missing_data_policy = get<std::string>(m, "missing_data_policy", "fault");
    if (mc.missing_data_policy != "fault" &&
        mc.missing_data_policy != "no_data_until_first_sample")
      throw std::invalid_argument("unsupported missing_data_policy");
    if (mc.missing_data_policy == "no_data_until_first_sample" &&
        (mc.required || mc.checker == "PlanningChecker"))
      throw std::invalid_argument("no_data_until_first_sample requires a non-required stream module");
    if (m["depends_on"] && !m["depends_on"].IsNull())
      mc.dependencies = m["depends_on"].as<std::vector<std::string>>();
    // 流规则采用“共享默认值 + 本模块覆盖”；frequency_basis明确选择source或receive。
    auto s = merge(root["stream_defaults"], m["stream"]);
    keys(s, "stream",
         {"stamp_domain", "frequency_basis", "stamp", "receive_age_sec",
          "frequency"});
    mc.stream.stamp_domain =
        get<std::string>(s, "stamp_domain", "device_relative");
    mc.stream.frequency_basis =
        get<std::string>(s, "frequency_basis", "source");
    if (mc.stream.frequency_basis != "source" &&
        mc.stream.frequency_basis != "receive")
      throw std::invalid_argument("invalid frequency_basis");
    // RECEIVE_AGE使用单调接收时间；不依赖下一帧，也不受bag的ROS时钟暂停影响。
    mc.stream.age = bounds(s["receive_age_sec"], false, {.5, 1.5, false});
    if (!s["frequency"] || s["frequency"].IsNull()) s["frequency"] = YAML::Node(YAML::NodeType::Map);
    auto f = s["frequency"];
    keys(f, "frequency",
         {"enabled", "warning_below_hz", "error_below_hz", "warning_above_hz",
          "error_above_hz"});
    // YAML把频率上下边界放在一个块中；核心将低频/高频各自转成可独立选择的Bounds。
    YAML::Node low(YAML::NodeType::Map), high(YAML::NodeType::Map);
    for (const auto& pair : {std::make_pair("warning_below_hz", "warning_below"),
                            std::make_pair("error_below_hz", "error_below")})
      if (f[pair.first]) low[pair.second] = f[pair.first];
    for (const auto& pair : {std::make_pair("warning_above_hz", "warning_above"),
                            std::make_pair("error_above_hz", "error_above")})
      if (f[pair.first]) high[pair.second] = f[pair.first];
    low["enabled"] = high["enabled"] = get<bool>(f, "enabled", true);
    mc.stream.low_rate = bounds(low, true, {5, 1, true});
    mc.stream.high_rate = bounds(high, false, {15, 30, false});
    if (mc.stream.low_rate.warning_enabled && mc.stream.high_rate.warning_enabled &&
        mc.stream.low_rate.warning >= mc.stream.high_rate.warning)
      throw std::invalid_argument("frequency ordering invalid");
    // stamp规则只处理源时间戳：正数/重复/倒退/相邻stamp间隔；不会关闭接收年龄探针。
    auto st = s["stamp"];
    keys(st, "stamp",
         {"enabled", "require_positive", "duplicate_level", "regression_level",
          "max_gap_warning_sec", "max_gap_error_sec"});
    mc.stream.stamp_checks_enabled = ruleEnabled(st);
    mc.stream.require_positive = get<bool>(st, "require_positive", false);
    mc.stream.duplicate_level = mc.stream.stamp_checks_enabled && st["duplicate_level"] && !st["duplicate_level"].IsNull()
        ? parseLevel(st, "duplicate_level", "WARNING") : OK;
    mc.stream.regression_level = mc.stream.stamp_checks_enabled && st["regression_level"] && !st["regression_level"].IsNull()
        ? parseLevel(st, "regression_level", "ERROR") : OK;
    YAML::Node gap(YAML::NodeType::Map);
    gap["enabled"] = mc.stream.stamp_checks_enabled;
    if (st && !st.IsNull()) {
      if (st["max_gap_warning_sec"]) gap["warning_above"] = st["max_gap_warning_sec"];
      if (st["max_gap_error_sec"]) gap["error_above"] = st["max_gap_error_sec"];
    }
    mc.stream.gap = bounds(gap, false, {.3, 1, false});
    // 生命周期配置独立继承；binding用于把相关传感器绑定到同一受管进程状态。
    auto l = merge(root["lifecycle_defaults"], m["lifecycle"]);
    keys(l, "lifecycle",
         {"control_binding", "startup_deadline_sec", "stop_deadline_sec",
          "ready_consecutive_samples"});
    mc.lifecycle.binding = get<std::string>(l, "control_binding", mc.id);
    mc.lifecycle.startup = positive(l, "startup_deadline_sec", 10);
    mc.lifecycle.stop = positive(l, "stop_deadline_sec", 5);
    mc.lifecycle.ready_samples =
        get<unsigned>(l, "ready_consecutive_samples", 10);
    if (!mc.lifecycle.ready_samples || mc.lifecycle.ready_samples > 1000000)
      throw std::invalid_argument("ready_consecutive_samples invalid");
    l["control_binding"] = mc.lifecycle.binding;
    l["startup_deadline_sec"] = mc.lifecycle.startup;
    l["stop_deadline_sec"] = mc.lifecycle.stop;
    l["ready_consecutive_samples"] = mc.lifecycle.ready_samples;
    s["stamp_domain"] = mc.stream.stamp_domain;
    s["frequency_basis"] = mc.stream.frequency_basis;
    mc.quality = m["quality"] ? YAML::Clone(m["quality"]) : YAML::Node(YAML::NodeType::Map);
    if (mc.quality.IsNull()) mc.quality = YAML::Node(YAML::NodeType::Map);
    mc.visual_topic = get<std::string>(m, "visualization_topic", "");
    mc.visual_frame = get<std::string>(m, "visualization_frame", "");
    // 确认quality与checker/adapter匹配后再保存该模块；错误配置不会留到运行中才暴露。
    qualityValidate(mc);
    // 保存合并后的规则和生命周期参数；不回填缺失阈值，避免未来重新加载时暗中启用规则。
    m["stream"] = s;
    m["lifecycle"] = l;
    m["missing_data_policy"] = mc.missing_data_policy;
    c.modules.push_back(mc);
  }
  if (c.modules.empty()) throw std::invalid_argument("at least one enabled module is required");
  // 依赖采用深度优先校验：既要求被依赖模块已启用，也拒绝A->B->A这类循环。
  std::map<std::string, int> visiting;
  std::function<void(const std::string&)> visit = [&](const std::string& id) {
    if (visiting[id] == 1) throw std::invalid_argument("dependency cycle");
    if (visiting[id] == 2) return;
    visiting[id] = 1;
    auto it = std::find_if(c.modules.begin(), c.modules.end(),
                           [&](const ModuleConfig& m) { return m.id == id; });
    if (it == c.modules.end())
      throw std::invalid_argument("unknown dependency: " + id);
    for (const auto& d : it->dependencies) visit(d);
    visiting[id] = 2;
  };
  for (const auto& m : c.modules) visit(m.id);
  // 将默认值和标准化字段写入有效快照；原始source_text仍原样保留，便于比较用户配置与生效配置。
  root["schema_version"] = 2;
  root["monitor"]["publish_rate_hz"] = c.publish_rate;
  root["monitor"]["statistics_window_sec"] = c.window;
  root["monitor"]["min_frequency_samples"] = c.frequency_samples;
  root["monitor"]["min_frequency_coverage_sec"] = c.frequency_coverage;
  root["monitor"]["soft_warning_hold_sec"] = c.soft_hold;
  root["monitor"]["recovery_hold_sec"] = c.recovery_hold;
  root["monitor"]["diagnostics_topic"] = c.diagnostics_topic;
  root["monitor"]["status_topic"] = c.status_topic;
  root["monitor"]["data_reference"] = c.data_reference;
  root["control"]["strict_intent"] = c.strict_control;
  root["control"]["state_topic"] = c.state_topic;
  root["control"]["heartbeat_warning_sec"] = c.heartbeat_warning;
  root["control"]["heartbeat_error_sec"] = c.heartbeat_error;
  root["logging"]["enabled"] = c.logging;
  // 有效配置始终展开三个列表，便于追溯本次筛选；原始配置文本保持用户写法。
  root["logging"]["events"]["include_event_types"] =
      std::vector<std::string>(c.event_filter.include_event_types.begin(),
                               c.event_filter.include_event_types.end());
  root["logging"]["events"]["exclude_rule_ids"] =
      std::vector<std::string>(c.event_filter.exclude_rule_ids.begin(),
                               c.event_filter.exclude_rule_ids.end());
  root["logging"]["events"]["exclude_module_ids"] =
      std::vector<std::string>(c.event_filter.exclude_module_ids.begin(),
                               c.event_filter.exclude_module_ids.end());
  root["logging"]["directory"] = c.log_directory;
  root["logging"]["metrics_period_sec"] = c.metrics_period;
  root["logging"]["metrics_queue_capacity"] = c.metrics_capacity;
  root["logging"]["critical_event_queue_capacity"] = c.event_capacity;
  root["logging"]["rotate_size_mb"] = double(c.rotate_bytes) / (1024 * 1024);
  root["logging"]["rotate_file_count"] = c.rotate_count;
  root["logging"]["total_budget_mb"] = double(c.budget_bytes) / (1024 * 1024);
  root["logging"]["flush_period_sec"] = c.flush_period;
  YAML::Emitter out;
  out << root;
  c.effective_text = out.c_str();
  return c;
}

// 保存统计窗口与规则选择，初始化单调接收时间和源时间历史。
StreamTracker::StreamTracker(const StreamConfig& c, const MonitorConfig& m)
    : config_(c),
      window_(m.window),
      coverage_(m.frequency_coverage),
      min_samples_(m.frequency_samples) {
  reset(steadyNow());
}
// 清空同一模块的时间/频率历史；新实例或可信重置后重新预热。
void StreamTracker::reset(double now) {
  started_ = now;
  last_receive_ = last_stamp_ = 0;
  count_ = 0;
  receipts_.clear();
  sources_.clear();
}
// 每条消息到达时检查源stamp重复/倒退/间隔，保存接收单调时间；不等待定时器。
Findings StreamTracker::receive(double stamp, double now) {
  Findings f;
  // 源stamp用于判断消息自身时间是否合法；now来自steady clock，只记录本机实际收到消息的时刻。
  const bool valid =
      std::isfinite(stamp) && (!config_.require_positive || stamp > 0);
  if (config_.stamp_checks_enabled && config_.require_positive)
  add(f, "STAMP_INVALID",
      valid ? "valid timestamp" : "zero/nonfinite timestamp",
      valid ? OK : ERROR, stamp, 0, "s", !valid);
  // 只有源时间戳有效且已有有效基线时才检查相邻关系；倒退stamp不拿来污染后续统计窗口。
  if (valid && count_ && last_stamp_ > 0) {
    if (stamp == last_stamp_ && config_.duplicate_level > OK)
      add(f, "STAMP_DUPLICATE", "duplicate source timestamp",
          config_.duplicate_level, stamp, 0, "s",
          config_.duplicate_level == ERROR);
    else if (stamp < last_stamp_ && config_.regression_level > OK) {
      add(f, "STAMP_REGRESSION", "unauthorized source timestamp regression",
          config_.regression_level, stamp, last_stamp_, "s", true);
    } else if (stamp > last_stamp_)
      threshold(f, "STAMP_GAP", stamp - last_stamp_, config_.gap, "s");
  }
  // 频率历史仅追加有效递增的源时间；接收时间历史则记录每次回调，包括带坏stamp的消息。
  if (valid && (last_stamp_ <= 0 || stamp > last_stamp_)) {
    sources_.push_back({now, stamp});
    last_stamp_ = stamp;
  }
  last_receive_ = now;
  receipts_.push_back(now);
  ++count_;
  return f;
}
// 独立定时检查接收年龄并裁剪统计窗口；没有新消息也会在阈值后报警。
Findings StreamTracker::evaluate(double now, Values& v, bool& ready) {
  Findings f;
  ready = false;
  // 两个窗口分别按本机steady时间和源stamp跨度裁剪，避免跨长时间暂停的旧样本影响频率。
  while (!receipts_.empty() && now - receipts_.front() > window_)
    receipts_.pop_front();
  while (!sources_.empty() &&
         (now - sources_.front().first > window_ ||
          sources_.back().second - sources_.front().second > window_))
    sources_.pop_front();
  // age永远是接收时钟年龄；还没收到第一帧时从模块启动时刻开始计算等待时间。
  const double age =
      count_ ? std::max(0.0, now - last_receive_) : now - started_;
  v["age_sec"] = count_ ? number(age) : "N/A";
  v["message_count"] = std::to_string(count_);
  v["last_stamp_sec"] = number(last_stamp_);
  v["stamp_domain"] = config_.stamp_domain;
  v["receive_rate_hz"] = number(
      receipts_.size() / std::min(window_, std::max(1e-9, now - started_)));
  // receive_rate用于现场调试；source_rate基于源stamp，可在bag倍速回放时保持设备真实采样率。
  double span = sources_.size() > 1
                    ? sources_.back().second - sources_.front().second
                    : 0;
  double source_rate = span > 0 ? (sources_.size() - 1) / span : 0;
  v["source_rate_hz"] = sources_.size() > 1 ? number(source_rate) : "N/A";
  double duration = config_.frequency_basis == "source"
                        ? span
                        : std::min(window_, now - started_);
  size_t samples =
      config_.frequency_basis == "source" ? sources_.size() : receipts_.size();
  // ready表示频率统计样本数/时间覆盖已足够且流没有超过年龄阈值，不等于整模块健康OK。
  ready = samples >= min_samples_ && duration >= coverage_ &&
          (!config_.age.warning_enabled || age <= config_.age.warning) &&
          (!config_.age.error_enabled || age <= config_.age.error);
  const double rate = config_.frequency_basis == "source"
                          ? source_rate
                          : receipts_.size() / std::max(1e-9, duration);
  v["rate_hz"] = ready ? number(rate) : "N/A";
  v["frequency_status"] = ready ? "AVAILABLE" : "WARMING_UP_OR_STALE";
  // 频率在统计准备完成后才比较，避免启动最初几帧用不稳定估算提前报警。
  if (ready) {
    threshold(f, "FREQUENCY_LOW", rate, config_.low_rate, "Hz");
    threshold(f, "FREQUENCY_HIGH", rate, config_.high_rate, "Hz");
  }
  threshold(f, "RECEIVE_AGE", age, config_.age, "s");
  v["transport_delay_sec"] = "N/A (no verified clock mapping)";
  return f;
}

// 设置软告警持续时间、恢复保持时间和事件回调；每条规则有独立状态。
RuleLatch::RuleLatch(double s, double r, EventSink sink)
    : soft_hold_(s), recovery_hold_(r), sink_(std::move(sink)) {}
// 合并同ID最严重结果，更新候选/触发/恢复状态；只更新本次提供的规则，不把缺席当恢复。
void RuleLatch::apply(const Findings& findings, double now) {
  std::map<std::string, Finding> current;
  // 同一周期若同一规则出现多次，只保留最高等级，保证每个规则ID只有一个状态机。
  for (const auto& f : findings) {
    auto it = current.find(f.id);
    if (it == current.end() || it->second.level < f.level) current[f.id] = f;
  }
  // Only passed/failed rules supplied by their owner are updated here. Missing
  // rules in another callback are not treated as recovery of this owner's rule.
  for (const auto& kv : current) {
    const auto& f = kv.second;
    auto& s = states_[kv.first];
    if (f.level > OK) {
      // 故障出现/升级时重置恢复计时；新等级需要重新满足软告警保持时间。
      s.finding = f;
      s.good_since = -1;
      if (s.candidate != f.level) {
        s.candidate = f.level;
        s.since = now;
        if (sink_) sink_("WARNING_CANDIDATE", f, s.active, f.level);
      }
      // hard错误立即触发；WARNING等软告警必须持续到soft_hold，短暂毛刺可被过滤。
      if (f.hard || now - s.since >= soft_hold_) {
        if (s.active != f.level) {
          int before = s.active;
          s.active = f.level;
          if (sink_) sink_("RULE_TRIGGERED", f, before, s.active);
        }
        s.period = std::max(s.period, f.level);
      }
    } else {
      // 只有规则所有者明确提交OK才开始恢复计时；另一类回调未提供该ID不代表规则已恢复。
      s.candidate = OK;
      if (s.active > OK) {
        if (s.good_since < 0) s.good_since = now;
        // 连续正常达到recovery_hold才清告警；中间再次异常会在上面的分支清零计时。
        if (now - s.good_since >= recovery_hold_) {
          int before = s.active;
          s.active = OK;
          if (sink_) sink_("RULE_RECOVERED", f, before, OK);
        }
      }
    }
  }
}
// 取所有规则当前状态和本发布周期短暂硬错误的最大等级。
int RuleLatch::level() const {
  int l = OK;
  for (const auto& kv : states_)
    l = std::max(l, std::max(kv.second.active, kv.second.period));
  return l;
}
// 读取本次发布周期出现过的最严重结果，避免短暂NaN被后一帧覆盖。
int RuleLatch::periodLevel() const {
  int level = OK;
  for (const auto& state : states_)
    level = std::max(level, state.second.period);
  return level;
}
// 拼接仍活跃的异常规则ID及原因，给诊断消息和界面使用。
std::string RuleLatch::reasons() const {
  std::string s;
  for (const auto& kv : states_)
    if (kv.second.active > OK || kv.second.period > OK) {
      if (!s.empty()) s += "; ";
      s += kv.first + ": " + kv.second.finding.reason;
    }
  return s.empty() ? "OK" : s;
}
// 清空规则历史；仅用于受信任的实例重置，不能用来掩盖仍存在的故障。
void RuleLatch::reset() { states_.clear(); }
// 发布后清除周期保持标记；未恢复的持续规则仍保留。
void RuleLatch::published() {
  for (auto& kv : states_) kv.second.period = OK;
}

// 读取点数/有效点/有效率三个可选规则；没有配置的规则保持关闭。
LidarChecker::LidarChecker(const YAML::Node& q)
    : total_(bounds(q["points_total"], true, {100, 1, true})),
      valid_(bounds(q["points_valid"], true, {100, 1, true})),
      ratio_(bounds(q["valid_ratio"], true, {.8, .5, true})) {}
// 把适配器统计值变成规则结果；仅执行选中的点数/比例规则，并保留基础结构错误。
Findings LidarChecker::check(const Sample& s, Values& v) {
  v = s.values;
  Findings f = s.findings;
  v["points_total"] = std::to_string(s.total);
  v["points_valid"] = std::to_string(s.valid_points);
  // ratio按有效点数/本帧总点数计算；空帧不伪造成有效比例。
  double ratio = s.total ? double(s.valid_points) / s.total : 0;
  v["valid_ratio"] = number(ratio);
  threshold(f, "LIDAR_COUNT_TOTAL", s.total, total_, "points");
  threshold(f, "LIDAR_COUNT_VALID", s.valid_points, valid_, "points");
  threshold(f, "LIDAR_RATIO", ratio, ratio_, "ratio");
  // 启用了点云业务质量规则时，完全空帧或没有有效点作为硬错误；全关时仍保留基础结构检查。
  if ((total_.enabled() || valid_.enabled() || ratio_.enabled()) && (!s.total || !s.valid_points))
    add(f, "LIDAR_EMPTY", "empty scan or no valid points", ERROR,
        s.valid_points, 1, "points", true);
  else if (total_.enabled() || valid_.enabled() || ratio_.enabled())
    add(f, "LIDAR_EMPTY", "scan contains valid points", OK);
  return f;
}
// 为IMU或定位保存同一套姿态配置，以前缀区分日志中的来源。
AttitudeTracker::AttitudeTracker(const YAML::Node& q, const std::string& prefix)
    : config_(q), prefix_(prefix) {}
// 验证可用四元数 -> 归一化 -> ZYX roll/pitch -> 可选角度/短弧跳变/角速度规则。
Findings AttitudeTracker::update(const Sample& s, Values& v) {
  Findings f;
  if (!ruleEnabled(config_)) {
    v["attitude_status"] = "DISABLED";
    reset();
    return f;
  }
  v["roll_deg"] = v["pitch_deg"] = v["attitude_rotation_jump_deg"] =
      v["attitude_angular_speed_dps"] = "N/A";
  // 有的消息（例如原始IMU）没有orientation；是否把缺失视作必需检查失败由require_available控制。
  const bool required = get<bool>(config_, "require_available", false);
  add(f, prefix_ + "_ATTITUDE_AVAILABLE", s.orientation_available
          ? "orientation supplied" : "orientation not supplied; cannot check roll/pitch",
      s.orientation_available ? OK : required ? ERROR : WARNING, 0, 0, "",
      !s.orientation_available && required);
  if (!s.orientation_available) {
    v["attitude_status"] = "UNAVAILABLE";
    if (required) v["required_checks_available"] = "false";
    reset();
    return f;
  }
  // 先检查四元数有限且非零，再评估单位范数误差；无效四元数不能转换成可信Euler角。
  const double norm = s.orientation.coeffs().stableNorm();
  const bool usable = s.orientation.coeffs().allFinite() &&
                      std::isfinite(norm) && norm > 1e-12;
  add(f, prefix_ + "_ATTITUDE_QUATERNION", usable ? "finite nonzero quaternion"
      : "nonfinite or zero quaternion; angles unavailable", usable ? OK : ERROR,
      norm, 0, "", !usable);
  if (!usable) {
    v["attitude_status"] = "INVALID";
    reset();
    return f;
  }
  const auto norm_bounds = bounds(config_["quaternion_norm_error"], false,
                                  {.02, .1, false});
  threshold(f, prefix_ + "_ATTITUDE_NORM", std::abs(norm - 1), norm_bounds, "ratio");
  if (norm_bounds.level(std::abs(norm - 1)) == ERROR) {
    v["attitude_status"] = "INVALID";
    reset();
    return f;
  }
  Eigen::Quaterniond q = s.orientation;
  q.coeffs() /= norm;
  // 归一化后按ZYX约定计算roll/pitch，显示为度；pitch输入裁剪到asin定义域[-1,1]。
  constexpr double degrees = 57.2957795130823208768;
  const double roll = std::atan2(2 * (q.w()*q.x() + q.y()*q.z()),
                                1 - 2 * (q.x()*q.x() + q.y()*q.y())) * degrees;
  const double sine_pitch = std::max(-1.0, std::min(1.0,
                                  2 * (q.w()*q.y() - q.z()*q.x())));
  const double pitch = std::asin(sine_pitch) * degrees;
  v["attitude_status"] = "AVAILABLE";
  const bool euler_regular = std::abs(sine_pitch) < 1 - 1e-8;
  v["roll_deg"] = euler_regular ? number(roll) : "N/A";
  v["pitch_deg"] = number(pitch);
  // An IMU header identifies the measurement frame, not its world reference.
  v["attitude_message_frame"] = s.frame;
  v["attitude_angle_convention"] = "ZYX Euler; degrees; reference frame must be gravity-aligned";
  add(f, prefix_ + "_ATTITUDE_EULER", euler_regular ? "regular Euler representation"
      : "pitch near +/-90 degrees; roll is undefined, quaternion checks retained",
      euler_regular ? OK : WARNING);
  if (euler_regular)
    threshold(f, prefix_ + "_ROLL_LIMIT", std::abs(roll),
              bounds(config_["roll_abs_deg"], false, {30,45,false}), "deg");
  threshold(f, prefix_ + "_PITCH_LIMIT", std::abs(pitch),
            bounds(config_["pitch_abs_deg"], false, {30,45,false}), "deg");
  // roll/pitch单帧结果可用，但跨帧跳变需要有效递增源时间；不满足时清基线重新预热。
  if (!s.valid || !std::isfinite(s.stamp) || s.stamp <= 0) {
    v["attitude_continuity_status"] = "INVALID_SAMPLE_OR_TIME";
    reset();
    return f;
  }
  const double dt = s.stamp - stamp_;
  // 只有同一frame、child_frame且dt落在配置范围时，才把两帧姿态解释为连续运动。
  const bool compare = have_previous_ && frame_ == s.frame &&
      child_frame_ == s.child_frame && dt >= get<double>(config_, "dt_min_sec", .0001) &&
      dt <= get<double>(config_, "dt_max_sec", 1);
  v["attitude_continuity_status"] = compare ? "AVAILABLE" : "WARMUP_OR_TIME_FRAME_RESET";
  if (compare) {
    // Quaternion distance is sign invariant and avoids Euler wrap/singularity jumps.
    const double dot = std::min(1.0, std::abs(previous_.dot(q)));
    const double angle = 2 * std::acos(dot) * degrees;
    v["attitude_rotation_jump_deg"] = number(angle);
    v["attitude_angular_speed_dps"] = number(angle / dt);
    threshold(f, prefix_ + "_ATTITUDE_JUMP", angle,
              bounds(config_["rotation_jump_deg"], false, {30,60,false}), "deg");
    threshold(f, prefix_ + "_ATTITUDE_RATE", angle / dt,
              bounds(config_["angular_speed_dps"], false, {180,360,false}), "deg/s");
  }
  previous_ = q;
  stamp_ = s.stamp;
  frame_ = s.frame;
  child_frame_ = s.child_frame;
  have_previous_ = true;
  return f;
}
// 保存SI幅值规则和共享姿态规则；单位转换由ROS适配器先完成。
ImuChecker::ImuChecker(const YAML::Node& q)
    : acceleration_(
          bounds(q["acceleration_norm_mps2"], false, {100, 200, false})),
      gyro_(bounds(q["angular_velocity_norm_rps"], false, {35, 70, false})),
      attitude_(q["attitude"], "IMU") {}
// 接收统一SI样本，运行所选加速度/角速度幅值规则，再合并姿态及适配器异常。
Findings ImuChecker::check(const Sample& s, Values& v) {
  v = s.values;
  Findings f = s.findings;
  // 适配器已把g换成m/s²；这里仅算三轴向量模长，避免逐轴方向变化造成错误告警。
  if (s.acceleration.allFinite())
    threshold(f, "IMU_ACCEL_NORM", s.acceleration.stableNorm(), acceleration_,
              "m/s^2");
  if (s.gyro.allFinite())
    threshold(f, "IMU_GYRO_NORM", s.gyro.stableNorm(), gyro_, "rad/s");
  auto a = attitude_.update(s, v);
  f.insert(f.end(), a.begin(), a.end());
  return f;
}
// 验证量程与测距 -> 可选人工范围/高低限 -> 以源时间差检查可选测距跳变/变化率。
Findings RangeChecker::check(const Sample& s, Values& v) {
  v = s.values;
  Findings f = s.findings;
  v["range_m"] = number(s.range_m);
  v["range_valid"] = "false";
  v["range_delta_m"] = v["range_rate_mps"] = "N/A";
  v["range_continuity_status"] = "UNAVAILABLE";
  v["required_checks_available"] = "false";
  v["checks_available"] = "range,time,frequency; vertical-height/odom consistency not calibrated";
  // Range消息必须声明有限、非负且递增的min/max；不符合时无法把数值解释为有效米制距离。
  const bool metadata = std::isfinite(s.range_min_m) && std::isfinite(s.range_max_m) &&
                        s.range_min_m >= 0 && s.range_min_m < s.range_max_m;
  add(f, "RANGE_METADATA", metadata ? "valid metric range limits"
      : "invalid limits or binary fixed-distance sensor; metric height unavailable",
      metadata ? OK : ERROR, 0, 0, "m", !metadata);
  if (!metadata) { v["range_status"] = "INVALID_METADATA"; reset(); return f; }
  // 正无穷通常表示无回波，负无穷表示过近/量程外；两者都不是可用于高度计算的数值距离。
  if (std::isinf(s.range_m)) {
    v["range_status"] = s.range_m > 0 ? "NO_RETURN" : "TOO_CLOSE_UNQUANTIFIED";
    if (quality_["no_return_level"] && !quality_["no_return_level"].IsNull())
    add(f, "RANGE_RETURN", "range is unquantified; no usable height estimate",
        parseLevel(quality_, "no_return_level", "WARNING"), s.range_m, 0, "m");
    reset();
    return f;
  }
  add(f, "RANGE_RETURN", "finite measurement returned", OK);
  const auto limits = quality_["valid_range_m"];
  // 同时尊重消息声明量程和用户可选valid_range_m；人工范围关闭时仍检查消息本身范围。
  const bool usable = std::isfinite(s.range_m) && s.range_m >= s.range_min_m &&
      s.range_m <= s.range_max_m && (!ruleEnabled(limits) ||
      (s.range_m >= get<double>(limits, "minimum", 0) &&
       s.range_m <= get<double>(limits, "maximum", std::numeric_limits<double>::infinity())));
  add(f, "RANGE_VALUE", usable ? "within message and configured measurement limits"
      : "NaN, negative or out-of-range distance", usable ? OK : ERROR,
      s.range_m, 0, "m", !usable);
  if (!usable) { v["range_status"] = "INVALID_VALUE"; reset(); return f; }
  v["range_status"] = "AVAILABLE";
  v["range_valid"] = "true";
  v["required_checks_available"] = "true";
  // 只有用户明确启用height_limits时才把沿光束距离当作应用阈值对象；这仍不是姿态标定后的垂直高度。
  const auto h = quality_["height_limits"];
  v["range_height_limits_status"] = ruleEnabled(h) ? "ENABLED" : "DISABLED";
  if (ruleEnabled(h)) {
    threshold(f, "RANGE_HEIGHT_LOW", s.range_m,
              bounds(h["minimum_m"], true, {.2,.1,true}), "m");
    threshold(f, "RANGE_HEIGHT_HIGH", s.range_m,
              bounds(h["maximum_m"], false, {20,30,false}), "m");
  }
  const auto c = quality_["continuity"];
  if (!ruleEnabled(c)) {
    v["range_continuity_status"] = "DISABLED";
    reset();
    return f;
  }
  if (!s.valid || !std::isfinite(s.stamp) || s.stamp <= 0) {
    v["range_continuity_status"] = "INVALID_SAMPLE_OR_TIME";
    reset();
    return f;
  }
  // continuity关闭或源时间/坐标系不连续时不计算差分；首个有效值只建立基线。
  const double dt = s.stamp - stamp_;
  const bool compare = have_previous_ && frame_ == s.frame &&
      dt >= get<double>(c, "dt_min_sec", .01) && dt <= get<double>(c, "dt_max_sec", 1.2);
  v["range_continuity_status"] = compare ? "AVAILABLE" : "WARMUP_OR_TIME_FRAME_RESET";
  if (compare) {
    const double delta = s.range_m - range_;
    v["range_delta_m"] = number(delta);
    v["range_rate_mps"] = number(delta / dt);
    threshold(f, "RANGE_JUMP", std::abs(delta),
              bounds(c["jump_m"], false, {1,3,false}), "m");
    threshold(f, "RANGE_RATE", std::abs(delta / dt),
              bounds(c["rate_abs_mps"], false, {3,8,false}), "m/s");
  }
  stamp_ = s.stamp;
  range_ = s.range_m;
  frame_ = s.frame;
  have_previous_ = true;
  return f;
}
// 检查矩阵有限、对角非负，并按配置检查对称和半正定；不把未知零矩阵当零噪声。
bool CovarianceChecker::valid(const Eigen::MatrixXd& m, double tol,
                              bool symmetry, bool psd) {
  // 对角不能显著为负；可选的PSD检查使用对称部分求特征值以容忍浮点微小不对称。
  if (!m.allFinite() || (m.diagonal().array() < -tol).any()) return false;
  if (symmetry && (m - m.transpose()).cwiseAbs().maxCoeff() > tol) return false;
  if (psd) {
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es((m + m.transpose()) * .5,
                                                      Eigen::EigenvaluesOnly);
    if (es.info() != Eigen::Success || es.eigenvalues().minCoeff() < -tol)
      return false;
  }
  return true;
}
// 清空位置/速度差分基线，避免跨实例或无效时间窗产生虚假的大速度。
void KinematicsTracker::reset() {
  have_pose_ = have_velocity_ = false;
  frame_.clear();
}
// 在同参考系有效时间窗内求速度、向量加速度和预测残差，再比较显式选择的规则。
Findings KinematicsTracker::update(const Sample& s, const YAML::Node& q,
                                   Values& v) {
  Findings f;
  // 复制本次quality节点，后续只读；kinematics/jump可分别关闭，两个规则组都关闭时立即清基线。
  auto k = q["kinematics"] ? YAML::Clone(q["kinematics"]) : YAML::Node(YAML::NodeType::Map);
  auto jump = q["jump"];
  if (!ruleEnabled(k) && !ruleEnabled(jump)) { reset(); return f; }
  if (!k || k.IsNull()) k = YAML::Node(YAML::NodeType::Map);
  double dt = s.stamp - stamp_;
  v["linear_speed_mps"] = v["linear_acceleration_mps2"] =
      v["angular_speed_rps"] = v["prediction_residual_m"] = "N/A";
  bool continuous = get<std::string>(q, "continuity", "continuous_odometry") ==
                    "continuous_odometry";
  bool twist =
      get<std::string>(k, "velocity_source", "position_difference") == "twist";
  // 位置/姿态/time任一无效就不能安全差分；清除旧基线，防止下一帧跨过坏数据得出巨大速度。
  if (!s.valid || !s.position.allFinite() ||
      !s.orientation.coeffs().allFinite() || s.orientation.norm() < 1e-12 ||
      !std::isfinite(s.stamp) || s.stamp <= 0) {
    have_pose_ = have_velocity_ = false;
    return f;
  }
  // 位置差分只在同一参考坐标系有效；frame变化需要明确重置，不能解释成机器人瞬移。
  if (!frame_.empty() && s.frame != frame_) {
    add(f, "LOC_FRAME_CHANGE", "unauthorized frame change; reset required",
        ERROR, 0, 0, "", true);
    have_pose_ = have_velocity_ = false;
    return f;
  }
  add(f, "LOC_FRAME_CHANGE", "frame unchanged", OK);
  if (have_pose_ && dt <= 0) {
    have_pose_ = have_velocity_ = false;
    return f;
  }
  // 源时间必须前进且dt在允许范围；频率异常或长暂停后的首帧只重建基线，不跨长间隔求导。
  bool valid_dt = have_pose_ && dt >= positive(k, "dt_min_sec", .001) &&
                  dt <= positive(k, "dt_max_sec", 1);
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  double midpoint = (s.stamp + stamp_) * .5;
  if (valid_dt) {
    // 速度来源二选一：由相邻位置差分得到，或使用消息twist并旋转到当前姿态对应参考系。
    velocity = twist ? s.orientation.normalized() * s.twist
                     : (s.position - position_) / dt;
    if (twist) midpoint = s.stamp;
    v["linear_speed_mps"] = number(velocity.stableNorm());
    threshold(f, "LOC_SPEED", velocity.stableNorm(),
              bounds(ruleEnabled(k) ? k["speed_mps"] : YAML::Node(), false, {15, 50, false}), "m/s");
    // A map orientation correction rotates a world-expressed twist without
    // physical acceleration. Global outputs need an independent continuous
    // motion frame before vector derivatives may be interpreted as physics.
    // 加速度按两次有效速度的向量差分计算；地图全局跳变模式不把坐标系校正当作物理加速度。
    if (continuous && have_velocity_ && midpoint > midpoint_) {
      double acceleration =
          ((velocity - velocity_) / (midpoint - midpoint_)).stableNorm();
      v["linear_acceleration_mps2"] = number(acceleration);
      threshold(f, "LOC_ACCELERATION", acceleration,
                bounds(ruleEnabled(k) ? k["acceleration_mps2"] : YAML::Node(), false, {10, 30, false}),
                "m/s^2");
      // 用上一速度外推当前位置，当前位置与预测位置差作为连续定位残差/跳变指标。
      if (continuous) {
        double residual =
            (s.position - (position_ + velocity_ * dt)).stableNorm();
        v["prediction_residual_m"] = number(residual);
        threshold(f, "LOC_JUMP", residual,
                  bounds(ruleEnabled(jump) ? jump["residual_m"] : YAML::Node(), false, {.5, 2, false}), "m");
      }
    }
    // 姿态角速度与旋转跳变使用四元数最短角距离，避免Euler角跨±180度产生假跳变。
    if (continuous) {
      double dot = std::min(
          1.0,
          std::abs(orientation_.normalized().dot(s.orientation.normalized())));
      double angle = 2 * std::acos(dot);
      v["angular_speed_rps"] = number(angle / dt);
      threshold(f, "LOC_ANGULAR_SPEED", angle / dt,
                bounds(ruleEnabled(k) ? k["angular_speed_rps"] : YAML::Node(), false, {3, 10, false}), "rad/s");
      threshold(f, "LOC_ROTATION_JUMP", angle,
                bounds(ruleEnabled(jump) ? jump["rotation_rad"] : YAML::Node(), false, {.3, 1, false}),
                "rad");
    }
    velocity_ = velocity;
    midpoint_ = midpoint;
    have_velocity_ = true;
  } else
    have_velocity_ = false;
  // 有效样本无论是否形成速度差分都会更新下一帧基线；不合法样本已经在前面提前退出。
  position_ = s.position;
  orientation_ = s.orientation;
  stamp_ = s.stamp;
  frame_ = s.frame;
  have_pose_ = true;
  // A single severe position/rotation jump must be observable. Soft dwell is
  // appropriate for warnings; ERROR motion bounds represent immediate faults.
  for (auto& finding : f) {
    if (finding.level == ERROR) finding.hard = true;
  }
  return f;
}
// 合并姿态、运动学、协方差规则；协方差未知及必需检查不可用时如实标记。
Findings LocalizationChecker::check(const Sample& s, Values& v) {
  v = s.values;
  Findings f = s.findings;
  // 姿态和运动学共用同一Sample，但各自维护时间/姿态基线；适配器已有异常也继续保留。
  auto attitude = attitude_.update(s, v);
  f.insert(f.end(), attitude.begin(), attitude.end());
  const bool attitude_available = v["required_checks_available"] != "false";
  auto motion = motion_.update(s, quality_, v);
  f.insert(f.end(), motion.begin(), motion.end());
  // 仅当协方差规则启用时读取估计；全零矩阵默认视为“未知”，不能当成零不确定度。
  auto c = quality_["covariance"];
  if (ruleEnabled(c)) {
    bool zero = s.covariance.isZero(0);
    auto policy = get<std::string>(c, "zero_matrix_policy", "unknown_warning");
    bool known = s.covariance_available && (!zero || policy == "valid");
    v["covariance_status"] = known ? "AVAILABLE" : "UNKNOWN";
    v["checks_available"] = known ? "pose,motion,covariance"
                                  : "pose,motion; covariance unavailable";
    v["required_checks_available"] =
        ((!known && get<bool>(c, "required_for_valid", false)) || !attitude_available) ? "false"
                                                              : "true";
    if (!known) {
      add(f, "LOC_COVARIANCE_UNKNOWN", "covariance estimate unavailable",
          policy == "unknown_warning" ? WARNING : OK);
    } else {
      add(f, "LOC_COVARIANCE_UNKNOWN", "known covariance", OK);
      // 先验证矩阵数值结构，再把位置/姿态对角最大值当作对应方差指标检查上限。
      bool legal = CovarianceChecker::valid(
          s.covariance, positive(c, "numeric_tolerance", 1e-9),
          get<bool>(c, "require_symmetric", false),
          get<bool>(c, "require_psd", false));
      add(f, "LOC_COVARIANCE_INVALID",
          legal ? "valid covariance"
                : "nonfinite/asymmetric/non-PSD covariance",
          legal ? OK : ERROR, 0, 0, "", !legal);
      if (legal) {
        double p = s.covariance.diagonal().head<3>().maxCoeff(),
               r = s.covariance.diagonal().tail<3>().maxCoeff();
        v["position_variance_m2"] = number(p);
        v["orientation_variance_rad2"] = number(r);
        threshold(f, "LOC_POSITION_VARIANCE", p,
                  bounds(c["position_variance_m2"], false, {1, 4, false}),
                  "m^2");
        threshold(
            f, "LOC_ORIENTATION_VARIANCE", r,
            bounds(c["orientation_variance_rad2"], false, {.1, .5, false}),
            "rad^2");
      }
    }
  } else {
    v["covariance_status"] = "DISABLED";
    v["checks_available"] = "pose,motion; covariance disabled";
  }
  return f;
}
// 根据配置中的checker选择C++规则对象，消息适配与业务判断因此可以独立扩展。
std::unique_ptr<IHealthChecker> CheckerFactory::create(const ModuleConfig& c) {
  if (c.checker == "LidarChecker")
    return std::unique_ptr<IHealthChecker>(new LidarChecker(c.quality));
  if (c.checker == "ImuChecker")
    return std::unique_ptr<IHealthChecker>(new ImuChecker(c.quality));
  if (c.checker == "RangeChecker")
    return std::unique_ptr<IHealthChecker>(new RangeChecker(c.quality));
  if (c.checker == "LocalizationChecker")
    return std::unique_ptr<IHealthChecker>(new LocalizationChecker(c.quality));
  if (c.checker == "RelayChecker" || c.checker == "PlanningChecker")
    return std::unique_ptr<IHealthChecker>(new RelayChecker());
  throw std::invalid_argument("unregistered checker");
}
// 设置启动/停止/控制心跳期限；严格模式先UNKNOWN，普通模式先STARTING。
LifecycleTracker::LifecycleTracker(const LifecycleConfig& c,
                                   const MonitorConfig& m, double now)
    : config_(c),
      strict_(m.strict_control),
      heartbeat_warning_(m.heartbeat_warning),
      heartbeat_error_(m.heartbeat_error),
      transition_(now),
      state_(m.strict_control ? "UNKNOWN" : "STARTING") {}
// 校验控制序号、实例和代际，拒绝旧心跳；将RUN/PAUSE/STOP意图转换为受控生命周期。
bool LifecycleTracker::control(const RuntimeUpdate& u, double now) {
  control_accepted_ = false;
  // 拒绝缺少身份/序号或使用未知动作/状态的消息；这类消息不会刷新心跳时间。
  if (u.controller.empty() || u.instance.empty() || u.sequence == 0)
    return false;
  if (u.desired != "RUN" && u.desired != "STOP" && u.desired != "PAUSE")
    return false;
  if (u.observed != "STARTING" && u.observed != "RUNNING" &&
      u.observed != "STOPPING" && u.observed != "STOPPED" &&
      u.observed != "FAULT" && u.observed != "PAUSED")
    return false;
  if (controlled_ && u.controller == update_.controller &&
      u.sequence <= update_.sequence)
    return false;
  if (retired_controllers_.count(u.controller) ||
      retired_instances_.count(u.controller + "/" + u.instance))
    return false;
  if (controlled_ && u.controller == update_.controller &&
      u.instance == update_.instance && u.frame_epoch < update_.frame_epoch)
    return false;
  // 控制器、进程实例或授权数据代际变化表示新一轮输入，应重启就绪流程并清理跨代基线。
  const bool reset = !controlled_ || u.controller != update_.controller ||
                     u.instance != update_.instance ||
                     u.frame_epoch != update_.frame_epoch;
  const bool changed =
      reset || u.desired != update_.desired || u.observed != update_.observed;
  if (changed) {
    transition_ = now;
    good_samples_ = 0;
  }
  heartbeat_ = now;
  if (controlled_ && u.controller != update_.controller)
    retired_controllers_.insert(update_.controller);
  if (controlled_ &&
      (u.controller != update_.controller || u.instance != update_.instance))
    retired_instances_.insert(update_.controller + "/" + update_.instance);
  controlled_ = true;
  control_accepted_ = true;
  update_ = u;
  if (reset) ++epoch_;
  // desired表示请求，observed表示控制器确认的实际状态；仅收到STOP意图还处于STOPPING。
  if (u.desired == "STOP")
    state_ = u.observed == "STOPPED" ? "STOPPED"
             : u.observed == "FAULT" ? "FAULT"
                                     : "STOPPING";
  else if (u.desired == "PAUSE")
    state_ = u.observed == "PAUSED" ? "PAUSED" : "FAULT";
  else if (u.observed == "FAULT" || u.observed == "STOPPED")
    state_ = "FAULT";
  else if (reset || changed)
    state_ = "STARTING";
  return reset;
}
// 只累计连续合格样本；坏样本清零，不用消息存在代替质量合格。
void LifecycleTracker::sample(bool good, double) {
  if (good)
    ++good_samples_;
  else
    good_samples_ = 0;
}
// 可选模块首帧到来后重新开始启动准备期限；不清除真实控制器故障。
void LifecycleTracker::beginInput(double now) {
  // Optional inputs can appear long after launch. Start their readiness deadline
  // at the first message; do not reset explicit process/heartbeat failures.
  if (state_ == "STARTING") transition_ = now;
}
// 定时检查控制新鲜度、启动/停止确认和输出准备，产生STARTING/RUNNING/PAUSED/STOPPED/FAULT。
LifecycleReport LifecycleTracker::evaluate(double now, bool ready, double age,
                                          bool wait_for_first_sample) {
  LifecycleReport r;
  // 严格控制模式必须收到实时意图；未接入控制器时不能猜测模块已经正常停止。
  if (strict_ && !controlled_) {
    state_ = "UNKNOWN";
    add(r.findings, "CONTROL_UNKNOWN", "no fresh runtime intention", ERROR, 0,
        0, "", true);
  } else if (controlled_ && now - heartbeat_ > heartbeat_error_) {
    state_ = "UNKNOWN";
    add(r.findings, "CONTROL_STALE", "runtime controller heartbeat lost", ERROR,
        now - heartbeat_, heartbeat_error_, "s", true);
  } else {
    add(r.findings, "CONTROL_UNKNOWN", "runtime intention available", OK);
    add(r.findings, "CONTROL_STALE", "controller heartbeat current", OK);
    if (controlled_)
      add(r.findings, "CONTROL_TRACE",
          update_.trace_healthy ? "runtime trace active"
                                : "runtime control trace incomplete",
          update_.trace_healthy ? OK : ERROR, 0, 0, "", !update_.trace_healthy);
    if (controlled_)
      add(r.findings, "CONTROL_HEARTBEAT", "controller heartbeat age",
          now - heartbeat_ > heartbeat_warning_ ? WARNING : OK,
          now - heartbeat_, heartbeat_warning_, "s");
    if (state_ == "UNKNOWN")
      state_ = update_.desired == "STOP" && update_.observed == "STOPPED"
                   ? "STOPPED"
                   : "STARTING";
    // 计划停止要等控制器确认STOPPED；未在期限内确认则报STOP_FAILED。
    if (state_ == "STOPPING" && now - transition_ > config_.stop) {
      state_ = "FAULT";
      add(r.findings, "STOP_FAILED", "stop confirmation timed out", ERROR,
          now - transition_, config_.stop, "s", true);
    }
    // 启动期限内需要流统计ready并累计连续合格样本；允许首帧前NO_DATA的可选模块暂停该期限。
    if (state_ == "STARTING") {
      if (ready && good_samples_ >= config_.ready_samples &&
          (!controlled_ || update_.observed == "RUNNING"))
        state_ = "RUNNING";
      else if (!wait_for_first_sample && now - transition_ > config_.startup) {
        state_ = "FAULT";
        add(r.findings, "STARTUP_TIMEOUT",
            "required output not ready by deadline", ERROR, now - transition_,
            config_.startup, "s", true);
      }
    }
    // 调用方用负age表示“硬接收年龄已越过ERROR”；此时立刻把运行状态转成FAULT。
    if (state_ == "RUNNING" && age < 0) {
      state_ = "FAULT";
      good_samples_ = 0;
    }  // Negative sentinel: output exceeded configured hard receive age.
    // 故障后的恢复需要进程仍处RUNNING、流重新ready且达到连续好样本数，不按单帧恢复。
    if (state_ == "FAULT" && update_.desired != "STOP" &&
        (!controlled_ || update_.observed == "RUNNING") && ready &&
        good_samples_ >= config_.ready_samples)
      state_ = "RUNNING";
    if (state_ == "FAULT")
      add(r.findings, "RUNTIME_FAULT", "startup/process/output/stop failure",
          ERROR, 0, 0, "", true);
    else {
      add(r.findings, "RUNTIME_FAULT", "runtime state healthy", OK);
      add(r.findings, "STARTUP_TIMEOUT", "startup not timed out", OK);
      add(r.findings, "STOP_FAILED", "stop not timed out", OK);
    }
  }
  r.state = state_;
  // expected_active决定是否继续要求输入；valid_state只在明确RUNNING时为true。
  r.expected_active = !(state_ == "STOPPED" || state_ == "PAUSED");
  r.valid_state = state_ == "RUNNING";
  return r;
}
// 把同一模块的时间探针、生命周期、质量规则和消抖器组合起来，保持各自职责独立。
ModuleMonitor::ModuleMonitor(const ModuleConfig& c, const MonitorConfig& m,
                             EventSink sink, double now)
    : config_(c),
      stream_(c.stream, m),
      lifecycle_(c.lifecycle, m, now),
      rules_(m.soft_hold, m.recovery_hold, sink),
      control_rules_(m.soft_hold, m.recovery_hold, sink),
      checker_(CheckerFactory::create(c)),
      sink_(std::move(sink)) {
  // 以本模块第一次建立的单调时刻启动年龄探针；质量阈值和订阅都已在构造前校验。
  stream_.reset(now);
}
// 样本回调主入口：源时间检查 -> 质量规则 -> 每规则状态更新 -> 连续就绪计数。
void ModuleMonitor::receive(const Sample& s) {
  // 可选流在首帧前显示NO_DATA且不倒计启动期限；首帧到来时才进入正常准备流程。
  const bool first_optional_sample = config_.allowsInitialNoData() && !have_data_;
  if (first_optional_sample) lifecycle_.beginInput(s.received);
  if (config_.checker == "PlanningChecker" &&
      s.values.count("synthetic_status")) {
    // 规划适配器已经根据TaskState生成规则；这里暂停/空闲时只豁免任务需求和结果期限类规则。
    for (const auto& value : s.values) values_[value.first] = value.second;
    if (lifecycle_.dataDemandEnabled())
      rules_.apply(s.findings, s.received);
    else {
      Findings passed;
      for (const auto& finding : s.findings)
        if (finding.id != "PLANNING_TASK_FRESH" &&
            finding.id != "PLANNING_DEADLINE")
          passed.push_back(finding);
      for (const auto* id : {"PLANNING_TASK_FRESH", "PLANNING_DEADLINE"})
        add(passed, id, "planned inactive/startup demand suspended", OK);
      rules_.apply(passed, s.received);
    }
    lifecycle_.sample(s.valid, s.received);
    have_data_ = true;
    return;
  }
  // 先检查源时间戳，再把严重时间错误标记到副本Sample，避免坏时间进入Checker差分基线。
  auto time = stream_.receive(s.stamp, s.received);
  Sample checked = s;
  if (anyError(time)) checked.valid = false;
  latest_sample_ = checker_->check(checked, values_);
  latest_sample_.insert(latest_sample_.end(), time.begin(), time.end());
  // A rule that occurred on a previous sample but is absent on the next sample
  // explicitly passes, without disturbing timer-owned/control-owned rules.
  std::set<std::string> present;
  for (const auto& f : latest_sample_) present.insert(f.id);
  for (const auto& id : sample_rule_ids_)
    if (!present.count(id))
      latest_sample_.push_back(
          Finding{id, "sample recovered", id, "", OK, 0, 0, false});
  sample_rule_ids_.clear();
  for (const auto& f : latest_sample_) sample_rule_ids_.insert(f.id);
  // 规则恢复只由本样本拥有的规则明确给出OK；最后用质量结果推进生命周期就绪计数。
  rules_.apply(latest_sample_, s.received);
  lifecycle_.sample(s.valid && !anyError(latest_sample_), s.received);
  have_data_ = true;
  if (first_optional_sample && sink_) {
    Finding f;
    f.id = "INPUT_AVAILABLE";
    f.reason = "first optional input received; normal quality/timeout checks and overall participation enabled";
    sink_("INPUT_AVAILABLE", f, OK, OK);
  }
}
// 接受生命周期变化；可信新实例才清历史，计划暂停/停止只暂停数据需求类告警。
bool ModuleMonitor::control(const RuntimeUpdate& u, double now) {
  const auto old = lifecycle_.runtime();
  const bool was_inactive = old.observed == "PAUSED" || old.observed == "STOPPED";
  // LifecycleTracker先验证控制身份和序号；未经接受的消息不能重置检查基线。
  bool reset = lifecycle_.control(u, now);
  if (!lifecycle_.controlAccepted()) return false;
  if (reset) {
    // 新实例/新代际拥有新的时间序列，清除旧stamp、运动学、消抖和显示值。
    stream_.reset(now);
    checker_->reset();
    rules_.reset();
    control_rules_.reset();
    sample_rule_ids_.clear();
    latest_sample_.clear();
    values_.clear();
    have_data_ = false;
  }
  // A confirmed planned pause/stop replaces historical data-demand runtime
  // failures. Current heartbeat/trace failures are evaluated again immediately.
  if (!reset && !was_inactive && (u.observed == "PAUSED" || u.observed == "STOPPED"))
    control_rules_.reset();
  if (sink_ && (reset || u.command != last_command_)) {
    Finding f;
    f.id = "LIFECYCLE";
    f.reason = u.desired + " / " + u.observed + " / " + u.reason;
    sink_(reset ? "INSTANCE_RESET" : "CONTROL_INTENT", f, 0, 0);
    last_command_ = u.command;
  }
  return reset;
}
// 定时组合流健康与生命周期，处理初始NO_DATA、业务有效性和总体健康参与策略。
ModuleReport ModuleMonitor::evaluate(double now) {
  ModuleReport r;
  r.id = config_.id;
  r.checker = config_.checker;
  r.required = config_.required;
  r.contributes_to_overall = contributesToOverall();
  r.topic = config_.topic;
  r.instance = lifecycle_.instance();
  r.command = lifecycle_.command();
  r.epoch = lifecycle_.epoch();
  r.values = values_;
  // 定时路径在无新样本时仍会执行：更新接收年龄/频率并判断流统计是否已预热完成。
  bool ready = false;
  auto stream = stream_.evaluate(now, r.values, ready);
  double age = stream_.received() ? now - stream_.lastReceive() : 0;
  bool planning = config_.checker == "PlanningChecker";
  const bool waiting_for_first_sample = config_.allowsInitialNoData() && !have_data_;
  // 规划消息只在活动任务要求输出时才按结果/任务状态计时，空闲期间不制造路径断流。
  if (planning) {
    ready = have_data_ && values_["task_receive_age_sec"] != "" &&
            (!ruleEnabled(config_.quality["task_state_timeout_sec"]) ||
             std::stod(values_["task_receive_age_sec"]) <=
                ruleValue(config_.quality["task_state_timeout_sec"], 2));
  }
  // 负age是内部硬超时标记；生命周期据此立即FAULT，而报告仍保留真实age数值。
  auto lc = lifecycle_.evaluate(
      now, ready,
      !planning && stream_.received() && config_.stream.age.error_enabled && age > config_.stream.age.error ? -1
                                                                        : age,
      waiting_for_first_sample);
  r.state = lc.state;
  if (waiting_for_first_sample && r.state == "STARTING") r.state = "NO_DATA";
  r.expected_active = lc.expected_active;
  if (r.state == "NO_DATA") r.expected_active = false;
  // A planned stop suspends only data-demand rules; hard invalid messages still
  // reach the sample gate. Startup checks hard samples immediately but wait for
  // the bounded startup deadline before demanding a fully populated stream.
  // PAUSED/STOPPED/STARTING/NO_DATA期间暂停流需求类规则；控制心跳规则仍单独评估。
  if (!planning && r.state != "STOPPED" && r.state != "PAUSED" &&
      r.state != "STOPPING" && r.state != "STARTING" && r.state != "NO_DATA")
    rules_.apply(stream, now);
  else {
    Findings pass;
    for (const auto* id : {"RECEIVE_AGE", "FREQUENCY_LOW", "FREQUENCY_HIGH"})
      add(pass, id, "data demand suspended", OK);
    rules_.apply(pass, now);
  }
  control_rules_.apply(lc.findings, now);
  // 数据等级、生命周期等级和整体有效性分别保存；计划停止不抹掉本周期刚发生的硬错误。
  r.data_level = rules_.level();
  if (r.state == "STOPPED" || r.state == "PAUSED")
    r.data_level = rules_.periodLevel();
  r.level = std::max(r.data_level, control_rules_.level());
  if (!lc.valid_state && r.level < ERROR) r.level = WARNING;
  r.has_data = have_data_ && lc.expected_active && stream_.received() &&
               (!config_.stream.age.error_enabled || age <= config_.stream.age.error);
  if (planning) {
    r.has_data = ready && lc.expected_active;
    r.values["rate_hz"] = "N/A (path demand follows goal)";
    if (lc.valid_state && values_["task_state"] == "IDLE") r.state = "IDLE";
  }
  // valid比等级更严格：必须处于RUNNING、有数据、无ERROR且所有必需检查当前可用。
  r.valid = lc.valid_state && r.has_data && r.level < ERROR &&
            r.values["required_checks_available"] != "false";
  if (r.level == ERROR)
    r.message = rules_.reasons() + "; " + control_rules_.reasons();
  else if (r.state == "NO_DATA")
    r.message = "NO_DATA: optional input has not produced data; waiting for the first message";
  else if (!lc.valid_state)
    r.message = r.state;
  else
    r.message = rules_.reasons();
  r.values["data_health_level"] = r.has_data ? levelName(r.data_level) : "N/A";
  r.values["missing_data_policy"] = config_.missing_data_policy;
  r.values["data_presence"] = have_data_ ? "HAS_RECEIVED" : "NO_DATA";
  r.values["contributes_to_overall"] = r.contributes_to_overall ? "true" : "false";
  if (waiting_for_first_sample && config_.checker == "RangeChecker") {
    for (const auto* key : {"range_raw_m", "range_m", "range_delta_m", "range_rate_mps"})
      r.values[key] = "N/A";
    r.values["range_status"] = "NO_DATA";
    r.values["range_valid"] = r.values["required_checks_available"] = "false";
  }
  r.values["operating_state"] = r.state;
  r.values["control_reason"] = lifecycle_.runtime().reason;
  r.values["operational_valid"] = r.valid ? "true" : "false";
  r.values["has_data_health"] = r.has_data ? "true" : "false";
  r.values["expected_active"] = r.expected_active ? "true" : "false";
  r.values["sample_status"] = waiting_for_first_sample ? "N/A" : levelName(r.data_level);
  r.values["sample_message"] = r.message;
  r.values["module_epoch"] = std::to_string(r.epoch);
  r.values["producer_instance_id"] = r.instance;
  if (sink_ && r.state != last_state_) {
    Finding f;
    f.id = "LIFECYCLE";
    f.reason = last_state_ + " -> " + r.state;
    sink_("STATE_CHANGED", f, 0, r.level);
    last_state_ = r.state;
  }
  rules_.published();
  control_rules_.published();
  return r;
}
}  // namespace robot_health
