#include <fastlio_health_monitor/health_core.h>
#include <unistd.h>

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <random>
#include <set>
#include <sstream>
#include <stdexcept>

namespace robot_health {
namespace {
void keys(const YAML::Node& node, const std::string& path,
          std::initializer_list<const char*> allowed) {
  if (!node || !node.IsMap())
    throw std::invalid_argument(path + " must be a mapping");
  std::set<std::string> names;
  for (const auto* s : allowed) names.insert(s);
  for (const auto& entry : node) {
    if (!names.count(entry.first.as<std::string>()))
      throw std::invalid_argument(path + ": unsupported field " +
                                  entry.first.as<std::string>());
  }
}
template <class T>
T get(const YAML::Node& node, const char* key, T fallback) {
  return node[key] ? node[key].as<T>() : fallback;
}
double positive(const YAML::Node& n, const char* key, double fallback,
                bool zero = false) {
  double v = get<double>(n, key, fallback);
  if (!std::isfinite(v) || v < 0 || (!zero && v == 0))
    throw std::invalid_argument(std::string(key) + " must be finite and " +
                                (zero ? "nonnegative" : "positive"));
  return v;
}
Bounds bounds(const YAML::Node& n, bool lower, Bounds defaults) {
  if (!n) return defaults;
  keys(
      n, "bounds",
      lower
          ? std::initializer_list<const char*>{"warning_below", "error_below"}
          : std::initializer_list<const char*>{"warning_above", "error_above"});
  Bounds b{
      positive(n, lower ? "warning_below" : "warning_above", defaults.warning,
               true),
      positive(n, lower ? "error_below" : "error_above", defaults.error, true),
      lower};
  if ((lower && b.error >= b.warning) || (!lower && b.warning >= b.error))
    throw std::invalid_argument("threshold ordering invalid");
  return b;
}
int parseLevel(const YAML::Node& n, const char* key, const char* def) {
  const std::string s = get<std::string>(n, key, def);
  if (s == "WARNING") return WARNING;
  if (s == "ERROR") return ERROR;
  throw std::invalid_argument(std::string(key) + " must be WARNING or ERROR");
}
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
void add(Findings& out, const std::string& id, const std::string& reason,
         int level, double value = 0, double limit = 0,
         const std::string& unit = "", bool hard = false) {
  out.push_back(Finding{id, reason, id, unit, level, value, limit, hard});
}
void threshold(Findings& out, const std::string& id, double value,
               const Bounds& b, const std::string& unit) {
  int l = std::isfinite(value) ? b.level(value) : ERROR;
  add(out, id, l ? "outside configured range" : "within range", l, value,
      l == ERROR ? b.error : b.warning, unit, !std::isfinite(value));
}
bool anyError(const Findings& f) {
  return std::any_of(f.begin(), f.end(),
                     [](const Finding& v) { return v.level == ERROR; });
}
void validateCovariance(const YAML::Node& c, bool imu) {
  if (!c) return;
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
    if (c[name]) c[name].as<bool>();
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
void qualityValidate(const ModuleConfig& m) {
  const auto q = m.quality;
  if (m.checker == "LidarChecker") {
    keys(q, m.id + ".quality",
         {"points_total", "points_valid", "valid_ratio", "range_m"});
    bounds(q["points_total"], true, {100, 1, true});
    bounds(q["points_valid"], true, {100, 1, true});
    auto b = bounds(q["valid_ratio"], true, {.8, .5, true});
    if (b.warning > 1) throw std::invalid_argument("valid_ratio exceeds 1");
    keys(q["range_m"], "range_m", {"minimum", "maximum"});
    if (positive(q["range_m"], "minimum", .0001, true) >=
        positive(q["range_m"], "maximum", 500))
      throw std::invalid_argument("range order invalid");
    if (m.adapter != "LivoxCustomAdapter" && m.adapter != "PointCloud2Adapter")
      throw std::invalid_argument("unsupported lidar adapter");
  } else if (m.checker == "ImuChecker") {
    keys(q, m.id + ".quality",
         {"acceleration_unit", "angular_velocity_unit", "acceleration_content",
          "required_measurements", "acceleration_norm_mps2",
          "angular_velocity_norm_rps", "covariance"});
    const auto u = get<std::string>(q, "acceleration_unit", "");
    if (u != "g" && u != "mps2")
      throw std::invalid_argument("acceleration_unit must be g or mps2");
    if (get<std::string>(q, "angular_velocity_unit", "radps") != "radps")
      throw std::invalid_argument("only radps is supported");
    auto content =
        get<std::string>(q, "acceleration_content", "specific_force");
    if (content != "specific_force" && content != "gravity_removed")
      throw std::invalid_argument("invalid acceleration_content");
    if (q["required_measurements"])
      keys(q["required_measurements"], "required_measurements",
           {"acceleration", "angular_velocity", "orientation"});
    if (q["required_measurements"])
      for (const auto& field : q["required_measurements"])
        field.second.as<bool>();
    bounds(q["acceleration_norm_mps2"], false, {100, 200, false});
    bounds(q["angular_velocity_norm_rps"], false, {35, 70, false});
    validateCovariance(q["covariance"], true);
    if (m.adapter != "RosImuAdapter")
      throw std::invalid_argument("unsupported imu adapter");
  } else if (m.checker == "LocalizationChecker") {
    keys(q, m.id + ".quality",
         {"expected_pose_frame", "expected_child_frame", "continuity",
          "quaternion_norm_error", "kinematics", "covariance", "jump"});
    if (m.adapter != "OdometryAdapter" && m.adapter != "PoseStampedAdapter")
      throw std::invalid_argument("unsupported localization adapter");
    auto k = q["kinematics"];
    keys(k, "kinematics",
         {"velocity_source", "dt_min_sec", "dt_max_sec", "speed_mps",
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
    bounds(k["speed_mps"], false, {15, 50, false});
    bounds(k["acceleration_mps2"], false, {10, 30, false});
    bounds(k["angular_speed_rps"], false, {3, 10, false});
    validateCovariance(q["covariance"], false);
    keys(q["jump"], "jump", {"residual_m", "rotation_rad"});
    bounds(q["jump"]["residual_m"], false, {.5, 2, false});
    bounds(q["jump"]["rotation_rad"], false, {.3, 1, false});
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
      positive(q, name, 2);
    if (!q["task_state_topic"] ||
        q["task_state_topic"].as<std::string>().empty())
      throw std::invalid_argument("planning requires task_state_topic");
    if (m.adapter != "PathAndTaskStateAdapter")
      throw std::invalid_argument("unsupported planning adapter");
  } else
    throw std::invalid_argument("checker not registered: " + m.checker);
}
}  // namespace

double steadyNow() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
std::string number(double value) {
  if (!std::isfinite(value)) return "N/A";
  std::ostringstream out;
  out << std::setprecision(12) << value;
  return out.str();
}
std::string uniqueId() {
  std::random_device rd;
  std::ostringstream out;
  out << std::hex << getpid() << '-'
      << std::chrono::high_resolution_clock::now().time_since_epoch().count()
      << '-' << rd();
  return out.str();
}
const char* levelName(int l) {
  return l == OK ? "OK" : l == WARNING ? "WARNING" : "ERROR";
}
int Bounds::level(double v) const {
  return lower ? (v < error     ? ERROR
                  : v < warning ? WARNING
                                : OK)
               : (v > error     ? ERROR
                  : v > warning ? WARNING
                                : OK);
}

MonitorConfig ConfigLoader::load(const std::string& path) {
  MonitorConfig c;
  c.source_path = path;
  std::ifstream in(path);
  if (!in) throw std::invalid_argument("cannot open config: " + path);
  c.source_text.assign(std::istreambuf_iterator<char>(in), {});
  YAML::Node root = YAML::Load(c.source_text);
  keys(root, "root",
       {"schema_version", "monitor", "control", "stream_defaults",
        "lifecycle_defaults", "logging", "modules"});
  if (get<int>(root, "schema_version", 0) != 2)
    throw std::invalid_argument("schema_version must be 2");
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
  n = root["control"];
  keys(n, "control",
       {"strict_intent", "state_topic", "heartbeat_warning_sec",
        "heartbeat_error_sec"});
  c.strict_control = get<bool>(n, "strict_intent", false);
  c.state_topic = get<std::string>(n, "state_topic", c.state_topic);
  c.heartbeat_warning = positive(n, "heartbeat_warning_sec", 1);
  c.heartbeat_error = positive(n, "heartbeat_error_sec", 3);
  if (c.heartbeat_warning >= c.heartbeat_error)
    throw std::invalid_argument("heartbeat bounds invalid");
  n = root["logging"];
  keys(n, "logging",
       {"enabled", "directory", "metrics_period_sec", "metrics_queue_capacity",
        "critical_event_queue_capacity", "rotate_size_mb", "rotate_file_count",
        "total_budget_mb", "flush_period_sec"});
  c.logging = get<bool>(n, "enabled", true);
  c.log_directory = get<std::string>(n, "directory", "");
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
  if (!root["modules"] || !root["modules"].IsSequence() ||
      root["modules"].size() == 0)
    throw std::invalid_argument("modules must be a nonempty list");
  std::set<std::string> ids;
  for (size_t i = 0; i < root["modules"].size(); ++i) {
    auto m = root["modules"][i];
    keys(m, "module",
         {"id", "checker", "adapter", "topic", "required", "depends_on",
          "stream", "lifecycle", "quality", "visualization_topic",
          "visualization_frame"});
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
    if (m["depends_on"])
      mc.dependencies = m["depends_on"].as<std::vector<std::string>>();
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
    mc.stream.age = bounds(s["receive_age_sec"], false, {.5, 1.5, false});
    if (!s["frequency"]) s["frequency"] = YAML::Node(YAML::NodeType::Map);
    auto f = s["frequency"];
    keys(f, "frequency",
         {"warning_below_hz", "error_below_hz", "warning_above_hz",
          "error_above_hz"});
    mc.stream.low_rate = {positive(f, "warning_below_hz", 5, true),
                          positive(f, "error_below_hz", 1, true), true};
    mc.stream.high_rate = {positive(f, "warning_above_hz", 15),
                           positive(f, "error_above_hz", 30), false};
    if (mc.stream.low_rate.error >= mc.stream.low_rate.warning ||
        mc.stream.low_rate.warning >= mc.stream.high_rate.warning ||
        mc.stream.high_rate.warning >= mc.stream.high_rate.error)
      throw std::invalid_argument("frequency ordering invalid");
    auto st = s["stamp"];
    keys(st, "stamp",
         {"require_positive", "duplicate_level", "regression_level",
          "max_gap_warning_sec", "max_gap_error_sec"});
    mc.stream.require_positive = get<bool>(st, "require_positive", true);
    mc.stream.duplicate_level = parseLevel(st, "duplicate_level", "WARNING");
    mc.stream.regression_level = parseLevel(st, "regression_level", "ERROR");
    mc.stream.gap = {positive(st, "max_gap_warning_sec", .3),
                     positive(st, "max_gap_error_sec", 1), false};
    if (mc.stream.gap.warning >= mc.stream.gap.error)
      throw std::invalid_argument("stamp gap ordering invalid");
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
    mc.quality = m["quality"];
    mc.visual_topic = get<std::string>(m, "visualization_topic", "");
    mc.visual_frame = get<std::string>(m, "visualization_frame", "");
    qualityValidate(mc);
    s["receive_age_sec"]["warning_above"] = mc.stream.age.warning;
    s["receive_age_sec"]["error_above"] = mc.stream.age.error;
    s["frequency"]["warning_below_hz"] = mc.stream.low_rate.warning;
    s["frequency"]["error_below_hz"] = mc.stream.low_rate.error;
    s["frequency"]["warning_above_hz"] = mc.stream.high_rate.warning;
    s["frequency"]["error_above_hz"] = mc.stream.high_rate.error;
    m["stream"] = s;
    m["lifecycle"] = l;
    c.modules.push_back(mc);
  }
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
  YAML::Emitter out;
  out << root;
  c.effective_text = out.c_str();
  return c;
}

StreamTracker::StreamTracker(const StreamConfig& c, const MonitorConfig& m)
    : config_(c),
      window_(m.window),
      coverage_(m.frequency_coverage),
      min_samples_(m.frequency_samples) {
  reset(steadyNow());
}
void StreamTracker::reset(double now) {
  started_ = now;
  last_receive_ = last_stamp_ = 0;
  count_ = 0;
  receipts_.clear();
  sources_.clear();
}
Findings StreamTracker::receive(double stamp, double now) {
  Findings f;
  const bool valid =
      std::isfinite(stamp) && (!config_.require_positive || stamp > 0);
  add(f, "STAMP_INVALID",
      valid ? "valid timestamp" : "zero/nonfinite timestamp",
      valid ? OK : ERROR, stamp, 0, "s", !valid);
  if (valid && count_ && last_stamp_ > 0) {
    if (stamp == last_stamp_)
      add(f, "STAMP_DUPLICATE", "duplicate source timestamp",
          config_.duplicate_level, stamp, 0, "s",
          config_.duplicate_level == ERROR);
    else if (stamp < last_stamp_) {
      add(f, "STAMP_REGRESSION", "unauthorized source timestamp regression",
          config_.regression_level, stamp, last_stamp_, "s", true);
    } else
      threshold(f, "STAMP_GAP", stamp - last_stamp_, config_.gap, "s");
  }
  if (valid && (last_stamp_ <= 0 || stamp > last_stamp_)) {
    sources_.push_back({now, stamp});
    last_stamp_ = stamp;
  }
  last_receive_ = now;
  receipts_.push_back(now);
  ++count_;
  return f;
}
Findings StreamTracker::evaluate(double now, Values& v, bool& ready) {
  Findings f;
  ready = false;
  while (!receipts_.empty() && now - receipts_.front() > window_)
    receipts_.pop_front();
  while (!sources_.empty() &&
         (now - sources_.front().first > window_ ||
          sources_.back().second - sources_.front().second > window_))
    sources_.pop_front();
  const double age =
      count_ ? std::max(0.0, now - last_receive_) : now - started_;
  v["age_sec"] = count_ ? number(age) : "N/A";
  v["message_count"] = std::to_string(count_);
  v["last_stamp_sec"] = number(last_stamp_);
  v["stamp_domain"] = config_.stamp_domain;
  v["receive_rate_hz"] = number(
      receipts_.size() / std::min(window_, std::max(1e-9, now - started_)));
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
  ready = samples >= min_samples_ && duration >= coverage_ &&
          age <= config_.age.warning;
  const double rate = config_.frequency_basis == "source"
                          ? source_rate
                          : receipts_.size() / std::max(1e-9, duration);
  v["rate_hz"] = ready ? number(rate) : "N/A";
  v["frequency_status"] = ready ? "AVAILABLE" : "WARMING_UP_OR_STALE";
  if (ready) {
    threshold(f, "FREQUENCY_LOW", rate, config_.low_rate, "Hz");
    threshold(f, "FREQUENCY_HIGH", rate, config_.high_rate, "Hz");
  }
  threshold(f, "RECEIVE_AGE", age, config_.age, "s");
  v["transport_delay_sec"] = "N/A (no verified clock mapping)";
  return f;
}

RuleLatch::RuleLatch(double s, double r, EventSink sink)
    : soft_hold_(s), recovery_hold_(r), sink_(std::move(sink)) {}
void RuleLatch::apply(const Findings& findings, double now) {
  std::map<std::string, Finding> current;
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
      s.finding = f;
      s.good_since = -1;
      if (s.candidate != f.level) {
        s.candidate = f.level;
        s.since = now;
        if (sink_) sink_("WARNING_CANDIDATE", f, s.active, f.level);
      }
      if (f.hard || now - s.since >= soft_hold_) {
        if (s.active != f.level) {
          int before = s.active;
          s.active = f.level;
          if (sink_) sink_("RULE_TRIGGERED", f, before, s.active);
        }
        s.period = std::max(s.period, f.level);
      }
    } else {
      s.candidate = OK;
      if (s.active > OK) {
        if (s.good_since < 0) s.good_since = now;
        if (now - s.good_since >= recovery_hold_) {
          int before = s.active;
          s.active = OK;
          if (sink_) sink_("RULE_RECOVERED", f, before, OK);
        }
      }
    }
  }
}
int RuleLatch::level() const {
  int l = OK;
  for (const auto& kv : states_)
    l = std::max(l, std::max(kv.second.active, kv.second.period));
  return l;
}
int RuleLatch::periodLevel() const {
  int level = OK;
  for (const auto& state : states_)
    level = std::max(level, state.second.period);
  return level;
}
std::string RuleLatch::reasons() const {
  std::string s;
  for (const auto& kv : states_)
    if (kv.second.active > OK || kv.second.period > OK) {
      if (!s.empty()) s += "; ";
      s += kv.first + ": " + kv.second.finding.reason;
    }
  return s.empty() ? "OK" : s;
}
void RuleLatch::reset() { states_.clear(); }
void RuleLatch::published() {
  for (auto& kv : states_) kv.second.period = OK;
}

LidarChecker::LidarChecker(const YAML::Node& q)
    : total_(bounds(q["points_total"], true, {100, 1, true})),
      valid_(bounds(q["points_valid"], true, {100, 1, true})),
      ratio_(bounds(q["valid_ratio"], true, {.8, .5, true})) {}
Findings LidarChecker::check(const Sample& s, Values& v) {
  v = s.values;
  Findings f = s.findings;
  v["points_total"] = std::to_string(s.total);
  v["points_valid"] = std::to_string(s.valid_points);
  double ratio = s.total ? double(s.valid_points) / s.total : 0;
  v["valid_ratio"] = number(ratio);
  threshold(f, "LIDAR_COUNT_TOTAL", s.total, total_, "points");
  threshold(f, "LIDAR_COUNT_VALID", s.valid_points, valid_, "points");
  threshold(f, "LIDAR_RATIO", ratio, ratio_, "ratio");
  if (!s.total || !s.valid_points)
    add(f, "LIDAR_EMPTY", "empty scan or no valid points", ERROR,
        s.valid_points, 1, "points", true);
  else
    add(f, "LIDAR_EMPTY", "scan contains valid points", OK);
  return f;
}
ImuChecker::ImuChecker(const YAML::Node& q)
    : acceleration_(
          bounds(q["acceleration_norm_mps2"], false, {100, 200, false})),
      gyro_(bounds(q["angular_velocity_norm_rps"], false, {35, 70, false})) {}
Findings ImuChecker::check(const Sample& s, Values& v) {
  v = s.values;
  Findings f = s.findings;
  if (s.acceleration.allFinite())
    threshold(f, "IMU_ACCEL_NORM", s.acceleration.stableNorm(), acceleration_,
              "m/s^2");
  if (s.gyro.allFinite())
    threshold(f, "IMU_GYRO_NORM", s.gyro.stableNorm(), gyro_, "rad/s");
  return f;
}
bool CovarianceChecker::valid(const Eigen::MatrixXd& m, double tol,
                              bool symmetry, bool psd) {
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
void KinematicsTracker::reset() {
  have_pose_ = have_velocity_ = false;
  frame_.clear();
}
Findings KinematicsTracker::update(const Sample& s, const YAML::Node& q,
                                   Values& v) {
  Findings f;
  auto k = q["kinematics"];
  double dt = s.stamp - stamp_;
  v["linear_speed_mps"] = v["linear_acceleration_mps2"] =
      v["angular_speed_rps"] = v["prediction_residual_m"] = "N/A";
  bool continuous = get<std::string>(q, "continuity", "continuous_odometry") ==
                    "continuous_odometry";
  bool twist =
      get<std::string>(k, "velocity_source", "position_difference") == "twist";
  if (!s.valid || !s.position.allFinite() ||
      !s.orientation.coeffs().allFinite() || s.orientation.norm() < 1e-12 ||
      !std::isfinite(s.stamp) || s.stamp <= 0) {
    have_pose_ = have_velocity_ = false;
    return f;
  }
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
  bool valid_dt = have_pose_ && dt >= positive(k, "dt_min_sec", .001) &&
                  dt <= positive(k, "dt_max_sec", 1);
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  double midpoint = (s.stamp + stamp_) * .5;
  if (valid_dt) {
    velocity = twist ? s.orientation.normalized() * s.twist
                     : (s.position - position_) / dt;
    if (twist) midpoint = s.stamp;
    v["linear_speed_mps"] = number(velocity.stableNorm());
    threshold(f, "LOC_SPEED", velocity.stableNorm(),
              bounds(k["speed_mps"], false, {15, 50, false}), "m/s");
    // A map orientation correction rotates a world-expressed twist without
    // physical acceleration. Global outputs need an independent continuous
    // motion frame before vector derivatives may be interpreted as physics.
    if (continuous && have_velocity_ && midpoint > midpoint_) {
      double acceleration =
          ((velocity - velocity_) / (midpoint - midpoint_)).stableNorm();
      v["linear_acceleration_mps2"] = number(acceleration);
      threshold(f, "LOC_ACCELERATION", acceleration,
                bounds(k["acceleration_mps2"], false, {10, 30, false}),
                "m/s^2");
      if (continuous) {
        double residual =
            (s.position - (position_ + velocity_ * dt)).stableNorm();
        v["prediction_residual_m"] = number(residual);
        threshold(f, "LOC_JUMP", residual,
                  bounds(q["jump"]["residual_m"], false, {.5, 2, false}), "m");
      }
    }
    if (continuous) {
      double dot = std::min(
          1.0,
          std::abs(orientation_.normalized().dot(s.orientation.normalized())));
      double angle = 2 * std::acos(dot);
      v["angular_speed_rps"] = number(angle / dt);
      threshold(f, "LOC_ANGULAR_SPEED", angle / dt,
                bounds(k["angular_speed_rps"], false, {3, 10, false}), "rad/s");
      threshold(f, "LOC_ROTATION_JUMP", angle,
                bounds(q["jump"]["rotation_rad"], false, {.3, 1, false}),
                "rad");
    }
    velocity_ = velocity;
    midpoint_ = midpoint;
    have_velocity_ = true;
  } else
    have_velocity_ = false;
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
Findings LocalizationChecker::check(const Sample& s, Values& v) {
  v = s.values;
  Findings f = s.findings;
  auto motion = motion_.update(s, quality_, v);
  f.insert(f.end(), motion.begin(), motion.end());
  auto c = quality_["covariance"];
  if (get<bool>(c, "enabled", true)) {
    bool zero = s.covariance.isZero(0);
    auto policy = get<std::string>(c, "zero_matrix_policy", "unknown_warning");
    bool known = s.covariance_available && (!zero || policy == "valid");
    v["covariance_status"] = known ? "AVAILABLE" : "UNKNOWN";
    v["checks_available"] = known ? "pose,motion,covariance"
                                  : "pose,motion; covariance unavailable";
    v["required_checks_available"] =
        (!known && get<bool>(c, "required_for_valid", false)) ? "false"
                                                              : "true";
    if (!known) {
      add(f, "LOC_COVARIANCE_UNKNOWN", "covariance estimate unavailable",
          policy == "unknown_warning" ? WARNING : OK);
    } else {
      add(f, "LOC_COVARIANCE_UNKNOWN", "known covariance", OK);
      bool legal = CovarianceChecker::valid(
          s.covariance, positive(c, "numeric_tolerance", 1e-9),
          get<bool>(c, "require_symmetric", true),
          get<bool>(c, "require_psd", true));
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
std::unique_ptr<IHealthChecker> CheckerFactory::create(const ModuleConfig& c) {
  if (c.checker == "LidarChecker")
    return std::unique_ptr<IHealthChecker>(new LidarChecker(c.quality));
  if (c.checker == "ImuChecker")
    return std::unique_ptr<IHealthChecker>(new ImuChecker(c.quality));
  if (c.checker == "LocalizationChecker")
    return std::unique_ptr<IHealthChecker>(new LocalizationChecker(c.quality));
  if (c.checker == "RelayChecker" || c.checker == "PlanningChecker")
    return std::unique_ptr<IHealthChecker>(new RelayChecker());
  throw std::invalid_argument("unregistered checker");
}
LifecycleTracker::LifecycleTracker(const LifecycleConfig& c,
                                   const MonitorConfig& m, double now)
    : config_(c),
      strict_(m.strict_control),
      heartbeat_warning_(m.heartbeat_warning),
      heartbeat_error_(m.heartbeat_error),
      transition_(now),
      state_(m.strict_control ? "UNKNOWN" : "STARTING") {}
bool LifecycleTracker::control(const RuntimeUpdate& u, double now) {
  control_accepted_ = false;
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
void LifecycleTracker::sample(bool good, double) {
  if (good)
    ++good_samples_;
  else
    good_samples_ = 0;
}
LifecycleReport LifecycleTracker::evaluate(double now, bool ready, double age) {
  LifecycleReport r;
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
    if (state_ == "STOPPING" && now - transition_ > config_.stop) {
      state_ = "FAULT";
      add(r.findings, "STOP_FAILED", "stop confirmation timed out", ERROR,
          now - transition_, config_.stop, "s", true);
    }
    if (state_ == "STARTING") {
      if (ready && good_samples_ >= config_.ready_samples &&
          (!controlled_ || update_.observed == "RUNNING"))
        state_ = "RUNNING";
      else if (now - transition_ > config_.startup) {
        state_ = "FAULT";
        add(r.findings, "STARTUP_TIMEOUT",
            "required output not ready by deadline", ERROR, now - transition_,
            config_.startup, "s", true);
      }
    }
    if (state_ == "RUNNING" && age < 0) {
      state_ = "FAULT";
      good_samples_ = 0;
    }  // Negative sentinel: output exceeded configured hard receive age.
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
  r.expected_active = !(state_ == "STOPPED" || state_ == "PAUSED");
  r.valid_state = state_ == "RUNNING";
  return r;
}
ModuleMonitor::ModuleMonitor(const ModuleConfig& c, const MonitorConfig& m,
                             EventSink sink, double now)
    : config_(c),
      stream_(c.stream, m),
      lifecycle_(c.lifecycle, m, now),
      rules_(m.soft_hold, m.recovery_hold, sink),
      control_rules_(m.soft_hold, m.recovery_hold, sink),
      checker_(CheckerFactory::create(c)),
      sink_(std::move(sink)) {
  stream_.reset(now);
}
void ModuleMonitor::receive(const Sample& s) {
  if (config_.checker == "PlanningChecker" &&
      s.values.count("synthetic_status")) {
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
  rules_.apply(latest_sample_, s.received);
  lifecycle_.sample(s.valid && !anyError(latest_sample_), s.received);
  have_data_ = true;
}
bool ModuleMonitor::control(const RuntimeUpdate& u, double now) {
  bool reset = lifecycle_.control(u, now);
  if (!lifecycle_.controlAccepted()) return false;
  if (reset) {
    stream_.reset(now);
    checker_->reset();
    rules_.reset();
    control_rules_.reset();
    sample_rule_ids_.clear();
    latest_sample_.clear();
    values_.clear();
    have_data_ = false;
  }
  if (sink_ && (reset || u.command != last_command_)) {
    Finding f;
    f.id = "LIFECYCLE";
    f.reason = u.desired + " / " + u.observed + " / " + u.reason;
    sink_(reset ? "INSTANCE_RESET" : "CONTROL_INTENT", f, 0, 0);
    last_command_ = u.command;
  }
  return reset;
}
ModuleReport ModuleMonitor::evaluate(double now) {
  ModuleReport r;
  r.id = config_.id;
  r.checker = config_.checker;
  r.required = config_.required;
  r.topic = config_.topic;
  r.instance = lifecycle_.instance();
  r.command = lifecycle_.command();
  r.epoch = lifecycle_.epoch();
  r.values = values_;
  bool ready = false;
  auto stream = stream_.evaluate(now, r.values, ready);
  double age = stream_.received() ? now - stream_.lastReceive() : 0;
  bool planning = config_.checker == "PlanningChecker";
  if (planning) {
    ready = have_data_ && values_["task_receive_age_sec"] != "" &&
            std::stod(values_["task_receive_age_sec"]) <=
                config_.quality["task_state_timeout_sec"].as<double>();
  }
  auto lc = lifecycle_.evaluate(
      now, ready,
      !planning && stream_.received() && age > config_.stream.age.error ? -1
                                                                        : age);
  r.state = lc.state;
  r.expected_active = lc.expected_active;
  // A planned stop suspends only data-demand rules; hard invalid messages still
  // reach the sample gate. Startup checks hard samples immediately but wait for
  // the bounded startup deadline before demanding a fully populated stream.
  if (!planning && r.state != "STOPPED" && r.state != "PAUSED" &&
      r.state != "STOPPING" && r.state != "STARTING")
    rules_.apply(stream, now);
  else {
    Findings pass;
    for (const auto* id : {"RECEIVE_AGE", "FREQUENCY_LOW", "FREQUENCY_HIGH"})
      add(pass, id, "data demand suspended", OK);
    rules_.apply(pass, now);
  }
  control_rules_.apply(lc.findings, now);
  r.data_level = rules_.level();
  if (r.state == "STOPPED" || r.state == "PAUSED")
    r.data_level = rules_.periodLevel();
  r.level = std::max(r.data_level, control_rules_.level());
  if (!lc.valid_state && r.level < ERROR) r.level = WARNING;
  r.has_data = have_data_ && lc.expected_active && stream_.received() &&
               age <= config_.stream.age.error;
  if (planning) {
    r.has_data = ready && lc.expected_active;
    r.values["rate_hz"] = "N/A (path demand follows goal)";
    if (lc.valid_state && values_["task_state"] == "IDLE") r.state = "IDLE";
  }
  r.valid = lc.valid_state && r.has_data && r.level < ERROR &&
            r.values["required_checks_available"] != "false";
  if (r.level == ERROR)
    r.message = rules_.reasons() + "; " + control_rules_.reasons();
  else if (!lc.valid_state)
    r.message = r.state;
  else
    r.message = rules_.reasons();
  r.values["data_health_level"] = r.has_data ? levelName(r.data_level) : "N/A";
  r.values["operating_state"] = r.state;
  r.values["operational_valid"] = r.valid ? "true" : "false";
  r.values["has_data_health"] = r.has_data ? "true" : "false";
  r.values["expected_active"] = r.expected_active ? "true" : "false";
  r.values["sample_status"] = levelName(r.data_level);
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
