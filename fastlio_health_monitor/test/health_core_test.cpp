// 核心回归测试数据流：构造Sample/YAML -> 直接调用C++对象 -> 核对规则、生命周期、日志。
// 时间推进使用明确数值，不依赖真实机器人，也不改写原始bag。
#include <fastlio_health_monitor/event_recorder.h>
#include <fastlio_health_monitor/health_core.h>
#include <gtest/gtest.h>

#include <boost/filesystem.hpp>
#include <fstream>
using namespace robot_health;
namespace {
// 构造连续定位样本：时间、平面位置与有限协方差，用于差分及跳变边界。
Sample pose(double t, double x, double y = 0) {
  Sample s;
  s.stamp = t;
  s.received = t;
  s.position = Eigen::Vector3d(x, y, 0);
  s.frame = "odom";
  s.covariance = Eigen::Matrix<double, 6, 6>::Identity() * .01;
  return s;
}
// 提供一组明确启用的运动规则，保证测试不会依赖隐藏默认阈值。
YAML::Node motion() {
  return YAML::Load(
      "continuity: continuous_odometry\nkinematics: {dt_min_sec: 0.001, "
      "dt_max_sec: 1.0, velocity_source: position_difference, speed_mps: "
      "{warning_above: 15, error_above: 50}, acceleration_mps2: "
      "{warning_above: 10, error_above: 30}, angular_speed_rps: "
      "{warning_above: 3, error_above: 10}}\njump: {residual_m: "
      "{warning_above: 0.5, error_above: 2}, rotation_rad: {warning_above: "
      "0.3, error_above: 1}}\n");
}
// 按规则ID读取结果；-1表示该规则未执行，不能误当成检测正常。
int findLevel(const Findings& f, const std::string& id) {
  for (const auto& v : f)
    if (v.id == id) return v.level;
  return -1;
}
}  // namespace
// 验收：Bounds / EqualityAndDirections；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Bounds, EqualityAndDirections) {
  Bounds high{1, 2, false}, low{10, 5, true};
  EXPECT_EQ(high.level(1), OK);
  EXPECT_EQ(high.level(2), WARNING);
  EXPECT_EQ(high.level(2.1), ERROR);
  EXPECT_EQ(low.level(10), OK);
  EXPECT_EQ(low.level(5), WARNING);
  EXPECT_EQ(low.level(4), ERROR);
}
// 验收：Stream / PruneWithoutInput；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Stream, PruneWithoutInput) {
  MonitorConfig m;
  m.window = 2;
  m.frequency_coverage = .1;
  StreamConfig c;
  StreamTracker t(c, m);
  t.reset(0);
  for (int i = 0; i < 15; ++i) t.receive(1 + i * .1, i * .1);
  Values v;
  bool ready;
  t.evaluate(1.5, v, ready);
  EXPECT_TRUE(ready);
  t.evaluate(5, v, ready);
  EXPECT_FALSE(ready);
  EXPECT_EQ(v["receive_rate_hz"], "0");
  EXPECT_EQ(v["source_rate_hz"], "N/A");
}
// 验收：Stream / RelativeClockAndPlaybackRate；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Stream, RelativeClockAndPlaybackRate) {
  MonitorConfig m;
  m.frequency_coverage = .5;
  StreamConfig c;
  StreamTracker t(c, m);
  t.reset(1000);
  for (int i = 0; i < 20; ++i) t.receive(597 + i * .1, 1000 + i * .05);
  Values v;
  bool ready;
  t.evaluate(1001, v, ready);
  EXPECT_TRUE(ready);
  EXPECT_NEAR(std::stod(v["source_rate_hz"]), 10, 1e-6);
  EXPECT_NEAR(std::stod(v["receive_rate_hz"]), 20, 1e-6);
  EXPECT_NE(v["transport_delay_sec"].find("N/A"), std::string::npos);
}
// 验收：Stream / RegressionDoesNotPoisonNextStamp；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Stream, RegressionDoesNotPoisonNextStamp) {
  MonitorConfig m;
  StreamConfig c;
  StreamTracker t(c, m);
  t.reset(0);
  t.receive(600, 0);
  EXPECT_EQ(findLevel(t.receive(3, .1), "STAMP_REGRESSION"), ERROR);
  EXPECT_EQ(findLevel(t.receive(600.1, .2), "STAMP_GAP"), OK);
}
// 验收：Rules / BriefHardFailureSurvivesNormalSample；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Rules, BriefHardFailureSurvivesNormalSample) {
  int triggered = 0;
  RuleLatch gate(.5, .0,
                 [&](const std::string& type, const Finding&, int, int) {
                   if (type == "RULE_TRIGGERED") ++triggered;
                 });
  Finding f{"FINITE", "NaN", "a", "m/s^2", ERROR, NAN, 0, true};
  gate.apply({f}, 0);
  f.level = OK;
  f.hard = false;
  gate.apply({f}, .01);
  EXPECT_EQ(gate.level(), ERROR);
  gate.published();
  EXPECT_EQ(gate.level(), OK);
  EXPECT_EQ(triggered, 1);
}
// 验收：Rules / SoftDwellAndRecovery；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Rules, SoftDwellAndRecovery) {
  RuleLatch g(.5, 1, {});
  Finding f{"RATE", "low", "rate", "Hz", WARNING, 1, 5, false};
  g.apply({f}, 0);
  EXPECT_EQ(g.level(), OK);
  g.apply({f}, .6);
  EXPECT_EQ(g.level(), WARNING);
  g.published();
  f.level = OK;
  g.apply({f}, .7);
  g.apply({f}, 1.6);
  EXPECT_EQ(g.level(), WARNING);
  g.apply({f}, 1.8);
  g.published();
  EXPECT_EQ(g.level(), OK);
}
// 验收：Motion / FirstSamplesAreNAAndMidpointAcceleration；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Motion, FirstSamplesAreNAAndMidpointAcceleration) {
  KinematicsTracker k;
  auto q = motion();
  Values v;
  k.update(pose(1, 0), q, v);
  EXPECT_EQ(v["linear_speed_mps"], "N/A");
  k.update(pose(1.1, .1), q, v);
  EXPECT_NEAR(std::stod(v["linear_speed_mps"]), 1, 1e-8);
  EXPECT_EQ(v["linear_acceleration_mps2"], "N/A");
  k.update(pose(1.2, .3), q, v);
  EXPECT_NEAR(std::stod(v["linear_acceleration_mps2"]), 10, 1e-7);
}
// 验收：Motion / TurnUsesVectorAcceleration；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Motion, TurnUsesVectorAcceleration) {
  KinematicsTracker k;
  auto q = motion();
  Values v;
  k.update(pose(1, 0), q, v);
  k.update(pose(1.1, .1), q, v);
  k.update(pose(1.2, .1, .1), q, v);
  EXPECT_NEAR(std::stod(v["linear_acceleration_mps2"]), std::sqrt(2.) * 10,
              1e-7);
}
// 验收：Motion / QuaternionSignIsEquivalent；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Motion, QuaternionSignIsEquivalent) {
  KinematicsTracker k;
  auto q = motion();
  Values v;
  k.update(pose(1, 0), q, v);
  auto s = pose(1.1, 0);
  s.orientation.coeffs() *= -1;
  auto f = k.update(s, q, v);
  EXPECT_EQ(findLevel(f, "LOC_ROTATION_JUMP"), OK);
  EXPECT_NEAR(std::stod(v["angular_speed_rps"]), 0, 1e-8);
}
// 验收：Motion / GapResetAndFrameFailure；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Motion, GapResetAndFrameFailure) {
  KinematicsTracker k;
  auto q = motion();
  Values v;
  k.update(pose(1, 0), q, v);
  k.update(pose(5, 100), q, v);
  EXPECT_EQ(v["linear_speed_mps"], "N/A");
  auto s = pose(5.1, 100);
  s.frame = "map";
  EXPECT_EQ(findLevel(k.update(s, q, v), "LOC_FRAME_CHANGE"), ERROR);
  k.reset();
  k.update(s, q, v);
  EXPECT_EQ(v["linear_speed_mps"], "N/A");
}
// 验收：Motion / GlobalFrameUsesTwist；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Motion, GlobalFrameUsesTwist) {
  KinematicsTracker k;
  auto q = motion();
  q["continuity"] = "global_localization";
  q["kinematics"]["velocity_source"] = "twist";
  Values v;
  auto a = pose(1, 0);
  a.twist = Eigen::Vector3d(1, 0, 0);
  k.update(a, q, v);
  a = pose(1.1, 100);
  a.twist = Eigen::Vector3d(1, 0, 0);
  k.update(a, q, v);
  EXPECT_NEAR(std::stod(v["linear_speed_mps"]), 1, 1e-7);
  EXPECT_EQ(v["prediction_residual_m"], "N/A");
}
// 验收：Covariance / RejectAsymmetricNegativeAndNonPSD；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Covariance, RejectAsymmetricNegativeAndNonPSD) {
  Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
  EXPECT_TRUE(CovarianceChecker::valid(m, 1e-9, true, true));
  m(0, 1) = 2;
  EXPECT_FALSE(CovarianceChecker::valid(m, 1e-9, true, true));
  m(1, 0) = 2;
  EXPECT_FALSE(CovarianceChecker::valid(m, 1e-9, true, true));
  m.setIdentity();
  m(0, 0) = -1;
  EXPECT_FALSE(CovarianceChecker::valid(m, 1e-9, true, true));
  m(0, 0) = NAN;
  EXPECT_FALSE(CovarianceChecker::valid(m, 1e-9, true, true));
}
// 验收：Lidar / EmptyAndRatio；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Lidar, EmptyAndRatio) {
  auto q = YAML::Load(
      "points_total: {warning_below: 100, error_below: 1}\npoints_valid: "
      "{warning_below: 100, error_below: 1}\nvalid_ratio: {warning_below: 0.8, "
      "error_below: 0.5}");
  LidarChecker k(q);
  Sample s;
  Values v;
  EXPECT_EQ(findLevel(k.check(s, v), "LIDAR_EMPTY"), ERROR);
  s.total = 1000;
  s.valid_points = 700;
  EXPECT_EQ(findLevel(k.check(s, v), "LIDAR_RATIO"), WARNING);
  s.valid_points = 400;
  EXPECT_EQ(findLevel(k.check(s, v), "LIDAR_RATIO"), ERROR);
}
// 验收：Lifecycle / NormalStopThirtySecondsAndRestart；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Lifecycle, NormalStopThirtySecondsAndRestart) {
  MonitorConfig m;
  m.strict_control = true;
  LifecycleConfig c;
  c.ready_samples = 2;
  LifecycleTracker l(c, m, 0);
  RuntimeUpdate u{"ctrl", "A",  "cmd1", "RUN", "RUNNING", "", 1,
                  0,      true, {},     {},    {},        {}};
  EXPECT_TRUE(l.control(u, 0));
  l.sample(true, .1);
  l.sample(true, .2);
  EXPECT_EQ(l.evaluate(.2, true, .1).state, "RUNNING");
  u.desired = "STOP";
  u.observed = "STOPPING";
  u.command = "cmd2";
  u.sequence = 2;
  l.control(u, 1);
  EXPECT_EQ(l.evaluate(1, false, 0).state, "STOPPING");
  u.observed = "STOPPED";
  u.sequence = 3;
  l.control(u, 1.1);
  for (int i = 2; i < 33; ++i) {
    u.sequence++;
    l.control(u, i);
    auto r = l.evaluate(i, false, 0);
    EXPECT_EQ(r.state, "STOPPED");
    EXPECT_FALSE(r.valid_state);
  }
  u.instance = "B";
  u.desired = "RUN";
  u.observed = "RUNNING";
  u.sequence++;
  EXPECT_TRUE(l.control(u, 33));
  EXPECT_EQ(l.evaluate(33, false, 0).state, "STARTING");
  l.sample(true, 33.1);
  l.sample(true, 33.2);
  EXPECT_EQ(l.evaluate(33.2, true, .1).state, "RUNNING");
}
// 验收：Lifecycle / NoIntentHeartbeatLossAndTimeout；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Lifecycle, NoIntentHeartbeatLossAndTimeout) {
  MonitorConfig m;
  m.strict_control = true;
  LifecycleConfig c;
  c.startup = 1;
  LifecycleTracker l(c, m, 0);
  EXPECT_EQ(l.evaluate(.1, false, 0).state, "UNKNOWN");
  RuntimeUpdate u{"ctrl", "A",  "x", "RUN", "RUNNING", "", 1,
                  0,      true, {},  {},    {},        {}};
  l.control(u, 0);
  EXPECT_EQ(l.evaluate(1.5, false, 0).state, "FAULT");
  EXPECT_EQ(l.evaluate(4, false, 0).state, "UNKNOWN");
}
// 验收：Lifecycle / ReplayedHeartbeatCannotExtendStop；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Lifecycle, ReplayedHeartbeatCannotExtendStop) {
  MonitorConfig m;
  m.strict_control = true;
  LifecycleConfig c;
  LifecycleTracker l(c, m, 0);
  RuntimeUpdate u{"ctrl", "A",  "x", "STOP", "STOPPED", "", 1,
                  0,      true, {},  {},     {},        {}};
  l.control(u, 0);
  EXPECT_FALSE(l.control(u, 2.9));
  EXPECT_EQ(l.evaluate(3.1, false, 0).state, "UNKNOWN");
}
// 验收：Logging / JSONEscapingAndDurableFailure；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Logging, JSONEscapingAndDurableFailure) {
  EXPECT_EQ(jsonString("a\n\"b"), "\"a\\n\\\"b\"");
  MonitorConfig c;
  c.log_directory = "/proc/robot_health_no_permission";
  c.source_text = c.effective_text = "x";
  EventRecorder r(c);
  EXPECT_FALSE(r.healthy());
}
// 验收：Logging / DurableEventAndManifest；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Logging, DurableEventAndManifest) {
  MonitorConfig c;
  c.log_directory = "/tmp/robot_health_core_tests";
  c.source_text = c.effective_text = "schema_version: 2";
  std::string folder;
  {
    EventRecorder r(c);
    folder = r.directory();
    Finding f;
    f.id = "NAN";
    f.value = NAN;
    EXPECT_TRUE(r.event("RULE_TRIGGERED", "imu", "A", 2, f, 0, 2, {}, true));
    EXPECT_TRUE(r.healthy());
  }
  std::ifstream in(folder + "/events.0.jsonl");
  std::string all((std::istreambuf_iterator<char>(in)), {});
  EXPECT_NE(all.find("\"value\":null"), std::string::npos);
  EXPECT_NE(all.find("RUN_END"), std::string::npos);
  EXPECT_TRUE(boost::filesystem::exists(folder + "/manifest.json"));
}
// 验收：Config / RealConfigAndUnknownFieldRejected；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Config, RealConfigAndUnknownFieldRejected) {
  auto c = ConfigLoader::load(
      HEALTH_TEST_CONFIG_DIR "/health_robot.yaml");
  EXPECT_EQ(c.modules.size(), 5u);
  auto root = YAML::Load(c.source_text);
  root["monitor"]["typo"] = 1;
  std::ofstream out("/tmp/robot_health_bad_config.yaml");
  out << root;
  out.close();
  EXPECT_THROW(ConfigLoader::load("/tmp/robot_health_bad_config.yaml"),
               std::invalid_argument);
}
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
// 验收：Motion / SevereSingleFrameJumpIsImmediate；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Motion, SevereSingleFrameJumpIsImmediate) {
  KinematicsTracker k;
  auto q = motion();
  Values v;
  k.update(pose(1, 0), q, v);
  auto f = k.update(pose(1.1, 30), q, v);
  bool immediate = false;
  for (const auto& r : f)
    if (r.id == "LOC_SPEED") immediate = r.level == ERROR && r.hard;
  EXPECT_TRUE(immediate);
}
// 验收：Lifecycle / RetiredInstanceAndFrameEpochCannotReturn；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Lifecycle, RetiredInstanceAndFrameEpochCannotReturn) {
  MonitorConfig m;
  m.strict_control = true;
  LifecycleConfig c;
  LifecycleTracker l(c, m, 0);
  RuntimeUpdate u{"ctrl", "A",  "x", "RUN", "RUNNING", "", 1,
                  0,      true, {},  {},    {},        {}};
  l.control(u, 0);
  u.instance = "B";
  u.sequence = 2;
  l.control(u, 1);
  u.instance = "A";
  u.sequence = 3;
  EXPECT_FALSE(l.control(u, 2));
  EXPECT_FALSE(l.controlAccepted());
  EXPECT_EQ(l.instance(), "B");
  u.instance = "B";
  u.sequence = 4;
  u.frame_epoch = 2;
  l.control(u, 2);
  u.sequence = 5;
  u.frame_epoch = 1;
  EXPECT_FALSE(l.control(u, 2.1));
  EXPECT_FALSE(l.controlAccepted());
}
// 验收：Module / StartupHardErrorVisibleAndControlHealthSeparate；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Module, StartupHardErrorVisibleAndControlHealthSeparate) {
  auto config = ConfigLoader::load(
      HEALTH_TEST_CONFIG_DIR "/health_robot.yaml");
  config.strict_control = true;
  auto c = config.modules[1];
  ModuleMonitor module(c, config, {}, 0);
  auto r = module.evaluate(.1);
  EXPECT_EQ(r.level, ERROR);
  EXPECT_EQ(r.data_level, OK);
  EXPECT_EQ(r.state, "UNKNOWN");
  RuntimeUpdate u{"ctrl", "A",  "x", "RUN", "RUNNING", "", 1,
                  0,      true, {},  {},    {},        {}};
  module.control(u, .2);
  Sample s;
  s.stamp = 1;
  s.received = .3;
  s.valid = false;
  s.acceleration.x() = NAN;
  s.findings.push_back(Finding{"IMU_ACCEL_FINITE", "NaN", "accel", "m/s^2",
                               ERROR, NAN, 0, true});
  module.receive(s);
  r = module.evaluate(.31);
  EXPECT_EQ(r.state, "STARTING");
  EXPECT_EQ(r.level, ERROR);
  EXPECT_EQ(r.data_level, ERROR);
  EXPECT_FALSE(r.valid);
}
// 验收：Logging / QueueOverflowIsVisible；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Logging, QueueOverflowIsVisible) {
  MonitorConfig c;
  c.log_directory = "/tmp/robot_health_core_tests";
  c.source_text = c.effective_text = "queue test";
  c.event_capacity = 1;
  EventRecorder recorder(c);
  Finding f;
  f.id = "QUEUE";
  f.reason = "pressure";
  for (int i = 0; i < 10000; ++i)
    recorder.event("RULE_TRIGGERED", "test", "A", 1, f, 0, 2);
  EXPECT_GT(std::stoull(recorder.status()["dropped_events"]), 0u);
  EXPECT_FALSE(recorder.healthy());
}
// 验收：Logging / RotationRemovalIsDeclared；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Logging, RotationRemovalIsDeclared) {
  MonitorConfig c;
  c.log_directory = "/tmp/robot_health_core_tests";
  c.source_text = c.effective_text = "rotation test";
  c.rotate_bytes = 1024;
  c.rotate_count = 2;
  EventRecorder recorder(c);
  Finding f;
  f.id = "ROTATE";
  f.reason = std::string(300, 'x');
  for (int i = 0; i < 8; ++i)
    EXPECT_TRUE(
        recorder.event("STATE_CHANGED", "test", "A", 1, f, 0, 0, {}, true));
  EXPECT_GT(std::stoull(recorder.status()["rotated_files_removed"]), 0u);
  EXPECT_EQ(recorder.status()["logs_incomplete"], "true");
}
// 验收：Logging / WriteBudgetFailureIsReported；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Logging, WriteBudgetFailureIsReported) {
  MonitorConfig c;
  c.log_directory = "/tmp/robot_health_core_tests";
  c.source_text = c.effective_text = "budget test";
  c.budget_bytes = 2048;
  EventRecorder recorder(c);
  Finding f;
  f.reason = std::string(5000, 'x');
  EXPECT_FALSE(
      recorder.event("RULE_TRIGGERED", "test", "A", 1, f, 0, 2, {}, true));
  EXPECT_FALSE(recorder.healthy());
}
// 验收：Config / ThresholdUnitsAndDependencyValidation；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Config, ThresholdUnitsAndDependencyValidation) {
  auto base = ConfigLoader::load(
      HEALTH_TEST_CONFIG_DIR "/health_robot.yaml");
  auto reject = [&](const YAML::Node& root) {
    std::ofstream out("/tmp/robot_health_config_reject.yaml");
    out << root;
    out.close();
    EXPECT_THROW(ConfigLoader::load("/tmp/robot_health_config_reject.yaml"),
                 std::exception);
  };
  auto root = YAML::Load(base.source_text);
  root["modules"][1]["quality"]["acceleration_unit"] = "wrong";
  reject(root);
  root = YAML::Load(base.source_text);
  root["modules"][0]["stream"]["receive_age_sec"]["error_above"] = .1;
  reject(root);
  root = YAML::Load(base.source_text);
  root["modules"][0]["depends_on"].push_back("localization");
  reject(root);
  root = YAML::Load(base.source_text);
  root["monitor"]["publish_rate_hz"] = std::numeric_limits<double>::quiet_NaN();
  reject(root);
  root = YAML::Load(base.source_text);
  root["modules"][1]["quality"]["covariance"]["enabled"] = "wrong";
  reject(root);
  root = YAML::Load(base.source_text);
  root["modules"][2]["adapter"] = "NotAnAdapter";
  reject(root);
}
// 验收：Motion / UnauthorizedFrameCannotBecomeNewBaseline；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Motion, UnauthorizedFrameCannotBecomeNewBaseline) {
  KinematicsTracker tracker;
  Values values;
  auto q = motion();
  tracker.update(pose(1, 0), q, values);
  auto changed = pose(1.1, 0);
  changed.frame = "map";
  EXPECT_EQ(findLevel(tracker.update(changed, q, values), "LOC_FRAME_CHANGE"),
            ERROR);
  changed.stamp = 1.2;
  EXPECT_EQ(findLevel(tracker.update(changed, q, values), "LOC_FRAME_CHANGE"),
            ERROR);
  tracker.reset();
  EXPECT_NE(findLevel(tracker.update(changed, q, values), "LOC_FRAME_CHANGE"),
            ERROR);
}
// 验收：Module / PlanningStopSuspendsTaskAndPathDemand；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Module, PlanningStopSuspendsTaskAndPathDemand) {
  auto c =
      ConfigLoader::load(HEALTH_TEST_CONFIG_DIR "/../test/fixtures/health_extensions.yaml");
  c.strict_control = true;
  auto config = c.modules[1];
  ModuleMonitor module(config, c, {}, 0);
  RuntimeUpdate update{"ctrl", "A",  "stop", "STOP", "STOPPED", "normal", 1,
                       0,      true, {},     {},     {},        {}};
  module.control(update, 0);
  Sample synthetic;
  synthetic.stamp = 1;
  synthetic.received = 10;
  synthetic.valid = false;
  synthetic.values["synthetic_status"] = "true";
  synthetic.values["task_receive_age_sec"] = "10";
  synthetic.findings.push_back(Finding{"PLANNING_TASK_FRESH", "no task", "task",
                                       "s", ERROR, 10, 2, true});
  update.sequence = 2;
  module.control(update, 10);
  module.receive(synthetic);
  auto report = module.evaluate(10);
  EXPECT_EQ(report.state, "STOPPED");
  EXPECT_EQ(report.level, WARNING);
  EXPECT_FALSE(report.valid);
}
// 验收：Motion / GlobalOrientationCorrectionDoesNotFakeAcceleration；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Motion, GlobalOrientationCorrectionDoesNotFakeAcceleration) {
  KinematicsTracker tracker;
  Values values;
  auto q = motion();
  q["continuity"] = "global_localization";
  q["kinematics"]["velocity_source"] = "twist";
  auto s = pose(1, 0);
  s.twist = Eigen::Vector3d(20, 0, 0);
  tracker.update(s, q, values);
  s.stamp = 1.1;
  tracker.update(s, q, values);
  s.stamp = 1.2;
  s.orientation =
      Eigen::Quaterniond(Eigen::AngleAxisd(1.5, Eigen::Vector3d::UnitZ()));
  auto f = tracker.update(s, q, values);
  EXPECT_EQ(values["linear_acceleration_mps2"], "N/A");
  EXPECT_EQ(findLevel(f, "LOC_ACCELERATION"), -1);
}
// 验收：Module / ConfirmedStopDoesNotReportHistoricalDataErrorForever；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(Module, ConfirmedStopDoesNotReportHistoricalDataErrorForever) {
  auto c = ConfigLoader::load(HEALTH_TEST_CONFIG_DIR "/health_robot.yaml");
  c.strict_control = true;
  ModuleMonitor module(c.modules[1], c, {}, 0);
  RuntimeUpdate runtime{"ctrl", "A",  "run", "RUN", "RUNNING", "", 1,
                        0,      true, {},    {},    {},        {}};
  module.control(runtime, 0);
  Sample s;
  s.stamp = 1;
  s.received = .1;
  s.valid = false;
  s.findings.push_back(
      Finding{"IMU_ACCEL_FINITE", "bad", "a", "", ERROR, NAN, 0, true});
  module.receive(s);
  EXPECT_EQ(module.evaluate(.2).level, ERROR);
  runtime.sequence = 2;
  runtime.command = "stop";
  runtime.desired = "STOP";
  runtime.observed = "STOPPED";
  module.control(runtime, .3);
  auto r = module.evaluate(.4);
  EXPECT_EQ(r.state, "STOPPED");
  EXPECT_EQ(r.level, WARNING);
  EXPECT_FALSE(r.has_data);
  EXPECT_EQ(r.values["data_health_level"], "N/A");
}
