// 本文件验证 YAML 的“没写就不检查”，不使用真实传感器，不改用户的 bag。
// 数据流：临时 YAML -> ConfigLoader -> Checker/StreamTracker -> Findings -> 日志落盘。
#include <fastlio_health_monitor/event_recorder.h>
#include <fastlio_health_monitor/health_core.h>
#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <unistd.h>

using namespace robot_health;
namespace {
// 每次创建唯一文件，避免重复运行或并发测试互相覆盖配置。
MonitorConfig loadText(const std::string& text) {
  char path[] = "/tmp/health_optional_XXXXXX";
  const int fd = mkstemp(path);
  if (fd < 0) throw std::runtime_error("mkstemp failed");
  close(fd);
  std::ofstream out(path); out << text; out.close();
  try {
    auto c = ConfigLoader::load(path);
    unlink(path);
    return c;
  } catch (...) { unlink(path); throw; }
}
// 查找指定规则；-1 表示完全未执行，区别于“执行后为 OK”。
int ruleLevel(const Findings& f, const std::string& id) {
  for (const auto& r : f) if (r.id == id) return r.level;
  return -1;
}
// JSON 兼容 YAML，递归检查事件/上下文没有被要求删除的键。
void checkKeys(const YAML::Node& n) {
  if (n.IsMap()) for (const auto& item : n) {
    auto key = item.first.as<std::string>();
    for (const auto* forbidden : {"producer_instance_id", "module_epoch", "metric",
                                 "effective_config_sha256", "data_sha256", "epoch"})
      EXPECT_NE(key, forbidden);
    checkKeys(item.second);
  }
  else if (n.IsSequence()) for (const auto& child : n) checkKeys(child);
}
}

// 验收：OptionalConfig / MinimalImuWithoutRuntimeSectionsOrQuality；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(OptionalConfig, MinimalImuWithoutRuntimeSectionsOrQuality) {
  auto c = loadText("modules: [{id: imu, checker: ImuChecker, adapter: RosImuAdapter, topic: /imu}]");
  ASSERT_EQ(c.modules.size(), 1u);
  EXPECT_FALSE(c.modules[0].stream.age.enabled());
  EXPECT_FALSE(c.modules[0].stream.low_rate.enabled());
  EXPECT_FALSE(c.modules[0].stream.high_rate.enabled());
  EXPECT_FALSE(c.modules[0].stream.gap.enabled());
  EXPECT_EQ(c.modules[0].stream.duplicate_level, OK);
  EXPECT_FALSE(c.log_directory.empty());
  ImuChecker checker(c.modules[0].quality); Sample s; Values v;
  s.acceleration.z() = 10000; s.gyro.z() = 10000;
  auto f = checker.check(s, v);
  EXPECT_EQ(ruleLevel(f, "IMU_ACCEL_NORM"), -1);
  EXPECT_EQ(ruleLevel(f, "IMU_GYRO_NORM"), -1);
}
// 验收：OptionalConfig / EachSupportedCheckerAcceptsMissingQualityRules；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(OptionalConfig, EachSupportedCheckerAcceptsMissingQualityRules) {
  for (const auto& pair : {std::make_pair("LidarChecker", "PointCloud2Adapter"),
      std::make_pair("ImuChecker", "RosImuAdapter"),
      std::make_pair("LocalizationChecker", "OdometryAdapter"),
      std::make_pair("RangeChecker", "RosRangeAdapter"),
      std::make_pair("RelayChecker", "DiagnosticRelayAdapter")}) {
    SCOPED_TRACE(pair.first);
    auto c = loadText(std::string("modules: [{id: sensor, checker: ") + pair.first +
                      ", adapter: " + pair.second + ", topic: /sample}]");
    EXPECT_NO_THROW(CheckerFactory::create(c.modules[0]));
  }
}
// 验收：OptionalConfig / RuntimeSectionsCanBeOmittedIndividually；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(OptionalConfig, RuntimeSectionsCanBeOmittedIndividually) {
  auto original = YAML::LoadFile(HEALTH_TEST_CONFIG_DIR "/health_bag.yaml");
  for (const auto* name : {"monitor", "control", "stream_defaults", "lifecycle_defaults", "logging", "schema_version"}) {
    SCOPED_TRACE(name);
    auto root = YAML::Clone(original); root.remove(name);
    YAML::Emitter out; out << root;
    EXPECT_NO_THROW(loadText(out.c_str()));
  }
}
// 验收：OptionalConfig / QualityBlocksCanBeRemovedIndividually；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(OptionalConfig, QualityBlocksCanBeRemovedIndividually) {
  auto original = YAML::LoadFile(HEALTH_TEST_CONFIG_DIR "/health_bag.yaml");
  for (size_t i = 0; i < original["modules"].size(); ++i) {
    for (const auto& field : original["modules"][i]["quality"]) {
      auto name = field.first.as<std::string>();
      SCOPED_TRACE(std::to_string(i) + ":" + name);
      auto root = YAML::Clone(original); root["modules"][i]["quality"].remove(name);
      YAML::Emitter out; out << root;
      EXPECT_NO_THROW(loadText(out.c_str()));
    }
  }
}
// 验收：OptionalConfig / DisabledModuleDoesNotNeedAnInterface；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(OptionalConfig, DisabledModuleDoesNotNeedAnInterface) {
  auto c = loadText("modules: [{enabled: false}, {id: imu, checker: ImuChecker, adapter: RosImuAdapter, topic: /imu}]");
  EXPECT_EQ(c.modules.size(), 1u);
  EXPECT_THROW(loadText("modules: [{enabled: false}]"), std::exception);
}
// 验收：OptionalConfig / UnknownFieldsAndInvalidThresholdsStillRejected；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(OptionalConfig, UnknownFieldsAndInvalidThresholdsStillRejected) {
  EXPECT_THROW(loadText("modules: [{id: imu, checker: ImuChecker, adapter: RosImuAdapter, topic: /imu, typo: true}]"), std::exception);
  EXPECT_THROW(loadText("modules: [{id: imu, checker: ImuChecker, adapter: RosImuAdapter, topic: /imu, stream: {receive_age_sec: {warning_above: 2, error_above: 1}}}]"), std::exception);
}
// 验收：OptionalBounds / MissingDisabledAndIndependentLevels；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(OptionalBounds, MissingDisabledAndIndependentLevels) {
  EXPECT_FALSE(configuredBounds(YAML::Node()).enabled());
  auto off = configuredBounds(YAML::Load("{enabled: false, warning_above: 1, error_above: 2}"));
  EXPECT_FALSE(off.enabled()); EXPECT_EQ(off.level(100), OK);
  auto warning = configuredBounds(YAML::Load("{warning_above: 1}"));
  EXPECT_EQ(warning.level(1), OK); EXPECT_EQ(warning.level(100), WARNING);
  EXPECT_FALSE(warning.error_enabled);
  auto error = configuredBounds(YAML::Load("{error_below: 5}"), true);
  EXPECT_EQ(error.level(5), OK); EXPECT_EQ(error.level(4), ERROR);
  EXPECT_FALSE(error.warning_enabled);
}
// 验收：OptionalStream / NoHiddenAlarmsWhenEntireStreamIsOmitted；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(OptionalStream, NoHiddenAlarmsWhenEntireStreamIsOmitted) {
  auto c = loadText("modules: [{id: imu, checker: ImuChecker, adapter: RosImuAdapter, topic: /imu}]");
  StreamTracker tracker(c.modules[0].stream, c); tracker.reset(0);
  EXPECT_TRUE(tracker.receive(1, 0).empty());
  EXPECT_TRUE(tracker.receive(1, .01).empty());
  EXPECT_TRUE(tracker.receive(.5, .02).empty());
  Values v; bool ready;
  EXPECT_TRUE(tracker.evaluate(100, v, ready).empty());
}
// 验收：OptionalStream / ProbeWarnsAndErrorsWithoutWaitingForNextMessage；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(OptionalStream, ProbeWarnsAndErrorsWithoutWaitingForNextMessage) {
  auto c = loadText("modules: [{id: imu, checker: ImuChecker, adapter: RosImuAdapter, topic: /imu, stream: {receive_age_sec: {warning_above: 0.1, error_above: 0.2}}}]");
  StreamTracker tracker(c.modules[0].stream, c); tracker.reset(0); tracker.receive(1, 0);
  Values v; bool ready;
  EXPECT_EQ(ruleLevel(tracker.evaluate(.1001, v, ready), "RECEIVE_AGE"), WARNING);
  EXPECT_EQ(ruleLevel(tracker.evaluate(.2001, v, ready), "RECEIVE_AGE"), ERROR);
  // 下一帧时间戳到 2s 才能做 STAMP_GAP；接收年龄早已在 .1/.2s 告警。
  EXPECT_EQ(ruleLevel(tracker.receive(2, 1), "STAMP_GAP"), -1);
}
// 验收：OptionalStream / InheritanceCanBeExplicitlyCancelled；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(OptionalStream, InheritanceCanBeExplicitlyCancelled) {
  auto c = loadText("stream_defaults: {receive_age_sec: {warning_above: 0.1, error_above: 0.2}}\nmodules: [{id: imu, checker: ImuChecker, adapter: RosImuAdapter, topic: /imu, stream: {receive_age_sec: {enabled: false}}}]");
  EXPECT_FALSE(c.modules[0].stream.age.enabled());
  c = loadText("stream_defaults: {stamp: {max_gap_warning_sec: 0.1, max_gap_error_sec: 0.2}}\nmodules: [{id: imu, checker: ImuChecker, adapter: RosImuAdapter, topic: /imu, stream: {stamp: null}}]");
  EXPECT_FALSE(c.modules[0].stream.gap.enabled());
}
// 验收：OptionalAttitude / RollOnlyAndDisabledPitch；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(OptionalAttitude, RollOnlyAndDisabledPitch) {
  AttitudeTracker tracker(YAML::Load("{roll_abs_deg: {warning_above: 10, error_above: 20}, pitch_abs_deg: {enabled: false, warning_above: 1, error_above: 2}}"), "IMU");
  Sample s; s.orientation_available=true; s.stamp=1; s.frame="base_link";
  s.orientation=Eigen::Quaterniond(Eigen::AngleAxisd(.6, Eigen::Vector3d::UnitX()));
  Values v; auto f=tracker.update(s,v);
  EXPECT_EQ(ruleLevel(f,"IMU_ROLL_LIMIT"),ERROR);
  EXPECT_EQ(ruleLevel(f,"IMU_PITCH_LIMIT"),-1);
  EXPECT_EQ(ruleLevel(f,"IMU_ATTITUDE_NORM"),-1);
}
// 验收：OptionalRange / NoHiddenHeightOrContinuityThresholds；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(OptionalRange, NoHiddenHeightOrContinuityThresholds) {
  RangeChecker checker(YAML::Load("{}")); Sample s; Values v;
  s.range_min_m=0; s.range_max_m=100; s.range_m=1; s.stamp=1;
  checker.check(s,v); s.range_m=90; s.stamp=1.1;
  auto f=checker.check(s,v);
  EXPECT_EQ(ruleLevel(f,"RANGE_JUMP"),-1);
  EXPECT_EQ(ruleLevel(f,"RANGE_HEIGHT_HIGH"),-1);
  EXPECT_EQ(v["range_valid"],"true");
}
// 验收：OptionalModule / RangeNoDataThenReceiptThenDropout；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(OptionalModule, RangeNoDataThenReceiptThenDropout) {
  auto c=loadText("monitor: {min_frequency_samples: 2, min_frequency_coverage_sec: 0.01, soft_warning_hold_sec: 0, recovery_hold_sec: 0}\nmodules: [{id: range, checker: RangeChecker, adapter: RosRangeAdapter, topic: /range, required: false, missing_data_policy: no_data_until_first_sample, lifecycle: {ready_consecutive_samples: 1}, stream: {receive_age_sec: {warning_above: 0.1, error_above: 0.2}}}]");
  ModuleMonitor monitor(c.modules[0],c,{},0);
  auto r=monitor.evaluate(50); EXPECT_EQ(r.state,"NO_DATA"); EXPECT_FALSE(r.contributes_to_overall);
  Sample s; s.stamp=1; s.received=50; s.range_min_m=0; s.range_max_m=100; s.range_m=1;
  monitor.receive(s); s.stamp=1.02; s.received=50.02; monitor.receive(s);
  r=monitor.evaluate(50.03); EXPECT_TRUE(r.contributes_to_overall); EXPECT_EQ(r.state,"RUNNING");
  r=monitor.evaluate(50.23); EXPECT_EQ(r.level,ERROR); EXPECT_NE(r.state,"NO_DATA");
}
// 验收：LoggingV3 / RemovedFieldsAbsentInAllFilesAndNestedValues；构造边界输入，核对真实C++结果，失败保留断言位置。
TEST(LoggingV3, RemovedFieldsAbsentInAllFilesAndNestedValues) {
  MonitorConfig c; c.log_directory="/tmp/robot_health_log_v3_tests";
  c.source_text=c.effective_text="{}";
  EventRecorder logger(c); const auto dir=logger.directory();
  Finding f; f.id="RECEIVE_AGE"; f.metric="age_sec";
  Values v={{"producer_instance_id","A"},{"module_epoch","2"},{"metric","age"},
            {"effective_config_sha256","old"},{"data_sha256","old"},
            {"source_stamp_ns","1791962378491622000"},{"age_sec","0.2"}};
  ASSERT_TRUE(logger.event("RULE_TRIGGERED","imu","A",2,f,0,2,v,true));
  ModuleReport r; r.id="imu"; r.instance="A"; r.epoch=2; r.values=v;
  logger.metrics(r,steadyNow()); ASSERT_TRUE(logger.finish("test complete",v));
  std::ifstream events(dir+"/events.0.jsonl"); std::string line; unsigned count=0;
  while(std::getline(events,line)) { checkKeys(YAML::Load(line)); ++count; }
  EXPECT_EQ(count,3u);
  checkKeys(YAML::LoadFile(dir+"/manifest.json"));
  std::ifstream metrics(dir+"/metrics.0.csv"); std::getline(metrics,line);
  EXPECT_EQ(line,"bag_elapsed_sec,source_stamp_sec,steady_elapsed_sec,module_id,operating_state,health_level,operational_valid,values_json");
  EXPECT_TRUE(logger.status().count("data_sha256")==0);
  auto size=boost::filesystem::file_size(dir+"/events.0.jsonl");
  EXPECT_TRUE(logger.finish("again"));
  EXPECT_TRUE(logger.event("IGNORED","imu","A",2,f,0,0));
  EXPECT_EQ(size,boost::filesystem::file_size(dir+"/events.0.jsonl"));
}

// 验收：每个实际发布的V2配置都必须能被当前C++加载器解析。
TEST(OptionalConfig, AllProvidedProfilesLoad) {
  boost::filesystem::path directory(HEALTH_TEST_CONFIG_DIR);
  for (boost::filesystem::directory_iterator it(directory), end; it != end; ++it) {
    const auto name=it->path().filename().string();
    if ((name=="health_robot.yaml" || name=="health_bag.yaml") && it->path().extension()==".yaml") {
      SCOPED_TRACE(name);
      EXPECT_NO_THROW(ConfigLoader::load(it->path().string()));
    }
  }
}
// 验收：枚举规则也支持enabled:false；不能因为保留默认等级而重新启用。
TEST(OptionalConfig, EnumRulesCanBeDisabledExplicitly) {
  auto c=loadText("modules: [{id: imu, checker: ImuChecker, adapter: RosImuAdapter, topic: /imu, stream: {stamp: {duplicate_level: {enabled: false, level: WARNING}, regression_level: {enabled: false, level: ERROR}}}}]");
  EXPECT_EQ(c.modules[0].stream.duplicate_level,OK);
  EXPECT_EQ(c.modules[0].stream.regression_level,OK);
}

// 验收：协方差的对称/半正定子检查省略时关闭；显式true才执行。
TEST(OptionalCovariance, MissingSymmetryAndPsdDoNotBecomeHiddenChecks) {
  auto q=YAML::Load("{covariance: {enabled: true, zero_matrix_policy: valid}}");
  Sample s; s.orientation_available=true; s.orientation.w()=1; s.stamp=1;
  s.covariance.setIdentity(); s.covariance(0,1)=2; s.covariance(1,0)=2;
  Values v; LocalizationChecker omitted(q);
  EXPECT_EQ(ruleLevel(omitted.check(s,v),"LOC_COVARIANCE_INVALID"),OK);
  q["covariance"]["require_psd"]=true; LocalizationChecker psd(q);
  EXPECT_EQ(ruleLevel(psd.check(s,v),"LOC_COVARIANCE_INVALID"),ERROR);
  q["covariance"]["require_psd"]=false; q["covariance"]["require_symmetric"]=true;
  s.covariance(1,0)=0; LocalizationChecker symmetry(q);
  EXPECT_EQ(ruleLevel(symmetry.check(s,v),"LOC_COVARIANCE_INVALID"),ERROR);
}
