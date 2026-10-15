# IMU 时间探针与接收超时：错误 bag 实测及完整视频

测试日期：2026-10-14（Asia/Shanghai）。范围：现有C++ V2监控、真实Livox输入、单调时钟断流检查与RViz。没有运行FAST-LIO建图，不使用合成Odometry。没有修改生产C++检测逻辑或用户当前YAML；新增的Python文件仅用于bag生成、测试观察和视频录制。

## 1. 视频和结论

桌面完整错误bag测试视频（本机文件，不随仓库发布）。1920×1080、H.264、约20fps，约130秒，无配音。保留整个实时录屏，各组bag按1倍速完整回放，末尾追加由实测数据生成的静态汇总页。测试段未删除或加速。

**当前100Hz、消抖0秒、IMU接收超时0.1/0.2秒的配置：下一条IMU到来前，约101.55ms触发WARNING、202.84ms触发ERROR。** 5Hz或100Hz保留0.5秒消抖时，断流期间均没有正式触发WARNING。ERROR可能先由生命周期RUNTIME_FAULT触发，不能把它错误描述成总要等待0.5秒才触发。

100/200ms是配置阈值，普通Linux/ROS回调存在调度开销。本次验收对WARN/ERROR的观察截止为150/250ms，不是恰好100/200ms的硬实时保证。短断流WARNING约135.66ms、100Hz诊断最大观察间隔约48.81ms，也表明检查频率不等于每次都准时执行。

## 2. 错误 bag 如何产生

原始bag：`$HOME/datasets/fastlio/outdoor_Mainbuilding_10hz_2020-12-24-16-38-00.bag`。

- 取前18秒输入，删除第6～7秒的IMU，形成约1秒空窗；删除第12～12.15秒的IMU，形成短时空窗。
- 保留雷达、IMU消息载荷和原始 `header.stamp`；不覆盖原始bag。
- **原记录时序副本**：imu_silence_1s_150ms.bag（本机文件，不随仓库发布）。保留原bag记录时间，删除184条和20条IMU。
- **源时间排程副本**：imu_source_scheduled_silence_1s_150ms.bag（本机文件，不随仓库发布）。为了单独测量探针，IMU的bag记录时间改为 `原bag起点 + 当前源时间戳 - 第一条源时间戳`，按新记录时间排序；雷达记录时间不变。删除203条和30条IMU。此副本不是原始到达时序的无改动回放，重排策略已写入manifest。

原bag在非故障2～5秒范围内：IMU源时间间隔中位数约5.00ms，但bag记录间隔中位数约0.027ms、P99约101.27ms、最大约105.13ms，即约100ms一批。统计见[bag_schedule_statistics.json](evidence/v2/imu_watchdog_20261014/bag_schedule_statistics.json)。把源频率200Hz直接当成电脑每5ms收到一条会产生错误预期。

原始bag修改前后SHA-256一致：`e2a96b36a494360d61d1f066b6ac0a385aca346e614cd54b7fc5706a8e3d62e4`。副本哈希、精确删除区间、数量和源时间边界分别见[原时序manifest](evidence/v2/imu_watchdog_20261014/imu_silence_1s_150ms.manifest.json)、[源时间排程manifest](evidence/v2/imu_watchdog_20261014/imu_source_scheduled_silence_1s_150ms.manifest.json)。

## 3. 配置与实测结果

四组的IMU接收阈值共同为WARNING >0.1秒、ERROR >0.2秒；A/B/C使用同一个源时间排程副本，D使用原记录时序副本。两类副本不能混称相同到达时序。

| 组 | 检查/发布频率 | 额外消抖 | 首次WARNING | 首次ERROR |
|---|---|---|---|---|
| A | 5 Hz | 0.5 s | 未触发 | 369.02 ms |
| B | 100 Hz | 0.5 s | 未触发 | 234.24 ms |
| C | 100 Hz | 0 s | 101.55 ms | 202.84 ms |
| D | 100 Hz | 0 s | 126.82 ms | 206.86 ms |

表中均以1秒断流期间最后一条IMU的观察接收时间为零点，且只统计下一条IMU到来前的诊断；没有使用bag结束后的通用ERROR来判通过。

- C实际长空窗约1007.87ms；短空窗约155.14ms。短空窗约135.66ms报WARNING，下一帧前没有接收超时ERROR。
- D实际短空窗约201.21ms，并非精确150ms，这是原记录时间批量发送带来的差别。非注入的2～5秒内有101条诊断样本处于含RECEIVE_AGE的WARNING；**这不是101次独立断流事件**。
- 因此当前配置已证明“无需等下一条IMU就能告警”，但不能据此宣布100ms阈值适用于原始批量回放或所有设备。需要基于实际到达抖动确定阈值，或在接入层明确源时序/到达时序契约。
- 恢复帧可能另外触发STAMP_GAP；这个事件是收到下一帧后的源时间检查，不算断流空窗期间的告警。恢复仍采用1秒保持规则。

用户当前配置源文件只读取。每组另存YAML快照，过滤掉与此输入测试无关的定位模块；IMU为required，LiDAR作为可视化及独立输入对照，不参与本次总体必需项汇总。单位和其他质量规则沿用配置。managed配置、桌面独立包和ZIP没有同步改成此次测试参数。

## 4. 测量方法与验收范围

- 测试观察进程订阅实际IMU和DiagnosticArray，记录 `time.monotonic()`；诊断中同时保留C++计算的age_sec、状态、级别、原因和源时间。
- 通过manifest中的源时间边界识别A/B/C空窗；D按原记录时间窗口识别空窗，并选择对应窗口中的实际最大接收间隔。源时间戳与接收单调时钟不相减。
- 告警延迟 = 收到告警诊断的单调时钟时间 − 最后一条IMU的观察接收时间，包含传输及观察开销，不是内核网络到达时间或绝对传输延迟。
- 仿真时钟在rospy初始化前启用，观察时钟与诊断的ROS时间域一致。ROS master使用隔离端口33514。
- C在首个故障前确认RUNNING/OK；检查两级诊断都在下一帧前出现、WARN≤150ms、ERROR≤250ms；短空窗有WARN且无接收超时ERROR；验证原始bag、源YAML和错误副本哈希。
- 主测试11项检查全部通过。A/B是预期不满足WARNING时效的对照，不伪装成全部四组均满足100/200ms要求。
- 本次没有做完整建图、高CPU负载或独立回调队列的压力验收；当前定时器仍与输入回调共用队列。

完整结果：[results.json](evidence/v2/imu_watchdog_20261014/results.json)。各组 `*_observations.json` 包含完整IMU接收序列和诊断序列；各组 `*_node.log` / `*_player.log` 保存启动输出。`runtime_logs/<run-id>/` 保存有效配置、版本/二进制哈希、JSONL事件和CSV指标。首次成功测量另保留在 `repeat_1_measurements`，初始观察器时钟校准记录不是最终验收结果。

## 5. 关键截图

### WARNING：下一条IMU之前

当前配置WARNING（本机测试归档，未纳入仓库）

### ERROR：下一条IMU之前

当前配置ERROR（本机测试归档，未纳入仓库）

### 实测汇总（根据结果生成，不是额外运行的故障画面）

实测汇总（本机测试归档，未纳入仓库）

## 6. 复现命令

下面用已生成副本重新跑完整四组测试，输出选择新目录和新视频文件名，以免覆盖已有证据；需要当前图形桌面和可用隔离端口33514。脚本自动启动和结束自己创建的ROS进程。

```bash
source /opt/ros/noetic/setup.bash
source $HOME/fastlio_ws/devel/setup.bash
python3 $HOME/fastlio_ws/src/fastlio_health_monitor/test/run_imu_watchdog_video.py \
  --bag $HOME/datasets/fastlio/health_fault_cases/imu_watchdog_20261014/imu_source_scheduled_silence_1s_150ms.bag \
  --original-record-bag $HOME/datasets/fastlio/health_fault_cases/imu_watchdog_20261014/imu_silence_1s_150ms.bag \
  --evidence /tmp/imu_watchdog_retest_20261014 \
  --video $HOME/Desktop/IMU时间探针_重测_2026-10-14.mp4
```

重新生成源时间排程副本（选择新的输出路径）：

```bash
python3 $HOME/fastlio_ws/src/fastlio_health_monitor/test/generate_imu_watchdog_bag.py \
  --source $HOME/datasets/fastlio/outdoor_Mainbuilding_10hz_2020-12-24-16-38-00.bag \
  --output /tmp/imu_probe_source_schedule_retest.bag \
  --schedule-source-time
```

不加 `--schedule-source-time` 生成保留原记录时序的错误副本。生成器拒绝覆盖源文件或已有输出文件。此处是简单测试脚本，实际检测仍由原C++ V2节点执行。

## 7. 视频及构建归档

本次 `catkin_make --pkg fastlio_health_monitor -j2` 成功，见build.log（本机测试归档，未纳入仓库）。新增测试脚本，不修改生产C++或用户YAML。视频完整解码结果、时长、分辨率、帧数及SHA-256见[video_metadata.json](evidence/v2/imu_watchdog_20261014/video_metadata.json)。`complete_live_recording.mp4`保留追加静态结果页前的整段实时录屏；桌面文件为最终成片。

## 2026-10-14：用户一键完整启动后的日志解读

本节读取用户实际使用完整launch启动的最新日志，不引用四组对比实验作为本次运行结果。

运行目录：`$HOME/fastlio_ws/logs/robot_health/3fa05-18de599809d53429-822fe314/`。manifest记录输入为源时间排程的错误副本；config_source.yaml确认100Hz、消抖0、IMU接收阈值0.1/0.2秒。

| events.0.jsonl行号 | 启动后时间s | 事件 | value含义 | 结果 |
|---|---:|---|---|---|
| 12 | 7.610137 | RECEIVE_AGE / RULE_TRIGGERED | 0.106153633秒没有新IMU | OK→WARNING，阈值0.1秒 |
| 14 | 7.710132 | RECEIVE_AGE / RULE_TRIGGERED | 0.206126990秒没有新IMU | WARNING→ERROR，阈值0.2秒 |
| 26 | 8.511892 | STAMP_GAP / RULE_TRIGGERED | 新旧源时间差1.00800219秒 | 新IMU到来后发现源间隔异常 |
| 33 | 9.520050 | RECEIVE_AGE / RULE_RECOVERED | 有通过证据并保持恢复时间 | ERROR→OK |
| 40 | 13.610107 | RECEIVE_AGE / RULE_TRIGGERED | 0.105233348秒没有新IMU | 短空窗WARNING |
| 42 | 13.660016 | STAMP_GAP / RULE_TRIGGERED | 新旧源时间差0.15498583秒 | 短空窗后的源间隔ERROR |

第12和14行的source_stamp_sec均为603.50278095；到第26行才变为604.51078314。这与事件时间先后共同表明：前两条接收超时告警没有等待下一条IMU，下一条到来后才进行STAMP_GAP判断。value是C++侧计算的接收年龄，不等同于此前视频测试观察器测得的诊断传输延迟。

第50/52行的RECEIVE_AGE告警发生在bag结束后，应与注入断流窗口分开看。CSV默认1秒采样，不保证捕捉100毫秒量级的短WARNING；关键触发/恢复应查看JSONL事件。wall_time_utc用于日期关联，细时间顺序查看steady_elapsed_sec。

另外发现该run的effective_config.yaml中monitor字段错误地包含control字段，未保留publish_rate_hz等，属于配置快照导出异常；此处参数依据config_source.yaml和实际事件阈值核对，生产代码尚未在本轮修复。问题补充到15评审的R11。

## 2026-10-14后续：回放生命周期优化

当前V2一键/managed入口已同步空格暂停、恢复及正常EOF，分别显示PAUSED和STOPPED，异常退出/断流仍ERROR。见[17_优化与验证](17_回放暂停恢复与正常结束优化.md)。本页早先视频、直接rosbag回放和EOF故障截图保留为当时行为的历史证据。
