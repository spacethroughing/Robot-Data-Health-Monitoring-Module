# 19 自采 Bag 副本修复与原生回放测试

日期：2026-10-15（Asia/Shanghai）。用户在只读诊断后授权建立副本、修复副本并测试，桌面原文件继续保留。

## 1. 文件位置与修复范围

| 文件 | 路径 |
| --- | --- |
| 原文件，不修改 | `$HOME/Desktop/2026-10-14-15-19-35.bag.active` |
| 修复后的副本 | `$HOME/Desktop/2026-10-14-15-19-35_修复副本/2026-10-14-15-19-35.recovered.bag` |

使用 Noetic 自带工具：

```bash
source /opt/ros/noetic/setup.bash
rosbag reindex --quiet \
  --output-dir $HOME/Desktop/2026-10-14-15-19-35_修复副本 \
  $HOME/Desktop/2026-10-14-15-19-35.bag.active
```

本机 Noetic 的 `--output-dir` 实现会先复制输入文件到指定的不同目录，然后只对输出副本追加/重建索引。没有在原文件所在位置执行原地 reindex，也没有为已存在的桌面原包创建重复的 `.orig` 备份，避免额外占用约 5.9 GB 空间。

修复完成并确认索引可读后，仅将副本扩展名改为 `.recovered.bag`。改名本身不是修复措施。没有使用 `rosbag fix` 修改消息定义，没有填补尾块，没有修改任何完整消息的时间戳或数据。

修复结果：

- `indexed: True`，ROS bag 2.0，无压缩。
- 时长 **286.366622 秒**，可读消息 **621,134 条**，与修复前只读扫描一致。
- 修复副本大小 5,910,132,911 字节。重建索引和末尾连接元数据会增加大小，不能仅凭大小判断是否增删了有效消息。
- 原包末尾 18,581 字节属于未完成的最后一个 chunk（含其头部），无法完整读取，因此未保留在副本中。
- 原文件与副本在偏移 `[4109, 5908981611)` 的完整 chunk 区域 SHA-256 完全一致，证明完整数据块及其中原有消息字节未改变。原雷达首帧的零时间戳也原样保留，没有为了健康结果而修正。

证据：[recovered_info.yaml](evidence/v2/desktop_active_bag_20261015/recovered_info.yaml)、copy_reindex.log（本机测试归档，未纳入仓库）、[recovery_integrity.json](evidence/v2/desktop_active_bag_20261015/recovery_integrity.json)。

## 2. `rosbag check` 的含义

副本已经消除 `bag unindexed`，但 `rosbag check` 尚不能称为“全部通过”：本机缺少 `battery_state/BatteryState`、`rofly_airground_ctrl/MotionCtrlDebugAirgroundSeparate`、`quadrotor_msgs/Polynome` 的当前消息类型/迁移规则，也输出了 Odometry 的迁移提示和本机 rosdep 信息。

这些是当前环境的消息定义与迁移检查结果，不能等同于索引损坏。本次不擅自迁移或改写它们。健康检测选择标准 `sensor_msgs/PointCloud2`、`sensor_msgs/Imu` 和 `nav_msgs/Odometry` 话题，以实际订阅/接收验证兼容性。自定义业务节点尚未完成安装和接入验证。

原始输出：recovered_check.log（本机测试归档，未纳入仓库）。

## 3. 原生回放测试方法

使用 `rosbag play` 原生 C++ 播放器，读取修复后的副本，不再使用只读脚本逐条转发消息。

- 独立 ROS master：`http://127.0.0.1:33520`；原先 11311 的工程不停止。
- 2 倍速完整回放所有可读时间范围，源时间约 286.37 秒；频率检测使用消息头源时间，与倍速区分。此轮用于适配、接收计数、消息质量与 EOF 检查，不是 100/200 ms 报警时延标定。
- 播放话题：`/rslidar_points`、`/mavros/imu/data_raw`、`/Odometry`、`/cloud_registered`。
- C++ V2 检测模块与 RViz 检测台运行；不重新运行 FAST-LIO 建图。画面为包内已有定位和已配准点云。
- 独立测试 YAML 使用 PointCloud2 雷达适配器、SI IMU、`camera_init/body`；保留用户当前主要阈值（IMU 接收年龄 0.1/0.2 秒、100 Hz 检查）。测试快照的 `stamp_domain` 标签改为 `ros_time`；生产 YAML 不修改。
- 首先确认三个模块已接入本次控制实例，再确认原生播放器的 TCPROS 连接，最后经原生暂停服务确认后开始发包，避免首次控制接入时的计数清零与总发送量混在一起。
- 测试控制心跳用于说明本轮运行意图。只有原生进程退出码为 0 且 `/clock` 到达文件末尾后，测试控制器才发布 `STOPPED` 并保持心跳；这不是将旧原文件的截断末尾伪装为正常 EOF。
- 截图不会暂停原生 C++ 播放器；原来的只读转发工具产生的重复遍历结果不用于本轮结论。

工具：test_recovered_native.py（本机测试归档，未纳入仓库）。配置与证据：[native_recovered](evidence/v2/desktop_active_bag_20261015/native_recovered/)。

## 4. 原生回放实测结果

**已完成，12 项自动核对全部通过。** 这 12 项验证接入、计数、部分正常运行状态、EOF 终态、日志与截图，不代表所有健康指标在整段运行中始终 OK。

| 输入 | 副本中可读数量 | C++ 检测器实际接收 | 核对 |
| --- | ---: | ---: | --- |
| `/rslidar_points` | 2,421 | 2,421 | 通过 |
| `/mavros/imu/data_raw` | 56,488 | 56,488 | 通过 |
| `/Odometry` | 2,223 | 2,223 | 通过 |
| `/cloud_registered` | 2,223 | 不经过检测器 | 用于 RViz |

运行中三项曾同时 `OK / RUNNING`；原生播放器退出码为 0。最后时钟 `1791962664.8505647`，文件末尾时间 `1791962664.8582442`，差约 7.68 ms，满足本次 50 ms EOF 核对容差。终态三个模块均为 `STOPPED`，持续约 3 秒保持控制心跳，日志模块保持 OK。回放、EOF 后保持及截图共约 146.96 秒墙钟时间。

### 原始数据中仍保留并触发的异常

- 雷达首帧零时间戳：`STAMP_INVALID / ERROR`，随后恢复；没有修改首帧来消除它。
- IMU 三次源间隔告警：约 21.04、23.44、22.40 ms，大于当前 20 ms 阈值，均为 WARNING。
- 定位加速度一次 WARNING：约 12.543 m/s²，大于当前 10 m/s² 阈值。
- 开头雷达和定位未开始发布，晚于模板 10 秒启动期限，因此有启动超时。这是记录中的启动顺序与当前启动期限不匹配，不能直接当成持续运行中的传感器失联。

### 本轮发现的 EOF 过渡边界

原生进程最终退出得到确认以前，输入已经停止，日志仍短暂记录了末尾接收超时：例如 events 第 52/54 行，IMU 接收年龄约 105/205 ms；随后 EOF 控制状态到达，当前界面恢复为 `STOPPED`。

因此本轮证明了 **EOF 最终状态正确**，没有证明“从最后一帧到 STOPPED 全过程零瞬时告警”。这些末尾事件不作为包内运行中丢数的证据，也不能用终态截图掩盖它们。进一步缩短/处理 EOF 确认窗口属于后续优化点；本轮没有修改生产 C++ 播放控制实现。

前一次只读工具中的 IMU 46 条计数差异，在本轮先确认控制实例后已消除，56,488/56,488 接收核对通过。之前不通过的历史结果仍保留，不改写为通过。

证据：

- [results.json](evidence/v2/desktop_active_bag_20261015/native_recovered/results.json)：12 项检查、数量、时钟与日志目录。
- diagnostics.json（本机测试归档，未纳入仓库）：实际诊断快照与状态变化。
- events.0.jsonl（本机测试归档，未纳入仓库）：逐规则事件。第 37 行为雷达时间戳，第 44 行为定位加速度；第 52 行以后需与 EOF 过渡一起解释。
- metrics.0.csv（本机测试归档，未纳入仓库）：每秒指标与当前代际计数；不是全量传感器记录。

### 实际截图

原生回放中的雷达和定位检测、已配准点云（本机测试归档，未纳入仓库）

原生回放后段实时数据（本机测试归档，未纳入仓库）

EOF确认后STOPPED与保留的点云画面（本机测试归档，未纳入仓库）

## 5. 原文件保全

原文件测试前 SHA-256：

```text
ed0e8c1eb5312a43713df81a8f6c5ea0faf7ba581d1068298a96550792baec5f
```

测试结束后再次核对原文件内容 SHA-256、大小和修改时间。副本的正常 EOF 只证明修复后可读消息已播完，不意味着恢复了最初未写完整或从未落盘的数据。

三项核对均一致，`unchanged=true`，见 [original_after.json](evidence/v2/desktop_active_bag_20261015/native_recovered/original_after.json)。测试创建的 ROS master、监控、RViz 和播放器均已关闭，原先 11311 的工程未停止。

## 6. 单独检查/回放副本

检查结构与话题：

```bash
source /opt/ros/noetic/setup.bash
rosbag info $HOME/Desktop/2026-10-14-15-19-35_修复副本/2026-10-14-15-19-35.recovered.bag
```

如果仅验证原生回放，可在独立 master 中运行，避免与已有工程的话题混合：

终端 1：

```bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:33521
roscore -p 33521
```

终端 2：

```bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:33521
rosparam set /use_sim_time true
rosbag play --clock $HOME/Desktop/2026-10-14-15-19-35_修复副本/2026-10-14-15-19-35.recovered.bag
```

这两组命令只播放数据，不启动健康检测台，也不提供健康模块的暂停/停止控制心跳。此包是 RSLidar/MAVROS 话题，不能直接把路径填进原 Avia 建图入口就认为完成传感器与外参适配；本轮标准监控话题适配和已有定位结果检测已验证，重新建图尚未验证。

## 7. 完整启动健康检测台的自动验证命令

**后续日常查看使用第 8 节的 roslaunch 入口。** 本节保留之前自动验证的复现方式，运行完会自动关闭。

以下复用本次已实测的自动验证入口，同时为每次运行创建独立日志目录，避免覆盖旧截图和结果文件。只复制几份小配置/测试工具，不复制 bag，也不修改修复副本或桌面原包。

整段复制到一个终端执行，不需要另外启动 roscore、RViz 或播放器：

```bash
bash -c '
set -e
source /opt/ros/noetic/setup.bash
source $HOME/fastlio_ws/devel/setup.bash

check_source=$HOME/fastlio_ws/src/FAST_LIO/doc/ros1_workspace/evidence/v2/desktop_active_bag_20261015
check_run=$(mktemp -d $HOME/fastlio_ws/logs/recovered_bag_check_XXXXXX)

cp "$check_source"/{test_recovered_native.py,test_config.yaml,active_bag_readonly_test.rviz,original_before.json} "$check_run/"
echo "本次结果与日志目录：$check_run/native_recovered"
exec /usr/bin/python3 -u "$check_run/test_recovered_native.py"
'
```

此入口固定使用修复副本、独立 master 33520、原生播放器 2 倍速。运行现有 C++ V2 健康监控和 RViz，显示包内已配准点云、已有定位和三路检测指标。自动起播、核对数量，最终显示 STOPPED 约 3 秒，然后关闭本次测试窗口/节点。同一时间只启动一份此验证程序。

**这是自动验证入口，暂不支持空格暂停的状态同步，也不是常驻交互启动器。** 不要把它与已适配 Avia 的交互回放入口混为一谈。若只是观察测试，等待它完成即可；需要提前结束时在该终端按 Ctrl+C。

### 配置文件是哪个

**修改检测参数的源文件：**

```text
$HOME/fastlio_ws/src/FAST_LIO/doc/ros1_workspace/evidence/v2/desktop_active_bag_20261015/test_config.yaml
```

命令会复制这份 YAML 到本次日志目录。脚本读取后只覆盖数据来源路径、日志目录和 `stamp_domain=ros_time`，再生成本次 C++ 节点实际加载的配置：

```text
$HOME/fastlio_ws/logs/recovered_bag_check_XXXXXX/native_recovered/test_config.yaml
```

`XXXXXX` 是每次自动生成的不同后缀。要改变下一次检测阈值，修改源文件再重新运行；运行中修改 YAML 不会热加载。旧的 `evidence/.../native_recovered/test_config.yaml` 是上次测试的生成快照，不是上述新命令的参数源文件。

| 配置项 | 当前值 |
| --- | --- |
| 雷达 | `PointCloud2Adapter`，`/rslidar_points` |
| IMU | `RosImuAdapter`，`/mavros/imu/data_raw`，加速度单位 `mps2` |
| 定位 | `OdometryAdapter`，`/Odometry`，`camera_init/body` |
| 检查频率 | `monitor.publish_rate_hz: 100.0` |
| IMU接收年龄 | `warning_above: 0.1`、`error_above: 0.2` 秒 |

本次逐规则日志在终端打印的结果目录下 `health_logs/<run_id>/events.0.jsonl`，指标在同目录 `metrics.0.csv`；`results.json` 保存数量核对。开头雷达/定位晚于 IMU 出现，会有启动缺数；零时间戳、源间隔和运动告警仍按原配置判断。EOF 过渡短暂超时的限制见第 4 节。

## 8. 日常使用：roslaunch常驻检测台（2026-10-15新增）

**后续日志行为更新：** 当前入口已默认启用正常EOF后封存健康和回放日志，同时新增bag时间/原消息头时间字段，详见[20_日志时间对齐与回放结束封存](20_日志时间对齐与回放结束封存.md)。下文原有运行日志是修改前历史证据，不包含这些新字段。

### 完整命令

整段复制到一个终端即可。使用独立 33522 master，避免与仍在运行的原 Avia 工程混合话题或覆盖同名节点。

```bash
bash -c '
source /opt/ros/noetic/setup.bash &&
source $HOME/fastlio_ws/devel/setup.bash &&
export ROS_MASTER_URI=http://127.0.0.1:33522 &&
exec roslaunch -p 33522 fastlio_health_monitor recorded_bag_health.launch
'
```

默认回放桌面修复副本，1倍速，自动启动C++健康监控、RViz和C++回放控制器。不要同时启动多份此入口。回放完成后界面常驻，Ctrl+C关闭本次launch；这与第7节自动验证程序的自动退出不同。

### 现在修改哪份YAML

```text
$HOME/fastlio_ws/src/fastlio_health_monitor/config/health_v2_rslidar_mavros.yaml
```

这份是新 roslaunch 的直接加载配置，已补充中文注释；修改后重启launch加载。IMU单位为 `mps2`、接收年龄0.1/0.2秒、100Hz检查及质量阈值继承创建本入口时的测试源文件。后续修改旧证据目录的 `test_config.yaml` 不会改变此入口。

默认日志：健康日志 `$HOME/fastlio_ws/logs/robot_health/<run_id>/`；回放控制日志 `$HOME/fastlio_ws/logs/robot_playback/<run_id>/`。每次运行产生独立目录，保留JSONL/CSV及配置快照。

### 操作与参数

- 在启动终端按空格，沿用C++回放控制器的暂停/恢复确认逻辑；控制状态对应 `PAUSED` / 恢复后重新就绪。
- 如果终端未向节点传入按键，可在另一个已加载ROS环境的终端通过服务明确暂停/恢复：

```bash
source /opt/ros/noetic/setup.bash
source $HOME/fastlio_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:33522
rosservice call /recorded_bag_playback/pause "data: true"
# 恢复时执行：
rosservice call /recorded_bag_playback/pause "data: false"
```

- 改2倍速：在roslaunch命令末尾增加 `rate:=2.0`。
- 不开RViz：增加 `rviz:=false`。
- 指定其他配置：增加 `health_config:=/绝对路径/配置.yaml`。
- 指定其他包：增加 `bag:=/绝对路径/文件.bag`，并同时适配消息、频率、单位和frame。
- 本入口连接就绪检查默认等待 `/rslidar_points`、`/mavros/imu/data_raw`、`/Odometry` 的监控订阅及原生播放器TCPROS连接。若在YAML中更换topic，需同步传入 `lidar_topic:=... imu_topic:=... localization_topic:=...`。

### 实现范围与验证状态

- 新launch不启动FAST-LIO，不创建虚假的雷达外参；显示的定位/配准点云来自包内已有结果。原始雷达 `rslidar` 无已知TF，因此默认不在 `camera_init` 下叠加原始雷达图层，但其质量检测正常订阅。
- C++回放控制器新增 `require_mapping_node`（默认true）和 `required_input_topics`（默认原来的两路输入）。新入口显式设为无需mapping进程并检查三路监控连接；原Avia入口保留原mapping节点要求。
- 暂停、恢复、EOF时钟核对和常驻心跳沿用既有C++路径，正常EOF目标状态为STOPPED；第4节提到的退出确认窗口瞬时接收告警风险仍保留，没有在本次更改中声称消除。
- 创建此入口时只完成编译，没有新增运行测试；第4节的12项通过来自先前自动验证。随后用户自行完整运行了新launch，实际日志分析见第9节：三路数量一致、正常EOF终态STOPPED，同时确认启动缺数和EOF过渡告警仍存在。本次用户运行未记录显式暂停事件，不能当成空格暂停/恢复已经实测。
- 本bag约44秒后才有雷达、约64秒后才有定位；默认启动期限仍为10秒，因此前段可出现启动缺数ERROR。原始首帧零时间戳及其他质量告警按规则保留。
- C++修改前备份与构建日志：`$HOME/fastlio_ws/优化备份/20261015_before_recorded_bag_roslaunch/`。原包和修复副本均未修改。

## 9. 用户roslaunch实际日志分析（2026-10-15 10:26）

本节只读取用户已经产生的日志，没有再次回放，没有新增测试，也没有修改配置。结构化摘要：[analysis.json](evidence/v2/roslaunch_log_analysis_20261015/analysis.json)。

### 日志位置与时间

健康日志根目录：`$HOME/fastlio_ws/logs/robot_health/`，本次run为 `7af80-18de92d97c8b2be0-2536610f`。

- events.0.jsonl（本机文件，不随仓库发布）：规则触发/恢复、生命周期和控制意图。
- metrics.0.csv（本机文件，不随仓库发布）：约每秒采样指标和当前代际计数，不能替代短时事件日志。
- effective_config.yaml（本机文件，不随仓库发布）：本次实际生效配置；100Hz、软消抖0秒、IMU接收年龄0.1/0.2秒。
- manifest.json（本机文件，不随仓库发布）：数据路径/哈希、配置路径/哈希、二进制版本等追溯信息。

回放控制日志：robot_playback/7af86-18de92d97d4f43b0-acd34401/events.0.jsonl（本机文件，不随仓库发布），记录起播、EOF核对与完成。

`wall_time_utc` 的 `Z` 表示UTC，例如日志 `02:26:15Z` 对应北京时间 **10:26:15**。`steady_elapsed_sec` 是各自记录器的单调时间经过量，包含初始化时间，不等于bag已经播放的秒数，不直接拿两个进程的该字段相减。本次控制日志显示北京时间约10:26:15起播、10:31:02确认完成、10:31:27关闭。

### 为什么60多秒才看到定位

当前入口没有启动定位算法进程，检测的是bag内已有的 `/Odometry`。从本包最早记录时间计算：

| 话题 | 首条记录距bag开始 | 含义 |
| --- | ---: | --- |
| `/mavros/imu/data_raw` | 0.00065 s | IMU基本从开头就有 |
| `/fused_cloud` | 26.167 s | 另一种融合点云，不能直接当作目标定位输出 |
| `/rslidar_points` | 44.301 s | 本次检测的原始雷达开始有记录 |
| `/localization_usage` | 63.838 s | 包内另一定位/融合话题 |
| `/Odometry` | **64.134 s** | 本次检测的定位输出首次有记录 |
| `/cloud_registered` | 64.135 s | 包内已配准点云首次有记录 |

因此1倍速回放到约64秒以前，播放器没有这路Odometry消息可以发送；不是检测台故意等60秒，也不能据此断言原实机定位进程恰在第64秒才启动。原录制时为何晚出现，需原机节点启动/录制日志才能进一步定位。

收到首条定位也不会立即变绿：还需要连续有效样本、频率统计覆盖与恢复保持。此次CSV首次采到 `localization OK/RUNNING` 是北京时间10:27:22，约为起播后67秒；CSV每秒一次，不能把该采样时间当成精确首帧到达时刻。

### 告警逐项解释

| 事件行号 | 内容 | 分析 |
| --- | --- | --- |
| 21、28 | 雷达、定位 `STARTUP_TIMEOUT`，期限10秒 | RUN后10秒仍未就绪，而记录本身分别44/64秒才出现，两者不匹配；不说明回放程序卡住 |
| 13、34、37 | IMU `STAMP_GAP WARNING`，约21.04/23.44/22.40ms | 超过20ms源间隔阈值；这是消息头间隔，不是接收年龄 |
| 40、42 | 雷达 `STAMP_INVALID ERROR`，后恢复 | 原包第一帧header.stamp=0；后续时间有效，没有修改原数据来消除告警 |
| 47 | 定位 `LOC_ACCELERATION WARNING`，12.543m/s²，阈值10 | 位置差分运动指标越过当前警告阈值；不能单凭此认定定位精度不合格，需结合运动/算法情况 |
| 44、45、51、52、53 | 相关启动/运行/运动规则恢复 | `FAULT -> RUNNING`的STATE_CHANGED事件当次仍可能带ERROR，源于恢复保持未结束，不等于又发生一次新故障 |
| 55、57 | EOF前IMU接收年龄108.38ms WARNING、208.44ms ERROR | 最后一批数据已经结束，但原生进程尚未退出确认；不是bag中间注入了断流 |
| 62、64、66 | EOF前雷达/定位接收年龄告警 | 同属结束确认空窗；定位未等到1秒ERROR阈值便进入STOPPED |
| 73～78 | 确认正常EOF后进入STOPPED | 输入要求停止、保留数据显示；后续约25秒保持STOPPED直到关闭 |

由末尾接收年龄和STOPPED控制事件推算，最后IMU接收到STOPPED确认约 **0.639秒**。这说明EOF最终状态正确，但过渡瞬时误报确实仍需优化，不能用最终STOPPED掩盖历史ERROR。

### 完整性结论与调整建议

本次最后代际计数：雷达 **2421/2421**、IMU **56488/56488**、定位 **2223/2223**，与副本中三路数量一致；回放控制日志确认原生退出码0且时钟到文件末尾。本轮没有显式暂停事件，初始 `PLAYBACK_RESUMED` 仅表示自动解除起播暂停，不是用户按键验证记录。

如果把前64秒视为本bag的预期启动阶段，可以单独调整现有雷达/定位模块的 `lifecycle.startup_deadline_sec`，例如雷达60秒、定位90秒，避免把预期迟到当作启动失败；这只改变启动阶段如何报警，不会让原包提前产生定位，也不应通过放宽IMU的0.1/0.2秒接收阈值来处理。本次只分析，没有替用户修改这些参数。

## 10. 消息健康日志逐项解读：重点看什么

分析对象仍是第9节的健康 `events.0.jsonl`，没有重跑、修改日志或配置。补充证据：[message_health_analysis.json](evidence/v2/roslaunch_log_analysis_20261015/message_health_analysis.json)，含源日志哈希、行号、候选与正式告警分组。

### 10.1 82行日志不等于82次故障

日志包含启动、控制、候选、正式触发、恢复和状态变化。18条 `RULE_TRIGGERED` 中：启动缺数相关6条，消息质量相关5条，EOF过渡相关7条。一件事可能同时触发接收年龄、生命周期汇总故障和状态变化，不能把它们相加当作独立故障。

- `WARNING_CANDIDATE` 是通用候选事件名称，`to_level` 也可能为ERROR；不要仅按事件名中的WARNING判级。
- `RULE_TRIGGERED` 表示某条规则正式进入/升级告警，`RULE_RECOVERED` 表示该规则解除。
- `STATE_CHANGED` 看 `reason` 中的STARTING/RUNNING/FAULT/STOPPED变化；其 `to_level` 是当次综合严重度，不是运行状态。第43、50行虽然是FAULT→RUNNING，当时历史规则还处于恢复保持，仍可能带ERROR。
- `RUNTIME_FAULT` 是生命周期汇总，不等于抓到了进程崩溃。这里应结合相邻的STARTUP_TIMEOUT或RECEIVE_AGE解释。

### 10.2 IMU消息：时间间隔有短时异常，不能只说“3条异常消息”

检测话题 `/mavros/imu/data_raw`，`STAMP_GAP` 查相邻消息头时间差；当前WARNING阈值20ms、ERROR阈值100ms。

| 正式触发行 | 首次触发值 | 恢复行 | 告警保持时长 |
| --- | ---: | --- | ---: |
| 13 | 21.039ms | 32 | 2.746s |
| 34 | 23.441ms | 35 | 1.012s |
| 37 | 22.403ms | 38 | 1.002s |

这是 **3段正式WARNING**。候选日志实际有7条（12、14、15、30、31、33、36行），记录了约21.04、22.75、20.05、22.86、23.08、23.44、22.40ms。第一段中连续出现多个间隔超限，使1秒恢复保持被重新打断，因此直到第32行才解除。

候选不是全量逐帧记录：连续同级异常可以合并，不应把7条候选当成准确的异常帧总数。2.746秒的告警保持也不等于IMU连续2.746秒无数据；此处实际超限量是约20～23ms的头时间间隔。

这几段没有升级为 `STAMP_GAP ERROR`。不能仅凭间隔推断究竟漏了几帧、还是源采样/驱动时间抖动，需要连续原始消息/序号或驱动证据。总接收数与bag一致，不能据此把它定性为本次回放额外丢包。

上下文还表明：单位 `mps2`、换算系数1，相关事件样本的加速度模长约9.8m/s²，ACCEL/GYRO协方差标记AVAILABLE。这些样本没有显示此前Livox单位错误。但 `checks_available` 明确注明 `noise/stuck/saturation uncalibrated`，因此本日志不能证明噪声、卡死、饱和和标定均合格。

### 10.3 雷达消息：明确的零时间戳错误

第40行 `STAMP_INVALID ERROR` 的 `context.source_stamp_sec` 为0，话题 `/rslidar_points`、frame为 `rslidar`；第42行有效时间恢复并解除规则。这是明确的消息头质量问题，时间无效会影响关联、同步及基于时间的计算，不能通过关闭时间戳检查来当作已修好。

触发到恢复约1.201秒，其中包含1秒恢复保持，不能推断这1.201秒的每帧都为0。本日志没有点数不足/无有效点比例等正式告警，不等于仅凭events便可证明所有点云指标和每帧极值；正常连续指标应看metrics或原始点云。

### 10.4 定位消息：差分加速度告警，日志还记录了14.73m/s²

第47行 `/Odometry` 的 `LOC_ACCELERATION` 首次触发值12.543m/s²；第48行又记录14.732m/s²。当前阈值为WARNING 10、ERROR 30，所以维持WARNING而不是ERROR；第53行规则恢复，此时值1.756m/s²。

14.732只是候选记录中较大值，不是全量消息的已验证峰值。当前 `velocity_source=position_difference`，该加速度由位置和源时间戳差分得到，并非IMU加速度计读数。事件发生在定位刚开始有输出的阶段，可能涉及真实运动或估计瞬态，需结合连续轨迹判断，单凭这份events不能定性定位精度差，也不能把它当成已验证的物理加速度真值。

没有记录定位速度、旋转/位置跳变、协方差越界等正式告警，不代表这些未告警指标的全量数值已存于events；需要指标曲线或原始定位数据进一步评估。

### 10.5 接收年龄：开头是缺数，末尾是结束确认空窗

第17/24行的接收年龄约11.3秒发生在雷达/定位还没有任何数据时。源码在零消息情况下从当前代际初始化计时，不是证明“最后一帧到现在已经11.3秒”。该阶段超过10秒启动期限，于是连带触发STARTUP_TIMEOUT与RUNTIME_FAULT；不能理解为已经稳定工作过又突然中断。

第55与57行则是明确的定时探针升级：

| 行 | 单调经过量 | 接收年龄 | 结果 | 上次消息源时间 |
| --- | ---: | ---: | --- | --- |
| 55 | 297.322012s | 108.384ms | WARNING，阈值100ms | 1791962664.85 |
| 57 | 297.422058s | 208.437ms | ERROR，阈值200ms | 1791962664.85 |

两次之间约100.046ms，上次源时间保持不变，说明没有靠下一条消息来触发升级。这个探针功能在本次末尾确实执行了，但触发场景是bag数据已结束、STOPPED意图尚未确认，不能拿来当作中途实机断流证据。雷达末尾也有306.6ms WARNING和506.6ms ERROR，定位有309.7ms WARNING。

第79～81行虽然写 `RULE_RECOVERED`，但 `reason` 是 **`data demand suspended`**，`value=0` 是通过规则的占位值，且上下文仍是原来最后一帧：它表示进入STOPPED后不再要求持续数据，不表示IMU/雷达/定位重新收到数据，也不是接收年龄真的重置成0。

### 10.6 当前应该关注的事项

1. 源消息方面：优先排查雷达首帧零时间戳；核查IMU短时头间隔不均匀、定位初始差分加速度是否符合平台实际运动。不能只看最终绿灯。
2. 监控生命周期方面：按实际启动顺序设置雷达/定位的启动期限；缩短或明确处理EOF结束确认空窗。两者与传感器数据自身的异常应分别处理。
3. 日志表达方面：恢复有“收到正常消息”和“停止后取消数据要求”两种原因；候选、正式告警及汇总状态也不是同一类事件。分析时结合 `event_type + rule_id + reason + context`，不能只搜索ERROR或统计行数。

这份日志没有证明持续运行阶段出现长时间传感器断流，也不能单凭未出现某规则告警证明噪声、定位精度或硬件状态全面健康。本次只分析，未替用户放宽阈值、修改数据或消除历史告警。

## 11. 10:56再次运行：源间隔阈值调整后的实际结果

用户再次运行产生的新日志：

- 健康事件：7cae4/events.0.jsonl（本机文件，不随仓库发布）。
- 实际配置：effective_config.yaml（本机文件，不随仓库发布）。
- 回放控制：7cae6/events.0.jsonl（本机文件，不随仓库发布）。
- 本次只读分析摘要：[7cae4_analysis.json](evidence/v2/roslaunch_log_analysis_20261015/7cae4_analysis.json)。没有再次回放或修改配置。

### 11.1 核心变化是阈值，不是bag数据改变

对两轮 `effective_config.yaml` 做字段核对，生效配置仅有下面两项变化，bag内容哈希一致：

| IMU配置 | 前一轮 | 本轮 |
| --- | ---: | ---: |
| `stream.stamp.max_gap_warning_sec` | 0.02 s | **0.1 s** |
| `stream.stamp.max_gap_error_sec` | 0.1 s | **0.2 s** |
| `stream.receive_age_sec.warning_above` | 0.1 s | 0.1 s，未变 |
| `stream.receive_age_sec.error_above` | 0.2 s | 0.2 s，未变 |

因此包内20～23ms头时间间隔不再超过新WARNING阈值100ms，本轮没有 `STAMP_GAP` 事件。不能据此说间隔问题已被修复；现在这类间隔被新规则允许。即使两项阈值的数值相同，STAMP_GAP仍是新帧到达后检查头时间差，RECEIVE_AGE仍是无新帧时持续检查现实接收空窗。

健康日志由82行变为69行，恰好少了上一轮13条IMU STAMP_GAP相关事件（7候选、3触发、3恢复）；正式触发由18条变为15条：启动缺数6条、消息质量2条、EOF过渡7条。不能用行数减少直接证明传感器质量变好。

### 11.2 仍然出现的健康问题

| 本轮事件行 | 内容 | 结论 |
| --- | --- | --- |
| 17、24 | 雷达、定位启动超时，期限10秒 | 本bag44/64秒才开始有对应记录，启动期限不匹配仍存在 |
| 27→29 | 雷达头时间戳0，ERROR后恢复 | 同一原始消息问题，未被阈值修改消除 |
| 34、35→40 | 定位加速度首次12.543、候选14.732m/s²，随后恢复为1.756 | 仍为WARNING：阈值10/30m/s²；14.732不是全量峰值证明 |
| 42→44 | IMU接收年龄104.958ms WARNING、204.993ms ERROR | 发生在最后源时间1791962664.85不再变化时，定时检查仍生效 |
| 49、53 | 雷达接收年龄304.45ms WARNING、504.40ms ERROR | EOF确认前空窗 |
| 51 | 定位接收年龄306.63ms WARNING | EOF确认前空窗，尚未达到1秒ERROR阈值 |
| 60～65 | EOF确认后进入STOPPED | 最终状态正确；不代表前面的短暂误报已消除 |
| 66～68 | `data demand suspended`恢复 | 停止数据需求，并非收到新数据恢复 |

IMU两次接收告警相隔约100.015ms，源时间相同，说明这次同样没有等待下一帧才升级ERROR。最后IMU接收到STOPPED确认约 **0.4986秒**；与上一轮数值不同不能解释成代码优化，本次两轮之间没有此项代码改动，EOF过渡问题仍然存在。

### 11.3 完整性、运行段与结论边界

- 最后代际计数：IMU **56,488**、雷达 **2,421**、定位 **2,223**，全部与副本对应话题数量一致；三项最后均STOPPED。
- 回放控制确认为1倍速、原生退出码0、时钟到文件末尾。本轮约北京时间10:56:52起播、11:01:39确认结束、11:01:43关闭，STOPPED保持约4秒。
- 定位加速度规则解除后至EOF前，没有记录新的运行段规则触发。单调经过量73～292秒窗口内无新增 `RULE_TRIGGERED`，不等于所有未实现或未标定指标全面通过。
- 该窗口内75～292秒、RUNNING的每秒CSV采样各约215个：雷达有效点比例均为1.0、采样点数18,126～57,231；IMU源频率约175.0～210.4Hz、定位约9.957～10.026Hz。这些是采样范围，不是每帧全量极值，也不能证明精度/噪声质量。
- 没有显式暂停事件，初始PLAYBACK_RESUMED仍只是自动起播，不能用本轮证明手动暂停/恢复。

结论：本轮完整回放和接收数量核对正常，IMU源间隔报警按新阈值不再触发；雷达首帧零时间戳、定位初始差分加速度WARNING、启动缺数及EOF过渡告警仍保留。接收年龄探针未因源间隔阈值修改而失效。本次未替用户修改配置或代码。


## 12. 三时间字段与自动封存的新一轮实测

2026-10-15已使用同一修复副本完整1倍速重跑，未改变包与健康阈值。新日志仅三种逐条时间，消息接收一致，暂停恢复和正常EOF封存、10秒不追加已验证。健康日志run为 `80143-18de96fe03a13164-61cfe4e`，详见[21_三时间字段回放实测与日志分析](21_三时间字段回放实测与日志分析.md)。历史run及其格式保留。
