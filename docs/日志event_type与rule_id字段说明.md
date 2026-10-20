# 日志中的 event_type 与 rule_id

本文列出 V2 工程 JSONL 事件日志里由现有 C++ 代码产生的 `event_type` 和 `rule_id`，并给出源码位置和含义。范围包括健康监控节点、运行控制器和 bag 播放器。新增适配器或规则后，列表也需要相应更新。

## 1. 两个字段的区别

- `event_type` 表示这条日志记录发生了什么，例如告警触发、状态改变或进程退出。
- `rule_id` 表示这条记录关联哪个检测规则或事件来源，例如 `IMU_ACCEL_NORM` 或 `LIFECYCLE`。
- 同一个 `rule_id` 可以出现在不同 `event_type` 中。例如同一检测规则可依次产生 `WARNING_CANDIDATE`、`RULE_TRIGGERED`、`RULE_RECOVERED`。
- 这些名称是代码里的稳定编号，不是在 YAML 里逐个填写。YAML 控制对应模块或检查项是否启用、阈值是多少。

日志字段由 [`EventRecorder::enqueueEvent()`](../fastlio_health_monitor/src/event_recorder.cpp#L280) 写入。健康规则状态事件由 [`RuleLatch::apply()`](../fastlio_health_monitor/src/health_core.cpp#L750) 生成。

## 2. 全部 event_type

### 健康监控与日志生命周期

位置：[`event_recorder.cpp`](../fastlio_health_monitor/src/event_recorder.cpp#L230)、[`health_core.cpp`](../fastlio_health_monitor/src/health_core.cpp#L750) 和 [`robot_health_monitor_node.cpp`](../fastlio_health_monitor/src/robot_health_monitor_node.cpp#L25)。

| event_type | 含义 |
| --- | --- |
| `RUN_BEGIN` | 本次日志开始。 |
| `RUN_END` | 本次日志结束并封存。 |
| `WARNING_CANDIDATE` | 规则刚发现异常候选。软告警还可能处于保持计时阶段；硬错误随后会立即触发。 |
| `RULE_TRIGGERED` | 规则告警正式生效或等级升级。 |
| `RULE_RECOVERED` | 规则达到恢复条件，告警解除。 |
| `INPUT_AVAILABLE` | 可选模块收到第一条输入，开始参与健康检测。 |
| `INSTANCE_RESET` | 接受新的进程实例或数据代际，并重置旧的时间、运动学和告警基线。 |
| `CONTROL_INTENT` | 接受了新的控制命令或启停意图。 |
| `STATE_CHANGED` | 模块运行状态发生变化；旧状态和新状态写在 `reason` 中。 |

### 运行控制器

位置：[`runtime_controller_node.cpp`](../fastlio_health_monitor/src/runtime_controller_node.cpp#L160)。这些事件的 `rule_id` 固定为 `RUNTIME_CONTROL`。

| event_type | 含义 |
| --- | --- |
| `PLAYBACK_SOURCE_CHANGED` | 被监控的播放源状态或实例发生变化。 |
| `START_REQUESTED` | 请求启动受管理进程。 |
| `START_FAILED` | 启动进程失败。 |
| `PROCESS_STARTED` | 子进程已启动；这不单独代表其输出数据已经就绪。 |
| `STOP_REQUESTED` | 请求停止受管理进程。 |
| `STOP_CONFIRMED` | 进程已停止并得到确认，或进程本来就已停止。 |
| `FRAME_RESET` | 执行了授权的坐标系／数据基线重置。 |
| `TASK_COMPLETED` | 配置的任务正常完成。 |
| `PROCESS_LOST` | 进程意外退出或被强制停止。 |
| `STOP_FAILED` | 停止确认超时，控制器强制结束进程。 |
| `ROS_NODE_LOST` | 被监督的 ROS 节点失联。 |

### bag 播放器

位置：[`bag_playback_node.cpp`](../fastlio_health_monitor/src/bag_playback_node.cpp#L135)。这些事件的 `rule_id` 固定为 `PLAYBACK_STATE`。

| event_type | 含义 |
| --- | --- |
| `PLAYBACK_STARTING` | 播放器正在等待必要订阅端和传输连接就绪。 |
| `PLAYBACK_PAUSED` | 暂停请求已确认。 |
| `PLAYBACK_RESUMED` | 恢复播放请求已确认。 |
| `PLAYBACK_FAILED` | 播放器启动失败、异常退出或连接检查失败。 |
| `MAPPING_NODE_LOST` | 建图节点不再响应。 |
| `MAPPING_NODE_RESTORED` | 建图节点恢复响应。 |
| `PLAYBACK_VERIFYING_EOF` | 播放进程已退出，正在核实 `/clock` 是否到达 bag 末尾。 |
| `PLAYBACK_COMPLETED` | 已确认正常播放到 bag 末尾。 |

## 3. 全部内置 rule_id

下面是当前内置代码会产生的规则 ID。正常样本也可能为规则提交 `OK` 结果，但日志只在告警候选、触发、恢复等事件发生时记录，不会为每个正常样本重复写一条事件。

### 日志、控制和生命周期

位置：日志 ID 在 [`event_recorder.cpp`](../fastlio_health_monitor/src/event_recorder.cpp#L230)、控制器 ID 在 [`runtime_controller_node.cpp`](../fastlio_health_monitor/src/runtime_controller_node.cpp#L166)、播放器 ID 在 [`bag_playback_node.cpp`](../fastlio_health_monitor/src/bag_playback_node.cpp#L216)、生命周期 ID 在 [`health_core.cpp`](../fastlio_health_monitor/src/health_core.cpp#L1317)。

| rule_id | 含义 |
| --- | --- |
| `RUN` | 日志开始或结束记录。 |
| `RUNTIME_CONTROL` | 运行控制器的进程、启停和监督事件。 |
| `PLAYBACK_STATE` | bag 播放器的播放状态事件。 |
| `LIFECYCLE` | 模块运行状态改变、控制意图或实例重置事件；不是数值检测规则。 |
| `INPUT_AVAILABLE` | 可选输入首次到达事件。 |
| `CONTROL_UNKNOWN` | 严格控制模式下尚无有效控制意图；收到有效意图后恢复。 |
| `CONTROL_STALE` | 控制器心跳超时；恢复后清除。 |
| `CONTROL_TRACE` | 运行控制日志追踪是否完整。 |
| `CONTROL_HEARTBEAT` | 控制器心跳年龄是否超过警告阈值。 |
| `STARTUP_TIMEOUT` | 模块在启动期限内未达到数据就绪条件。 |
| `STOP_FAILED` | 停止请求在期限内没有得到确认。控制器事件和健康监控规则都可能使用这个 ID。 |
| `RUNTIME_FAULT` | 模块当前处于启动、进程、输出或停止故障状态。 |

### 时间戳、数据年龄与频率

位置：[`StreamTracker::receive()` 和 `StreamTracker::evaluate()`](../fastlio_health_monitor/src/health_core.cpp#L670)。

| rule_id | 含义 |
| --- | --- |
| `STAMP_INVALID` | 源时间戳非有限、为零或不满足正时间要求。 |
| `STAMP_DUPLICATE` | 连续有效消息的源时间戳重复。 |
| `STAMP_REGRESSION` | 源时间戳比前一条消息倒退。 |
| `STAMP_GAP` | 相邻源时间戳间隔超过配置值。 |
| `RECEIVE_AGE` | 本机距离最后一次收到消息的时间超过配置值。 |
| `FREQUENCY_LOW` | 计算频率低于配置下限。 |
| `FREQUENCY_HIGH` | 计算频率高于配置上限。 |

### 雷达

适配器结构检查位置：[`input_adapters.cpp`](../fastlio_health_monitor/src/input_adapters.cpp#L100)；点数和有效率检查位置：[`health_core.cpp`](../fastlio_health_monitor/src/health_core.cpp#L838)。

| rule_id | 含义 |
| --- | --- |
| `LIDAR_STRUCTURE` | 点云消息结构或内部点数信息不一致。 |
| `LIDAR_COUNT_TOTAL` | 每帧点数总量越过配置边界。 |
| `LIDAR_COUNT_VALID` | 每帧有效点数越过配置边界。 |
| `LIDAR_RATIO` | 有效点占比越过配置边界。 |
| `LIDAR_EMPTY` | 点云为空或没有有效点。 |

### IMU

适配器规则位置：[`input_adapters.cpp`](../fastlio_health_monitor/src/input_adapters.cpp#L285)；幅值规则位置：[`health_core.cpp`](../fastlio_health_monitor/src/health_core.cpp#L960)。

| rule_id | 含义 |
| --- | --- |
| `IMU_ACCEL_AVAILABLE` | 按配置要求检查加速度测量是否可用。 |
| `IMU_GYRO_AVAILABLE` | 按配置要求检查陀螺仪测量是否可用。 |
| `IMU_ORIENTATION_AVAILABLE` | 按配置要求检查 IMU 姿态是否可用。 |
| `IMU_ACCEL_FINITE` | 加速度是否为有限数值，单位换算后是否溢出。 |
| `IMU_GYRO_FINITE` | 角速度是否为有限数值。 |
| `IMU_ORIENTATION_FINITE` | 姿态四元数是否有效。 |
| `IMU_ACCEL_COV_UNKNOWN` | 加速度协方差未知。 |
| `IMU_GYRO_COV_UNKNOWN` | 陀螺仪协方差未知。 |
| `IMU_ORIENTATION_COV_UNKNOWN` | 姿态协方差未知。 |
| `IMU_ACCEL_COV_INVALID` | 加速度协方差矩阵数值无效。 |
| `IMU_GYRO_COV_INVALID` | 陀螺仪协方差矩阵数值无效。 |
| `IMU_ORIENTATION_COV_INVALID` | 姿态协方差矩阵数值无效。 |
| `IMU_ACCEL_NORM` | 加速度三轴向量模长超出配置范围。 |
| `IMU_GYRO_NORM` | 角速度三轴向量模长超出配置范围。 |

### 姿态角：IMU 和定位共用

位置：[`AttitudeTracker::update()`](../fastlio_health_monitor/src/health_core.cpp#L864)。代码用 `IMU_` 或 `LOC_` 前缀区分数据来源，因此例如 `IMU_ROLL_LIMIT` 与 `LOC_ROLL_LIMIT` 是两个独立 ID。

| ID 后缀 | 含义 |
| --- | --- |
| `ATTITUDE_AVAILABLE` | 姿态数据是否提供。 |
| `ATTITUDE_QUATERNION` | 姿态四元数是否有限且非零。 |
| `ATTITUDE_NORM` | 四元数单位范数误差是否超限。 |
| `ATTITUDE_EULER` | 欧拉角是否可用；pitch 接近 ±90° 时 roll 不可定义。 |
| `ROLL_LIMIT` | roll 绝对角度是否超限。 |
| `PITCH_LIMIT` | pitch 绝对角度是否超限。 |
| `ATTITUDE_JUMP` | 相邻姿态旋转跳变是否过大。 |
| `ATTITUDE_RATE` | 相邻姿态估计的角速度是否过大。 |

实际 ID 形式：`IMU_ATTITUDE_AVAILABLE`、`LOC_ATTITUDE_AVAILABLE`，其他后缀同理。

### 测距

消息视场检查位置：[`input_adapters.cpp`](../fastlio_health_monitor/src/input_adapters.cpp#L490)；距离和连续性检查位置：[`RangeChecker::check()`](../fastlio_health_monitor/src/health_core.cpp#L980)。

| rule_id | 含义 |
| --- | --- |
| `RANGE_FOV` | 测距消息视场角字段非法。 |
| `RANGE_METADATA` | 消息声明的最小／最大量程不合法。 |
| `RANGE_RETURN` | 没有有限的回波距离，或传感器未返回可用距离。 |
| `RANGE_VALUE` | 距离非有限、为负值或超出消息／配置量程。 |
| `RANGE_HEIGHT_LOW` | 测距值低于启用的下限。 |
| `RANGE_HEIGHT_HIGH` | 测距值高于启用的上限。 |
| `RANGE_JUMP` | 相邻测距值变化量超限。 |
| `RANGE_RATE` | 测距变化率超限。 |

`RANGE_HEIGHT_LOW/HIGH` 保留了历史规则名称；程序检查的是沿传感器光束的距离，不会自动投影成标定后的垂直离地高度。

### 定位

消息结构检查位置：[`input_adapters.cpp`](../fastlio_health_monitor/src/input_adapters.cpp#L408)；运动学、跳变与协方差检查位置：[`health_core.cpp`](../fastlio_health_monitor/src/health_core.cpp#L1088)。

| rule_id | 含义 |
| --- | --- |
| `LOC_POSE_FINITE` | 位姿的位置或四元数含非有限值。 |
| `LOC_QUATERNION_ZERO` | 定位姿态四元数为零或无效。 |
| `LOC_QUATERNION_NORM` | 定位四元数归一化误差超限。 |
| `LOC_EXPECTED_FRAME` | 位姿参考坐标系与配置不符。 |
| `LOC_CHILD_FRAME` | 子坐标系与配置不符。 |
| `LOC_TWIST_FINITE` | 配置采用的 twist 速度含非有限值。 |
| `LOC_FRAME_CHANGE` | 坐标系发生未授权变化。 |
| `LOC_SPEED` | 定位速度超限。 |
| `LOC_ACCELERATION` | 定位加速度超限。 |
| `LOC_JUMP` | 连续定位位置跳变超限。 |
| `LOC_ANGULAR_SPEED` | 定位姿态角速度超限。 |
| `LOC_ROTATION_JUMP` | 连续定位姿态旋转跳变超限。 |
| `LOC_COVARIANCE_UNKNOWN` | 定位协方差不可用或未知。 |
| `LOC_COVARIANCE_INVALID` | 定位协方差矩阵数值无效。 |
| `LOC_POSITION_VARIANCE` | 位置方差超限。 |
| `LOC_ORIENTATION_VARIANCE` | 姿态方差超限。 |

### 诊断转发与规划

位置：[`input_adapters.cpp`](../fastlio_health_monitor/src/input_adapters.cpp#L518)。

| rule_id | 含义 |
| --- | --- |
| `RELAY_LEVEL_VALID` | 上游诊断等级是否在有效范围内。 |
| `RELAY_NONEMPTY` | 上游诊断数组是否非空。 |
| `RELAY_STATUS` | 上游 DiagnosticArray 汇总后的状态。 |
| `PLANNING_TASK_FRESH` | 规划任务状态是否新鲜。 |
| `PLANNING_GOAL_ID` | 活动目标是否具有有效 ID 和时间戳。 |
| `PLANNING_DEADLINE` | 活动目标是否在期限内获得新路径；无活动目标时路径需求暂停。 |
| `PLANNING_PATH_FINITE` | 路径中的位置和姿态是否为有限有效值。 |
| `PLANNING_PATH_FRAME` | 路径各点坐标系是否一致。 |
| `PLANNING_PATH_CONTINUITY` | 相邻路径段是否超过配置长度。 |
| `PLANNING_PATH_EMPTY` | 活动目标是否收到非空路径。 |
| `PLANNING_GOAL_EPOCH` | 路径时间戳是否晚于当前目标时间戳。 |

## 4. 对照日志的例子

```json
{"event_type":"RULE_TRIGGERED","rule_id":"IMU_ACCEL_NORM","module_id":"imu"}
```

含义：`imu` 模块的加速度模长规则已触发。

```json
{"event_type":"STATE_CHANGED","rule_id":"LIFECYCLE","module_id":"imu","reason":"STARTING -> RUNNING"}
```

含义：`imu` 模块从启动准备状态进入运行状态。状态文字看 `reason`；`from_level` 和 `to_level` 表示健康等级，不代表运行状态文字。

## 5. YAML 配置入口

- `health_robot.yaml` 配真实机器人输入，`health_bag.yaml` 配 bag 回放输入。
- 数据时间、接收年龄和频率规则在每个模块的 `stream` 区域。
- 雷达、IMU、姿态、定位、测距、规划的业务规则在对应模块的 `quality` 区域。
- 控制心跳在根级 `control` 区域；启动／停止期限和连续就绪样本数在 `lifecycle_defaults` 或模块的 `lifecycle` 覆盖项中。
- `logging.enabled` 控制整体日志记录。当前配置没有为某个 `event_type` 或 `rule_id` 单独设置的 YAML 开关。

字段的阈值、默认值及每项 `enabled` 的具体说明见 [`health_robot.yaml`](../fastlio_health_monitor/config/health_robot.yaml) 和 [`health_bag.yaml`](../fastlio_health_monitor/config/health_bag.yaml) 中的配置注释。
