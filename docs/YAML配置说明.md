# YAML配置说明

## 1. 改哪一份

安装后的目录：`~/robot_health_ws_20261019/src/fastlio_health_monitor/config/`。

| 文件 | 对应入口 | 内容 |
| --- | --- | --- |
| health_robot.yaml | live.launch / monitor.launch / live_managed.launch | 实机五模块，测距可选 |
| health_bag.yaml | replay.launch | 回放五模块，测距可选 |
| runtime_localization.example.yaml | live_managed.launch | 真实定位进程启动命令、ROS节点名、启停管理 |

主目录只有上表三份。两份健康配置初始阈值相同，但分别维护；模块和规则开关已经写全。入口的config参数决定实际加载哪个文件；启动日志和每次run的config_source/effective_config用于核对。桌面安装材料不会自动更新安装后的文件。

## 2. 最常改的字段

```yaml
- id: localization
  enabled: true  # 模块开关，false将停止加载这一模块。
  checker: LocalizationChecker
  adapter: OdometryAdapter
  topic: /Odometry
  required: true
  lifecycle:
    control_binding: localization
  # stream与quality按实际文件保留完整结构
```

- id是模块名称，保持唯一；checker/adapter配对不能乱改。
- topic改成真实数据话题，不能只改大小写相近的另一个名字。
- required=true是起播必需输入；不会自动为你启动生产者。
- depends_on是业务依赖；依赖模块无效会使本模块operational_valid=false，不等于本模块消息一定损坏。

## 3. 时间与频率

| 字段 | 大白话 |
| --- | --- |
| monitor.publish_rate_hz | 多久检查/发布一次；100Hz理想间隔10ms，实际调度可能更晚 |
| statistics_window_sec | 统计最近几秒的频率；初始5秒 |
| min_frequency_samples / coverage | 数据数量和覆盖时间足够后才算稳定频率 |
| stream.receive_age_sec | 距最后一次收到消息多久；独立单调时钟探针，没下一帧也会报警 |
| stream.stamp.max_gap_* | 相邻消息头时间差；下一帧到了才能算 |
| stream.frequency | 源频率的上下限 |
| frequency_basis: source | 按header时间算源频率，bag加速不直接把源频率翻倍 |
| soft_warning_hold_sec | 软告警持续多久才触发；当前0，不额外消抖 |
| recovery_hold_sec | 好转后持续多久才恢复；当前1秒 |

当前IMU接收年龄0.1/0.2秒；测距3Hz约每0.333秒一帧，使用0.7/1.2秒初值，不能套用IMU阈值。改回放倍率会影响实际接收间隔，即使源频率仍正常，也可能需要匹配接收年龄策略。

所有above/below使用严格大于/小于。比如warning_above=30、error_above=45时，等于45仍不是“>45”的ERROR；超过45才进入该ERROR条件。

## 4. 雷达质量

- points_total / points_valid：总点数、有效点数下限。
- valid_ratio：有效点比例下限。
- range_m：计算有效点的距离范围。
- 空点云、坏字段/缓冲区、NaN点过多分别记录原因。

这些是输入点云质量，不代表环境几何一定适合定位，也不直接等于建图精度。

## 5. IMU单位和姿态

```yaml
acceleration_unit: mps2
angular_velocity_unit: radps
acceleration_content: specific_force
```

- mps2是m/s²，MAVROS按此接入；只有驱动明确以g输出才改g，程序会乘9.80665。
- specific_force包含重力，静止时模长约9.8；不要把它当成已经去重力的运动加速度。
- angular_velocity使用rad/s。
- covariance.acceleration_unit=input_squared表示协方差随输入单位换算平方缩放。

原始/data_raw缺姿态时orientation=false；融合/data模块orientation=true，并开启attitude。

IMU融合数据和Odometry各有一份独立姿态配置：

```yaml
attitude:
  enabled: true
  require_available: true
  quaternion_norm_error:
    enabled: true
    warning_above: 0.02
    error_above: 0.1
  roll_abs_deg:
    enabled: true
    warning_above: 30.0
    error_above: 45.0
  pitch_abs_deg:
    enabled: true
    warning_above: 30.0
    error_above: 45.0
  rotation_jump_deg:
    warning_above: 30.0
    error_above: 60.0
  angular_speed_dps:
    warning_above: 180.0
    error_above: 360.0
  dt_min_sec: 0.0001
  dt_max_sec: 1.0
```

角度单位度，使用绝对值；30/45只是工程初值，要按实际机动范围确认。缺失/无效四元数不输出假零角。接近pitch±90°时roll不唯一，显示N/A并保留四元数距离检查。姿态变化使用四元数最短旋转，避免q/-q和欧拉角跨界假跳变。

前提是世界参考/机体安装轴向清楚；IMU的header.frame_id是测量坐标系标签，不能单凭base_link认定世界方向已校准。当前不直接比较IMU角度与Odometry角度差。

## 6. 定位输出

- expected_pose_frame / expected_child_frame：对应header.frame_id / child_frame_id。
- kinematics.velocity_source=position_difference：按位置差分算速度，再计算加速度；不是把Odometry.twist自动当作可信速度。
- speed_mps、acceleration_mps2、angular_speed_rps：运动异常阈值。
- jump.residual_m：与运动预测相比的位置跳变。
- jump.rotation_rad：相邻姿态旋转跳变；单位弧度，和attitude中的“度”不要混淆。
- covariance：有限/对称/PSD及方差上限；未知/全零协方差策略可配置。

continuous_odometry表示连续里程计。若改成允许全局位置校正的global_localization，当前设计要求可信twist，不能继续用位置差分把地图校正误判成真实飞行速度。

## 7. 对地测距与无数据

```yaml
required: false
missing_data_policy: no_data_until_first_sample
```

这两项一起表示：未收到首帧时灰色“无数据”，不影响总体；收到首帧后自动开始正常检查和总体汇总；此后意外断流仍报警。第一条坏数据也会被检查。新控制实例可重置等待，普通断流不会自动退回无数据。

若任务要求启动必须有测距，改成required=true，并将missing_data_policy改为fault或删除该行。两者矛盾会拒绝配置。

距离量测：

- valid_range_m：设备有效距离范围；当前maximum=1000沿用驱动声明，是待真实型号确认的占位上限，不代表已验证物理量程。
- NaN、负值、超出有效范围为ERROR；±Inf没有可用定量距离，按显式配置的no_return_level处理；省略该规则不单独报警，但数值仍不可用。
- height_limits.enabled=false：默认不启用任务最低/最高测距限制，避免地面2～3cm被误判成低空故障。
- continuity：跳变差值和变化率。地形/倾角变化也会影响；变化率并非已经标定的垂直速度。

range沿传感器测量方向，不自动等于垂直对地高度。没有外参、地面与时间对齐信息时，不直接拿range与Odometry.z相减；本版本未实现倾角投影和自动飞行状态门控。

## 8. 正常启停与日志

lifecycle_defaults的startup_deadline_sec=10表示开始后10秒未就绪会告警；旧bag定位64秒才有数据时会先报启动缺数。ready_consecutive_samples还要结合频率统计覆盖和恢复保持。

control.strict_intent=false支持被动接入；但一旦接了可信控制器，仍检查心跳新鲜度。true表示必需模块都要有可信控制意图；只托管localization却未给sensors发状态时，不要盲目改true。

logging.directory默认 `~/.ros/robot_health/health`，显式~/前缀会展开到当前登录用户，不依赖原作者用户名。可以改成有写权限的绝对目录，不支持任意shell变量/命令替换。relative目录相对节点工作目录，部署时建议用~/或绝对路径。

metrics_period_sec控制CSV采样，不改变100Hz时间探针。队列/轮转/预算触发丢失或写失败会反映日志异常，不能把删除旧轮转文件当成“全历史完整”。

## 9. 现在主要用enabled开关

主要使用 `enabled: true/false`；保留检测规则和阈值，关闭只改false，恢复只改true。每个模块和支持开关的检测规则都已明确写出enabled。Topic、单位、参考系和发布频率是运行参数；原生布尔设置本身用true/false。

```yaml
receive_age_sec:
  enabled: false  # 关闭整个接收年龄检查。
  warning_above: 0.1
  error_above: 0.2
```

- 每个模块也有enabled；三模块模式关闭imu_attitude与ground_range。
- quality和stream是分组，不是独立开关；请改其内具体规则的enabled。
- stamp.enabled控制源时间规则整组；duplicate_level/regression_level内部也有独立enabled和level。
- 原生布尔项如require_positive、require_psd本身是开关，直接填true/false。
- 关闭父规则时子规则不执行，即便子规则写true；这样恢复父规则时无需重填参数。
- 当前主配置每模块时间规则完整展开，stream_defaults仅共享计算约定。解析器仍兼容旧版省略/null，但日常不靠删除/注释控制。
- 单位、发布频率等运行参数没有业务检测开关；保留并按注释填写。
- 修改后重启launch；如果只改YAML不用重新编译。NaN、坏结构等基础守卫仍执行。

详细清单见[配置精简与enabled开关](配置精简与enabled开关.md)，逐项参数含义见[全部配置项说明](全部配置项说明.md)。
