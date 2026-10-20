# 配置精简与enabled开关

更新：2026-10-19。当前桌面包主配置目录只有三份YAML。

## 1. 选对文件

| 你做什么 | 改哪个 |
| --- | --- |
| 接真实机器人 | src/fastlio_health_monitor/config/health_robot.yaml |
| 回放bag | src/fastlio_health_monitor/config/health_bag.yaml |
| 程序托管定位的启动/停止 | src/fastlio_health_monitor/config/runtime_localization.example.yaml |

三模块、最小版、同一硬件的重复别名不再各占一个配置。测试专用感知/规划材料位于test/fixtures，离线交付包evidence里的YAML是历史运行快照；GitHub仓库不提交这些运行快照。

## 2. 日常只改enabled

```yaml
receive_age_sec:
  enabled: false
  warning_above: 0.1
  error_above: 0.2
```

true启用，false关闭。关闭后阈值留着，恢复时只改true。一定写`enabled: false`，冒号后面有空格；`enabled:false`在YAML里可能不是键值配置。

## 3. 开关完整位置

| 层级 | 开关 |
| --- | --- |
| 模块 | 每个modules条目都写enabled |
| 接收年龄 | stream.receive_age_sec.enabled |
| 输入频率 | stream.frequency.enabled |
| 源时间戳整组 | stream.stamp.enabled |
| 重复/倒退等级规则 | stream.stamp.duplicate_level.enabled / regression_level.enabled |
| 雷达数量/比例/人工范围 | quality.points_total / points_valid / valid_ratio / range_m各自的enabled |
| IMU幅值 | quality.acceleration_norm_mps2 / angular_velocity_norm_rps各自enabled |
| IMU/定位姿态 | quality.attitude.enabled，以及内部norm/roll/pitch/jump/rate各自enabled |
| 定位四元数归一误差 | quality.quaternion_norm_error.enabled |
| 定位运动 | quality.kinematics.enabled，以及内部speed/acceleration/angular_speed各自enabled |
| 定位跳变 | quality.jump.enabled，以及内部residual_m / rotation_rad各自enabled |
| 协方差 | quality.covariance.enabled；定位position/orientation variance各自enabled |
| 测距 | quality.valid_range_m / no_return_level / height_limits / continuity各自enabled，内部阈值块也完整写enabled |
| 日志 | logging.enabled |
| 托管进程组 | groups里的enabled |

原始IMU通常没有姿态，已明确写quality.attitude.enabled:false；融合IMU和定位姿态维持原先启用状态。测距任务高低限维持false，其子规则true只是保留参数，父规则false时不会执行。

require_positive、require_symmetric、require_psd、required_measurements里的测量要求等本身就是布尔开关，直接用true/false。Topic、单位、frame、检查频率、阈值数值是运行/计算参数，不额外添加解析器不支持的enabled。

## 4. 五模块改三模块

在health_bag.yaml或health_robot.yaml对应的模块条目，仅修改：

```yaml
- id: imu_attitude
  enabled: false
  # 其余现有字段保留。
- id: ground_range
  enabled: false
  # 其余现有字段保留。
```

lidar、imu、localization继续true。没有独立的三模块YAML。不要复制上面这个节选覆盖完整文件；在原条目改两行即可。

required:false表示不要求输入，它仍会被加载；enabled:false表示整个模块不加载。要关闭一个仍被其他模块depends_on依赖的模块，先同步调整依赖；框架不会静默忽略缺失依赖。

## 5. 加载与兼容

改YAML后重启同一个launch。实机默认health_robot.yaml，replay.launch及旧回放别名默认health_bag.yaml。每个模块的源时间规则完整展开，公共stream_defaults仅保留时钟域/频率基准。

当前修改的是桌面交付包；安装脚本复制出的工作区不会自动跟随更新。已经安装时需要同步新源码/配置并重新编译，或安装到新工作区。旧版本目录和历史日志保持原样，整理前备份在/home/lmw/fastlio_ws/优化备份/20261019_before_explicit_enabled。
