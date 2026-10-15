# FAST-LIO 定位健康监控（ROS1 / C++）

## V2当前版本（2026-10-15）

新增可复用C++核心库、标准消息适配器、独立运行控制器、结构化追溯日志和动态RViz面板。完整实现、阈值、状态协议、复现与截图见[12_V2重构实施与测试记录](../FAST_LIO/doc/ros1_workspace/12_V2重构实施与测试记录.md)。

2026-10-15 更新：rosbag 日志统一使用 `bag_elapsed_sec`、`source_stamp_sec`、`steady_elapsed_sec` 三种时间；通过正常 EOF 确认后封存本次 JSONL/CSV。新增 `recorded_bag_health.launch` 与 `health_v2_rslidar_mavros.yaml`，用于检查 bag 内已有的 RSLiDAR、MAVROS IMU 和定位结果，不启动 FAST-LIO。详见[自采包回放记录](../FAST_LIO/doc/ros1_workspace/19_自采Bag副本修复与原生回放测试.md)和[三时间字段分析](../FAST_LIO/doc/ros1_workspace/21_三时间字段回放实测与日志分析.md)。

```bash
# 先在Bash加载 /opt/ros/noetic/setup.bash 和工作区devel/setup.bash。
roslaunch fastlio_health_monitor managed_bag_testbench.launch

# 另一终端，正常关闭/重启定位；监控、管理端和传感器继续运行。
rosservice call /robot_runtime/control "{binding: 'localization', command: 'STOP'}"
rosservice call /robot_runtime/control "{binding: 'localization', command: 'RUN'}"
```

`STOP`响应表示意图已接收，实际完成等待RuntimeState.STOPPED；正常停止灰显、业务有效false，旧UInt8=WARNING。未发STOP的断流、管理心跳失联或控制日志失败仍报ERROR。正常停机没有当前数据健康判定，旧数据故障保留历史日志，当前周期收到的新硬错误仍可见。

普通一键回放命令`visualization_bag_testbench.launch`现在默认V2，可传`health_config:=...`，使用C++起播协调器：暂停启动播放器，确认算法和监控的两个输入均已建立实际TCPROS连接后恢复，避免漏收开头消息。`health_v2:=false`用于V1对照。V2单独启动使用`health_monitor_v2.launch`。

| 实际配置 | 用途 |
|---|---|
| `config/health_v2_avia.yaml` | Livox g输入、标准定位，独立/普通回放 |
| `config/health_v2_managed.yaml` | 需要明确管理意图及心跳 |
| `config/runtime_bag.yaml` | C++进程管理的命令列表，定位与传感器分别控制 |
| `config/health_v2_pointcloud2.yaml` | 标准点云、SI IMU、odom/base_link |
| `config/health_v2_rslidar_mavros.yaml` | RSLiDAR PointCloud2、MAVROS IMU 和 bag 内定位输出 |
| `config/health_v2_extensions.yaml` | 感知DiagnosticArray和规划Path+TaskState |

规划模板`FAST_LIO/doc/ros1_workspace/planned_config/health_monitor_v2.example.yaml`包含未实现字段，不能直接加载。噪声、真实饱和、时钟映射及定位精度仍待标定/验证；不把OK当作绝对精度证明。

功能源码均为C++14。测试/构建脚本在`test/`；原始bag保持不变，故障样例写入独立目录。日志目录由 YAML 中的 `logging.directory` 和 `runtime_bag.yaml` 的 `log_directory` 配置，包含配置/源/二进制/数据哈希及实例/代际/命令关联。

## V1兼容说明（下方历史参数/单独启动入口）


独立 Catkin 包，订阅雷达原始点云、IMU 和 FAST-LIO `nav_msgs/Odometry`，以 `diagnostic_msgs/DiagnosticArray` 和 `std_msgs/UInt8` 发布健康状态。所有监控逻辑均由 C++ 实现。

更新日期：2026-10-10。完整材料见[文档总入口](../FAST_LIO/doc/ros1_workspace/README.md)、[启动与操作手册](../FAST_LIO/doc/ros1_workspace/06_启动与操作说明.md)、[真实设备安装与配置](../FAST_LIO/doc/ros1_workspace/09_真实设备安装与配置.md)、[代码与默认阈值](../FAST_LIO/doc/ros1_workspace/04_代码说明.md)及[项目归档清单](../FAST_LIO/doc/ros1_workspace/07_项目完结归档.md)。

系统整体数据流见[FAST-LIO 定位健康监控架构图](../FAST_LIO/doc/ros1_workspace/images/fastlio_health_architecture.svg)。

启动与参数调整可对照[操作速查图](../FAST_LIO/doc/ros1_workspace/images/fastlio_health_quickstart.svg)和[文字版启动手册](../FAST_LIO/doc/ros1_workspace/06_启动与操作说明.md)。

真实设备和 rosbag 回放的操作流程见[完整使用说明图 PNG](../FAST_LIO/doc/ros1_workspace/images/fastlio_health_device_guide.png)、[SVG 原图](../FAST_LIO/doc/ros1_workspace/images/fastlio_health_device_guide.svg)、[真实设备安装与配置手册](../FAST_LIO/doc/ros1_workspace/09_真实设备安装与配置.md)及[回放操作手册](../FAST_LIO/doc/ros1_workspace/06_启动与操作说明.md#2-rosbag-一键回放推荐)。

## 编译和终端环境

```bash
cd $HOME/fastlio_ws
```

当前终端默认是 zsh。ROS Noetic 和工作区 setup 文件为 Bash 格式；在每个新终端中先进入 Bash，再加载环境：

```bash
bash
source /opt/ros/noetic/setup.bash
source $HOME/fastlio_ws/devel/setup.bash
```

如尚未编译，在 Bash 终端执行：

```bash
cd $HOME/fastlio_ws
catkin_make -j2
source devel/setup.bash
```

## 真实 Livox Avia 设备

组合检测台不启动硬件驱动。先按[实机安装与配置手册](../FAST_LIO/doc/ros1_workspace/09_真实设备安装与配置.md)完成依赖、网卡和真实广播码配置，然后在终端 1 启动驱动：

```bash
roslaunch livox_ros_driver livox_lidar_msg.launch bd_list:=""
```

检查驱动已发布 `/livox/lidar` 和 `/livox/imu` 后，在终端 2 启动一键检测台：

```bash
roslaunch fastlio_health_monitor visualization_testbench.launch
```

该 launch 会启动 FAST-LIO、C++ 健康监控、Avia 静态外参 TF 和 RViz；不需要再单独启动 `mapping_avia.launch` 或 `health_monitor.launch`。检测台还会显示 FAST-LIO 当前帧 `/cloud_registered` 和按 1 Hz 更新的累积地图 `/Laser_map`。

RViz 中原始雷达、FAST-LIO 当前帧及累积地图均按点的 intensity 彩虹着色，保留原来的雷达显示方式。三层 `Decay Time` 均为 0：显示最新一份消息，收到下一份才替换，不因等待超时清空。`/Laser_map` 每次发布完整地图，固定 intensity 范围 0–152，避免整图随自动范围变化而变色；该范围可在 RViz 中按设备调整。

## rosbag 回放

回放期间不要启动实体雷达驱动。一个 launch 会同时启动仿真时间、FAST-LIO、健康监控、TF、RViz 和 rosbag 播放器：

```bash
bash
source /opt/ros/noetic/setup.bash
source $HOME/fastlio_ws/devel/setup.bash
roslaunch fastlio_health_monitor visualization_bag_testbench.launch
```

默认使用项目现有 outdoor bag。替换 bag 路径或回放速率：

```bash
roslaunch fastlio_health_monitor visualization_bag_testbench.launch \
  bag:=/path/to/other.bag rate:=1.0
```

播放器会等待 LiDAR 和 IMU 订阅者连接再开始。bag 播放完毕后检测台继续运行，数据超时后变成 ERROR 是预期断流提示；按 `Ctrl+C` 关闭整套 launch。检测台默认关闭 PCD 累积保存；累积 `/Laser_map` 仅在检测台开启且有 RViz 订阅时以 1 Hz 发布。

地图发布队列为 1 并启用 latch，缓存最后一份完整地图。bag 播完后画面可以继续保留建好的地图，健康面板仍按真实输入新鲜度报告 ERROR；保留画面不代表传感器仍在更新。

重新回放同一个 bag 时，先用 `Ctrl+C` 关闭当前整套 launch，再重新运行上述命令；不要只在仍运行的 FAST-LIO 节点上倒回 bag 时间戳。

联调截图和逐项结果：[可视化检测台验证记录](../FAST_LIO/doc/ros1_workspace/02_验证记录.md#5-可视化检测台运行验证)。

Velodyne 或 Ouster 等 `sensor_msgs/PointCloud2` 输入：

```bash
roslaunch fastlio_health_monitor health_monitor.launch \
  config:=$(rospack find fastlio_health_monitor)/config/pointcloud2.yaml
```

默认 topic：`/livox/lidar`、`/livox/imu`、`/Odometry`。参数可在 YAML 中修改。

上述 PointCloud2 命令仅启动监控器；当前组合检测台固定使用 Avia 配置，其他雷达需同时适配 FAST-LIO、输入 topic 与 TF。PointCloud2 分支已实现并编译，本次端到端验证使用 Livox CustomMsg bag，尚未验证实体设备。

## 输出

### IMU 加速度单位（2026-10-10 修复）

私有参数 `imu_acceleration_unit` 仅接受 `g` 或 `mps2`。节点默认及 `config/avia.yaml` 为 `g`，匹配当前 Livox 输入；`config/pointcloud2.yaml` 的通用 IMU 示例为 `mps2`。更换 IMU 时根据驱动/数据集单位配置，单位与雷达类型无自动关联。

g 输入乘 9.80665，mps2 输入保持数值；诊断、面板与 `maximum_acceleration_mps2` 阈值统一采用 m/s²。诊断同时保留 `linear_acceleration_norm_raw`、`imu_acceleration_unit`、`imu_acceleration_scale`，面板显示原始值和换算值。非法单位配置报 FATAL 并退出，角速度保持 rad/s。

修复后监控节点和插件已编译，12 组单位/异常场景、非法单位退出以及完整 1 倍速 bag 回放均通过；287 条诊断与源 IMU 消息逐条配对核对。运行中的旧程序不会自动更新，需结束原 launch 后重新启动。[新运行截图](../FAST_LIO/doc/ros1_workspace/images/imu_units_bag_testbench.png)和[实测记录](../FAST_LIO/doc/ros1_workspace/02_验证记录.md#11-imu-单位修复实测2026-10-10)已归档；早期截图的 IMU 单位标签错误仍作为历史记录保留。

默认 Displays 停靠区可能占用过多高度而截断健康指标；可通过 Panels 隐藏 Displays 或扩大健康面板高度。新截图使用完整显示指标的布局，阈值和输入不变。

### 健康状态

- `/fastlio_health_monitor/diagnostics`：4 项标准诊断，分别为总体、LiDAR、IMU、定位输出；状态值使用 ROS DiagnosticStatus 的 OK/WARN/ERROR。
- `/fastlio_health_monitor/status`：`std_msgs/UInt8`，`0=OK`、`1=WARNING`、`2=ERROR`，latched。

检查内容：LiDAR 到达频率、断流、点数和有效点比例；IMU 到达频率、断流、加速度和角速度的有限性及量级；里程计断流、时间戳、姿态四元数、协方差及相邻位姿速度跳变。阈值均可配置。年龄根据墙钟接收时间计算，因此 rosbag 暂停时也会报告断流。

查看：

```bash
rostopic echo /fastlio_health_monitor/status
rostopic echo /fastlio_health_monitor/diagnostics
```

## 2026-10-14：V2回放生命周期

`visualization_bag_testbench.launch`的空格暂停/恢复通过C++包装器确认并通知监控；暂停显示PAUSED，正常EOF验证后显示STOPPED并保留地图/状态心跳。可调用 `/fastlio_bag_playback/pause`（std_srvs/SetBool）。managed源配置通过playback_state_topic和suspend_output_bindings同步依赖输出，并通过supervised_ros_node检查实际节点。实际断流、异常退出和状态失联仍报错。详见FAST_LIO/doc/ros1_workspace/17_回放暂停恢复与正常结束优化.md。


## 自采RSLidar/MAVROS包内结果检测（2026-10-15）

```bash
source /opt/ros/noetic/setup.bash
source $HOME/fastlio_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:33522
roslaunch -p 33522 fastlio_health_monitor recorded_bag_health.launch
```

默认桌面修复副本、1倍速、C++监控和RViz常驻；参数源为 `config/health_v2_rslidar_mavros.yaml`，调整后重启。可传 `rate:=2.0`、`rviz:=false`、`health_config:=...`；更换YAML的topic需同步 `lidar_topic`、`imu_topic`、`localization_topic` 参数。

沿用C++回放控制器的空格/服务暂停恢复路径，服务 `/recorded_bag_playback/pause`。正常EOF目标为STOPPED并保留界面，Ctrl+C关闭。显示包内已有定位/配准点云，不启动FAST-LIO、不伪造原始雷达外参。原数据约44秒后才有雷达、64秒后才有定位，当前10秒启动期限会产生前段缺数告警。

此新入口已编译，未新增运行验证；之前自动回放12项通过不等于本入口已实测。EOF确认前瞬时接收告警的限制仍保留。详见工作区 `FAST_LIO/doc/ros1_workspace/19_自采Bag副本修复与原生回放测试.md` 第8节。


### 2026-10-15：回放日志新格式与封存

`recorded_bag_health.launch` 默认 `finish_log_on_playback_complete:=true`：确认EOF后封存健康/回放JSONL与CSV，界面和心跳继续保留。需要持续记录可传false；下一包重启创建新run。暂停或故障不按正常EOF结束日志。

当前JSONL与CSV的时间只保留 `bag_elapsed_sec`、`source_stamp_sec`、`steady_elapsed_sec`。分别来自/clock减bag起点、消息头、本进程单调计时；无bag时钟或消息关联时对应值为null（CSV为空）。接收年龄/间隔等检测指标继续保留，历史日志不修改。

面板显示“日志记录：已结束”，正常封存不等于日志故障。完整回放实测和分析见工作区文档 `21_三时间字段回放实测与日志分析.md`；EOF确认前瞬时接收告警仍为已知边界。
