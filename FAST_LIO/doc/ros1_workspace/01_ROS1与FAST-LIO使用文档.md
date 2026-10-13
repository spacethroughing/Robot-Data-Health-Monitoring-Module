# ROS1 与 FAST-LIO2 使用文档

更新日期：2026-10-09（Asia/Shanghai）。完整的一键回放、设备启动和故障定位流程见 [06_启动与操作说明](06_启动与操作说明.md)；本文保留 ROS1 工程学习与基础操作。

## 1. 环境和工程

- ROS 发行版：Noetic。
- 工作区：`$HOME/fastlio_ws`，Catkin workspace。
- FAST-LIO2 包：`src/FAST_LIO`，上游地址 `https://github.com/hku-mars/FAST_LIO.git`。
- Livox 驱动：`src/livox_ros_driver`。
- 定位健康监控：`src/fastlio_health_monitor`。
- 现成测试数据：`$HOME/datasets/fastlio/outdoor_Mainbuilding_10hz_2020-12-24-16-38-00.bag`。

当前工程已经存在 Git checkout，因此本次直接在已有代码上开发，没有重复 clone。查询上游来源：

```bash
git -C $HOME/fastlio_ws/src/FAST_LIO remote -v
```

## 2. 编译工作区

当前默认终端为 zsh，先单独输入 `bash` 并按回车，进入 Bash 后再逐行执行下面的命令。

```bash
source /opt/ros/noetic/setup.bash
cd $HOME/fastlio_ws
catkin_make -j2
source devel/setup.bash
```

Catkin 会依赖顺序构建 `livox_ros_driver`、`fast_lio` 和 `fastlio_health_monitor`。新增功能包源码位于 `src/fastlio_health_monitor/src/health_monitor_node.cpp`。

## 3. launch、参数与 YAML

FAST-LIO 的 `launch/mapping_avia.launch` 通过 `rosparam` 加载 `config/avia.yaml`。YAML 中 `common/lid_topic`、`common/imu_topic` 等键进入 ROS 参数服务器。launch 中的 `<node>` 启动定位可执行程序；`rviz:=false` 可关闭 RViz。

启动 FAST-LIO：

```bash
roslaunch fast_lio mapping_avia.launch rviz:=false
```

监控包的 `launch/health_monitor.launch` 在节点私有命名空间加载 YAML。Avia 默认配置为 `config/avia.yaml`：

```bash
roslaunch fastlio_health_monitor health_monitor.launch
```

PointCloud2 雷达使用对应配置：

```bash
roslaunch fastlio_health_monitor health_monitor.launch \
  config:=$(rospack find fastlio_health_monitor)/config/pointcloud2.yaml
```

常用 ROS1 命令：

```bash
rosnode list
rostopic list
rostopic info /livox/lidar
rostopic echo -n 1 /Odometry
rosrun tf tf_echo camera_init body
rosparam get /common/lid_topic
```

`tf_echo` 需要等 FAST-LIO 收到足够的雷达和 IMU 数据后才能解析 `camera_init -> body`。

## 4. 测试 bag

查看 bag 元信息：

```bash
rosbag info $HOME/datasets/fastlio/outdoor_Mainbuilding_10hz_2020-12-24-16-38-00.bag
```

该 bag 包含 `/livox/lidar`（`livox_ros_driver/CustomMsg`）和 `/livox/imu`（`sensor_msgs/Imu`），与 Avia YAML 默认输入匹配。日常回放推荐直接使用一键入口：

```bash
roslaunch fastlio_health_monitor visualization_bag_testbench.launch
```

手动分步启动需在不同终端分别运行检测台和播放器，详见[启动手册第 4 节](06_启动与操作说明.md#4-手动分步回放)。`roslaunch` 与 `rosbag play` 都会占用前台终端，不能把下一条启动命令当作前一个程序的交互输入。

## 5. 工作流

1. 先确认 `/opt/ros/noetic/setup.bash` 和工作区 `devel/setup.bash` 已 source。
2. 用 `roslaunch` 启动 FAST-LIO 和监控器。
3. 用 `rosnode list`、`rostopic list` 检查 ROS graph。
4. 用 `rostopic echo` 检查 Odometry 和健康诊断。
5. 用 `tf_echo camera_init body` 检查 TF。
6. 每个技术点完成后，立即在[工作进度](工作进度.md)、[验证记录](02_验证记录.md)和[问题复盘](03_问题复盘.md)更新结果；每日工作结束前追加日志。

## 6. 可视化检测台

### 每个终端加载 ROS 环境

当前终端默认使用 zsh。ROS Noetic 和工作区生成的是 Bash setup 文件，建议在每个新终端进入 Bash 后加载环境：

```bash
bash
source /opt/ros/noetic/setup.bash
source $HOME/fastlio_ws/devel/setup.bash
```

若工作区还未编译，在 Bash 终端执行：

```bash
cd $HOME/fastlio_ws
catkin_make -j2
source devel/setup.bash
```

### 真实 Livox Avia 设备

组合检测台会启动 FAST-LIO、健康监控、静态外参 TF 和 RViz；它不会代替 Livox 硬件驱动。先确认驱动已配置本机雷达序列号并启动驱动：

```bash
# 终端 1：按实际设备配置 livox_lidar_config.json
roslaunch livox_ros_driver livox_lidar_msg.launch
```

检查驱动输入后，在另一个已加载 ROS 环境的终端启动检测台：

```bash
# 终端 2
roslaunch fastlio_health_monitor visualization_testbench.launch
```

驱动应发布 `/livox/lidar`（`livox_ros_driver/CustomMsg`）和 `/livox/imu`（`sensor_msgs/Imu`）。检查命令：

```bash
rostopic type /livox/lidar
rostopic type /livox/imu
rostopic hz /livox/lidar
```

RViz 左侧显示总体、LiDAR、IMU 和 Odometry 的 OK/WARNING/ERROR 状态及指标；右侧显示原始点云、FAST-LIO 当前帧注册点云、累积地图 `/Laser_map`、定位输出、轨迹和 TF。组合 launch 已包含定位及监控节点，不要再单独启动 `mapping_avia.launch` 或 `health_monitor.launch`。

原始雷达、FAST-LIO 当前帧和累积地图均采用原来的 intensity 彩虹着色。三层 `Decay Time=0` 表示保持最新一份消息，下一份到达才替换；累积地图固定 intensity 范围 0–152 防止全图反复变色。bag 播完后地图仍可保留，但监控会因输入断流转为 ERROR。每次重播 bag 前先结束上一套 launch，再重新启动，避免 bag 时间戳回退后 FAST-LIO 未从头初始化。

### 使用 rosbag 回放

回放时不要同时启动真实 Livox 驱动。现在可以用一个 launch 同时启动仿真时间、FAST-LIO、健康监控、TF、RViz 和 rosbag 播放器：

```bash
roslaunch fastlio_health_monitor visualization_bag_testbench.launch
```

默认回放现有 outdoor bag，播放器会等 LiDAR 和 IMU 订阅者连接后再开始。若要指定其他 bag 或回放速率：

```bash
roslaunch fastlio_health_monitor visualization_bag_testbench.launch \
  bag:=/path/to/other.bag rate:=1.0
```

状态和 TF 可在另一个已加载环境的终端检查：

```bash
rostopic echo -n 1 /fastlio_health_monitor/status
rostopic echo -n 1 /fastlio_health_monitor/diagnostics
rosrun tf tf_echo camera_init livox_raw
```

状态码为 `0=OK`、`1=WARNING`、`2=ERROR`。回放结束后 rosbag 节点会退出，检测台继续运行；输入超过新鲜度阈值后显示 ERROR 属于预期断流提示。按 `Ctrl+C` 关闭整套 launch。最新运行截图和地图频闪对照见[验证记录第 8 节](02_验证记录.md#8-地图频闪修复与原始雷达配色恢复)。

外参 TF 使用与 `config/avia.yaml` 的 `extrinsic_T` 和单位阵 `extrinsic_R` 相匹配的固定值发布到 `/tf_static`，不是自动读取 YAML。静态变换不受 rosbag 时钟与传感器 header 时间基准差异影响。若修改 Avia 外参，也要同步修改 `visualization_testbench.launch` 的 `body_to_livox_raw` 参数。
