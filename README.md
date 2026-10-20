# Robot Data Health Monitoring Module

ROS 1 Noetic / C++14 工程，包含 FAST-LIO、Livox ROS Driver 和可独立接入的机器人健康检测 ROS 包。健康检测判断、数据适配和日志处理由 C++ 实现；少量 Python/Bash 文件只负责构建材料生成、启动模板或测试编排。

## 仓库内容

- FAST_LIO：FAST-LIO 激光惯性里程计与建图源码及工程文档。
- livox_ros_driver：Livox ROS 驱动源码。
- fastlio_health_monitor：健康检测模块。支持雷达、IMU、姿态、定位、可选测距及其他 ROS 模块的适配；提供诊断 Topic、JSONL/CSV 日志和可选 RViz 面板。
- docs/代码目录树与文件说明.md：健康检测包逐文件目录树、代码职责、配置和数据流说明。

## 环境和编译

适用环境：Ubuntu 20.04、ROS Noetic、Catkin、C++14。下面假设工作区在 ~/fastlio_ws：

~~~bash
sudo apt update
sudo apt install -y git build-essential cmake python3-rosdep \
  libyaml-cpp-dev libeigen3-dev libboost-filesystem-dev \
  libboost-system-dev libssl-dev qtbase5-dev

# rosdep 只需在本机初始化一次
sudo rosdep init
rosdep update
source /opt/ros/noetic/setup.bash

mkdir -p ~/fastlio_ws/src
cd ~/fastlio_ws/src
git clone https://github.com/spacethroughing/Robot-Data-Health-Monitoring-Module.git

# 如果 src/CMakeLists.txt 不存在，再初始化 catkin workspace
test -e CMakeLists.txt || catkin_init_workspace

cd ~/fastlio_ws
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro noetic -r -y
catkin_make -j2
source devel/setup.bash
~~~

也可以使用仓库内的安装脚本，在已安装 ROS Noetic 的 Ubuntu 20.04 上安装依赖并把健康检测包编译到一个新工作区：

~~~bash
# gui 会安装 RViz；无图形机器人用 headless
bash scripts/install_dependencies.sh gui
bash scripts/install_workspace.sh --workspace "$HOME/robot_health_ws" --gui

# 无图形版本：
# bash scripts/install_dependencies.sh headless
# bash scripts/install_workspace.sh --workspace "$HOME/robot_health_ws_headless" --headless
~~~

`install_workspace.sh` 只复制 `fastlio_health_monitor` 包，不覆盖目标工作区中已有的同名源码。仓库根目录和桌面部署包两种目录结构均可作为脚本来源。

Livox 驱动还需要系统中安装 Livox-SDK。按 livox_ros_driver/README_CN.md 安装 SDK 和驱动依赖；使用标准 PointCloud2 输入时，健康检测本身不要求编译 Livox CustomMsg 适配器。

每个新终端先加载环境：

~~~bash
source /opt/ros/noetic/setup.bash
source ~/fastlio_ws/devel/setup.bash
~~~

无图形机器人可在构建时关闭 RViz 插件：

~~~bash
cd ~/fastlio_ws
catkin_make -j2 -DBUILD_HEALTH_RVIZ_PANEL=OFF
~~~

## 配置文件

配置在 fastlio_health_monitor/config/：

| 文件 | 用途 |
| --- | --- |
| health_robot.yaml | 实机实时数据配置。修改传感器/定位 Topic、启用模块和健康阈值。 |
| health_bag.yaml | rosbag 回放配置。与实机配置分开，便于设置回放 Topic 和回放检测规则。 |
| runtime_localization.example.yaml | 由监控器托管定位进程时的示例；部署前要填写实际启动命令与 ROS 节点名称。 |

规则主要通过 enabled: true/false 开关。各字段含义、默认值和数据流请看对应 YAML 注释及 docs/代码目录树与文件说明.md。

## 启动

### 实机实时监控

传感器驱动和定位节点已在 ROS 中发布数据时：

~~~bash
roslaunch fastlio_health_monitor live.launch
~~~

无图形运行：

~~~bash
roslaunch fastlio_health_monitor live.launch rviz:=false
~~~

live.launch 只接收现有 Topic，不会启动或伪造传感器、FAST-LIO 或其他定位节点。接入 Topic 与规则由 health_robot.yaml 决定。

### rosbag 回放监控

~~~bash
roslaunch fastlio_health_monitor replay.launch \
  bag:=/完整路径/recording.bag rate:=1.0 rviz:=true
~~~

无图形回放：

~~~bash
roslaunch fastlio_health_monitor replay.launch \
  bag:=/完整路径/recording.bag rate:=1.0 rviz:=false
~~~

replay.launch 使用 health_bag.yaml，等待监控订阅者就绪后再开始回放；正常播到文件末尾会通知健康监控封存本次日志。该入口回放 bag 中已录制的 Topic，不会自动启动 FAST-LIO 重算定位。原 bag 以只读方式打开。

### 只启动监控或界面

~~~bash
roslaunch fastlio_health_monitor monitor.launch
roslaunch fastlio_health_monitor rviz.launch
~~~

前者只启动 C++ 健康检测节点，后者只打开 RViz 显示布局。分终端运行时，先确认 ROS master、输入数据和配置路径已就绪。

## 输出

- /fastlio_health_monitor/diagnostics：diagnostic_msgs/DiagnosticArray，包含模块级状态和规则详情。
- /fastlio_health_monitor/status：std_msgs/UInt8，0 表示 OK、1 表示 WARNING、2 表示 ERROR。
- 健康和回放日志：默认写入 ~/.ros/robot_health 下按运行创建的目录；最终位置受 YAML 和 launch 参数控制。

RViz 面板展示诊断消息；无界面机器人可用 rostopic echo、rostopic hz 或读取 JSONL/CSV 日志查看状态。

## 文档

- [健康监控包完整代码目录树与文件说明](docs/代码目录树与文件说明.md)
- [ROS 包使用说明](fastlio_health_monitor/README.md)
- [FAST-LIO 与 ROS1 工程文档](FAST_LIO/doc/ros1_workspace/README.md)
- [真实机器人安装与配置文档](FAST_LIO/doc/ros1_workspace/09_真实设备安装与配置.md)
