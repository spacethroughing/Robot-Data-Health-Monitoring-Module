# fastlio_health_monitor V2 可选规则版（0.4.0）

功能模块为C++。配置、源文件、函数、ROS接口、launch和RViz均有中文说明。

## 入口

- 实机：`roslaunch fastlio_health_monitor live.launch rviz:=false`，默认config/health_robot.yaml。
- 回放：`roslaunch fastlio_health_monitor replay.launch bag:=/完整路径/文件.bag`，默认config/health_bag.yaml。
- 只运行监控：`roslaunch fastlio_health_monitor monitor.launch`。
- 只打开界面：`roslaunch fastlio_health_monitor rviz.launch`。

先source ROS和正确工作区；完整安装、分终端及跨电脑使用方式见交付目录的真实安装使用指南。

## 代码顺序

1. health_core.h定义统一Sample、Finding、ModuleReport及类接口。
2. ConfigLoader加载YAML；不存在的业务规则关闭，运行参数采用注明默认值。
3. input_adapters.cpp把ROS消息转换为Sample并换算单位。
4. StreamTracker.receive检查源时间戳，SteadyTimer调用evaluate检查接收年龄与频率。
5. 各IHealthChecker实现质量检查；RuleLatch处理逐条触发/恢复。
6. LifecycleTracker识别明确的正常暂停/停止/重启；ModuleMonitor形成报告。
7. robot_health_monitor_node.cpp沿依赖汇总并发布DiagnosticArray/UInt8。
8. EventRecorder异步保存精简JSONL/CSV，正常EOF后封存；HealthPanel只显示诊断。

完整文件树及每个源码、头文件、ROS接口、launch、配置和RViz布局的作用见交付包的[代码目录树与文件说明](../docs/代码目录树与文件说明.md)。

## 规则开关

主要使用 `enabled: true/false`；保留检测规则和阈值，关闭只改false，恢复只改true。每个模块和支持开关的检测规则都已明确写出enabled。Topic、单位、参考系和发布频率是运行参数；原生布尔设置本身用true/false。 主配置仅health_robot.yaml、health_bag.yaml、runtime_localization.example.yaml；三模块通过关闭imu_attitude/ground_range选择。

默认构建标准ROS适配器。BUILD_HEALTH_RVIZ_PANEL=OFF用于无图形机器人；已有livox_ros_driver且需要CustomMsg时BUILD_HEALTH_LIVOX_ADAPTER=ON。内部实例/代际消息保留，日志不再输出指定的五个字段。

## 输出

- /fastlio_health_monitor/diagnostics：diagnostic_msgs/DiagnosticArray，完整诊断。
- /fastlio_health_monitor/status：std_msgs/UInt8，0/1/2。
- 实机日志默认~/.ros/robot_health/health，每次新run；回放器/控制器日志独立。

源码不启动雷达/MAVROS，不代替定位算法，也不直接控制飞行。
