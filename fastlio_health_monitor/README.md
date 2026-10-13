# fastlio_health_monitor V2

这是 ROS1/C++14 V2 健康监控包，已从旧版监控节点整理为 V2-only 入口。安装、配置、编译、启动、故障 bag 回放和日志查询步骤请看仓库根目录 [README](../README.md)。

关键文件：

- `src/health_core.cpp`：C++ 健康规则与状态汇总核心。
- `src/input_adapters.cpp`：ROS 消息到统一样本结构的适配器。
- `src/robot_health_monitor_node.cpp`：ROS 入口和 diagnostics 发布。
- `src/event_recorder.cpp`：JSONL 事件与 CSV 指标记录。
- `src/runtime_controller_node.cpp`：managed 生命周期及进程管理。
- `config/`、`launch/`、`rviz/`：V2 参数、启动入口和可视化配置。
