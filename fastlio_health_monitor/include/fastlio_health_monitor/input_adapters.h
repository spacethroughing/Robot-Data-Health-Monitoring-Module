/**
 * ROS消息接入层的公共接口。
 *
 * 数据流：ROS Topic消息 -> 具体适配器解码/校验/单位归一 -> Sample -> sink回调
 * -> ModuleMonitor。这里负责“把消息翻译成统一数据”，不负责判断整条数据流
 * 是否断开，也不负责应用健康阈值；这些工作由核心跟踪器和Checker完成。
 *
 * 每种消息格式对应一个IInputAdapter实现。新增驱动或消息类型时，在
 * AdapterFactory中注册实现，核心健康规则仍只依赖Sample，不需要包含驱动头文件。
 */
#pragma once
#include <fastlio_health_monitor/health_core.h>
#include <ros/ros.h>
namespace robot_health {
/**
 * 所有ROS输入适配器的共同接口。
 * 适配器在创建时建立Topic订阅；收到消息后构造Sample并同步交给SampleSink。
 * Sensor适配器通常一直等待Topic数据；需要按任务状态检查的规划适配器可在
 * poll()中检查当前是否应有输出。生命周期或上游实例变化时调用reconnect()，
 * 让具体实现关闭旧订阅并建立新订阅，避免沿用旧连接中的缓存消息。
 */
class IInputAdapter {
 public:
  // 样本出口。调用方通常把该回调连接到对应ModuleMonitor::receive()。
  using SampleSink = std::function<void(const Sample&)>;
  // 多态销毁入口；保证通过基类指针释放具体ROS适配器时正确析构。
  virtual ~IInputAdapter() = default;
  // 重建订阅或重置适配器内部连接状态；生命周期切换后由节点调用。
  virtual void reconnect() = 0;
  // 定时轮询接口。普通传感器适配器无需处理；任务型适配器可检查输出时限。
  virtual void poll(double now) = 0;
};
/**
 * 适配器工厂：把YAML里的adapter名称映射到C++订阅实现。
 * 输入包括ROS句柄、已解析的模块配置和样本回调；返回对象负责订阅配置中的Topic。
 * 未注册名称或构建时未启用的可选驱动适配器会抛出异常，调用方应将其作为启动配置错误处理。
 */
class AdapterFactory {
 public:
  // 创建一个与模块配置相匹配的订阅器；适配器存活期间保持订阅并向sink输出样本。
  static std::unique_ptr<IInputAdapter> create(ros::NodeHandle& nh,
                                               const ModuleConfig& config,
                                               IInputAdapter::SampleSink sink);
};
}  // namespace robot_health
