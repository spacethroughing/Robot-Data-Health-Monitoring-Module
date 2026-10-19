/**
 * RViz健康面板的插件接口。
 *
 * 数据流：/fastlio_health_monitor/diagnostics -> callback解析标准诊断消息
 * -> 按模块ID缓存最近状态 -> Qt槽函数刷新卡片。面板只负责展示健康节点
 * 已经计算出的状态，不重复执行传感器阈值判断。QTimer检查的是“诊断消息多久
 * 没更新”，因此面板通信超时与诊断消息中报告的传感器异常是两类状态。
 *
 * ROS回调可能发生在ROS回调线程；界面更新通过Qt槽函数排入GUI线程，避免从
 * ROS线程直接操作QLabel等界面对象。topic_允许用户更改订阅的话题。
 */
#pragma once
#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros/ros.h>
#include <rviz/panel.h>

#include <QLabel>
#include <QLineEdit>
#include <QMap>
#include <QTimer>
#include <QVBoxLayout>
namespace fastlio_health_monitor {
/**
 * RViz插件类：为每个模块显示健康等级、诊断详情和生命周期状态。
 * RViz负责创建对象并调用onInitialize()；配置文件中的Topic在初始化或用户
 * 更改topic_后订阅。面板可以脱离健康节点单独运行，此时超时状态提示诊断源无更新。
 */
class HealthPanel : public rviz::Panel {
  Q_OBJECT
 public:
  // Qt/RViz创建插件时使用；parent由RViz持有，本类不自行删除父对象。
  explicit HealthPanel(QWidget* parent = nullptr);
  // RViz完成插件构造后调用；在此创建控件、定时器并建立初始诊断订阅。
  void onInitialize() override;
  // 保存用户修改的Topic等面板设置到RViz配置文件。
  void save(rviz::Config config) const override;
  // 从RViz配置恢复面板设置；界面参数恢复后需要更新订阅。
  void load(const rviz::Config& config) override;
 private Q_SLOTS:
  // 读取topic_文本并重建DiagnosticArray订阅。
  void subscribeTopic();
  // GUI线程中的单模块刷新入口；更新等级徽标、说明文字和运行状态。
  void updateModule(const QString& id, int level, const QString& details,
                    const QString& state);
  // 定时查找长时间未收到诊断消息的模块，显示面板通信超时。
  void checkTimeout();

 private:
  // 单模块的界面缓存；received记录最近诊断到达的单调时间，用于显示通信超时。
  struct Row {
    QLabel* badge = nullptr;
    QLabel* details = nullptr;
    double received = 0;
  };
  // ROS订阅回调：解析DiagnosticArray并把每个模块更新排队到Qt GUI线程。
  void callback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg);
  // 按健康等级和运行状态设置徽标颜色/样式；只映射显示样式，不改变诊断结论。
  void style(QLabel* label, int level, const QString& state);
  // ROS订阅所需的句柄与订阅对象；sub_重置时会停止旧Topic订阅。
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  // 面板布局控件；topic_保存诊断Topic，rows_容纳动态生成的模块卡片。
  QLineEdit* topic_ = nullptr;
  QVBoxLayout* rows_ = nullptr;
  // 以模块ID查找卡片和最近状态，避免每次诊断到来时重建整个界面。
  QMap<QString, Row> modules_;
  // GUI定时器周期性执行checkTimeout()，用于发现诊断消息本身的接收中断。
  QTimer timeout_;
};
}  // namespace fastlio_health_monitor
