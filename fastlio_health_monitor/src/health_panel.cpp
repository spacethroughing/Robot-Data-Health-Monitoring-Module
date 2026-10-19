/**
 * 显示数据流：DiagnosticArray -> 按module_id缓存 -> Qt卡片更新。
 * NO_DATA/PAUSED/STOPPED与健康等级分开显示，所有诊断阈值以C++核心输出为准。
 */
#include <fastlio_health_monitor/health_panel.h>
#include <pluginlib/class_list_macros.h>
#include <rviz/config.h>

#include <QFont>
#include <QGroupBox>
#include <QMetaObject>
#include <QScrollArea>
#include <chrono>
#include <cmath>
namespace fastlio_health_monitor {
namespace {
// 面板本地单调时间；用于检测诊断通信中断，不参与传感器健康判定。
double now() {
  return std::chrono::duration<double>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}
// 按KeyValue键读取诊断值；不依赖模块排列位置，缺失时返回说明用默认文字。
QString value(const diagnostic_msgs::DiagnosticStatus& s,
              const std::string& name, const QString& fallback = "N/A") {
  for (const auto& v : s.values)
    if (v.key == name) return QString::fromStdString(v.value);
  return fallback;
}
// 为诊断数值选择易读的小数/科学记数格式；保留N/A等状态文本。
QString quantity(const diagnostic_msgs::DiagnosticStatus& s, const char* key) {
  QString raw = value(s, key);
  bool ok = false;
  double v = raw.toDouble(&ok);
  if (!ok) return raw;
  return std::abs(v) > 0 && std::abs(v) < .001 ? QString::number(v, 'g', 3)
                                               : QString::number(v, 'f', 3);
}
// 把现成诊断值组织为中文说明：输入、频率、接收年龄、姿态、测距、日志状态。
QString details(const diagnostic_msgs::DiagnosticStatus& s) {
  QString text = QString::fromStdString(s.message);
  if (value(s, "missing_data_policy", "") == "no_data_until_first_sample" &&
      value(s, "data_presence", "") == "NO_DATA" && int(s.level) < 2)
    text = "无数据：尚未收到此可选输入，等待数据接入";
  const QString topic = value(s, "topic", "");
  if (!topic.isEmpty()) text += "\n输入：" + topic;
  auto add = [&](const char* key, const char* label, const char* unit) {
    QString v = value(s, key, "");
    if (!v.isEmpty()) text += QString("\n%1：%2 %3").arg(label, v, unit);
  };
  if (!value(s, "age_sec", "").isEmpty())
    text += QString("\n频率 %1 Hz | 接收年龄 %2 s | 有效 %3")
                .arg(quantity(s, "rate_hz"), quantity(s, "age_sec"),
                     value(s, "operational_valid"));
  if (!value(s, "points_total", "").isEmpty())
    text += QString("\n总点数 %1 / 有效 %2 / 比例 %3")
                .arg(value(s, "points_total"), value(s, "points_valid"),
                     quantity(s, "valid_ratio"));
  if (!value(s, "imu_acceleration_unit", "").isEmpty())
    text += QString("\n加速度 %1 m/s²（原始 %2 %3）\n角速度 %4 rad/s")
                .arg(quantity(s, "linear_acceleration_norm_mps2"),
                     quantity(s, "linear_acceleration_norm_raw"),
                     value(s, "imu_acceleration_unit"),
                     quantity(s, "angular_velocity_norm_rps"));
  if (!value(s, "position_x_m", "").isEmpty())
    text +=
        QString(
            "\n位置 [%1, %2, %3] m\n速度 %4 m/s / 加速度 %5 m/s²\n位置方差 "
            "%6 m² / 姿态方差 %7 rad²")
            .arg(quantity(s, "position_x_m"), quantity(s, "position_y_m"),
                 quantity(s, "position_z_m"), quantity(s, "linear_speed_mps"),
                 quantity(s, "linear_acceleration_mps2"),
                 quantity(s, "position_variance_m2"),
                 quantity(s, "orientation_variance_rad2"));
  add("covariance_status", "协方差覆盖", "");
  add("roll_deg", "横滚角 roll", "°");
  add("pitch_deg", "俯仰角 pitch", "°");
  add("attitude_status", "姿态数据", "");
  if (!value(s, "range_raw_m", "").isEmpty()) {
    text += QString("\n对地测距 %1 m | 测距状态 %2\n测距变化率 %3 m/s")
                .arg(value(s, "range_raw_m"), value(s, "range_status"),
                     quantity(s, "range_rate_mps"));
    text += "\n测距沿传感器测量方向；未作垂直高度标定";
  }

  add("goal_id", "规划目标", "");
  add("task_state", "任务状态", "");
  add("module_epoch", "数据代际", "");
  add("control_reason", "运行状态原因", "");
  add("optional_no_data_count", "暂无数据的可选模块数", "");

  QString logs_normal = value(s, "logs_normal", "");
  // Older V2 publishers expose only the enabled/incomplete flags.
  if (logs_normal.isEmpty()) {
    const QString enabled = value(s, "logs_enabled", "");
    const QString incomplete = value(s, "logs_incomplete", "");
    if (enabled == "false" || incomplete == "true")
      logs_normal = "false";
    else if (enabled == "true" && incomplete == "false")
      logs_normal = "true";
    else if (!enabled.isEmpty() || !incomplete.isEmpty())
      logs_normal = "N/A";
  }
  if (!logs_normal.isEmpty())
    text += "\n日志正常：" +
            (logs_normal == "true"    ? QString("True")
             : logs_normal == "false" ? QString("False")
                                      : QString("N/A"));
  const QString recording_state = value(s, "log_recording_state", "");
  if (!recording_state.isEmpty())
    text += "\n日志记录：" +
            (recording_state == "FINISHED" ? QString("已结束")
             : recording_state == "RECORDING" ? QString("记录中")
             : recording_state == "FINISHING" ? QString("正在封存")
             : recording_state == "DISABLED" ? QString("未启用")
                                              : QString("写入异常"));
  return text;
}
}  // namespace
// 创建RViz中文诊断面板与滚动模块卡片，界面只展示诊断结果，不另算健康阈值。
HealthPanel::HealthPanel(QWidget* p) : rviz::Panel(p) {
  auto* root = new QVBoxLayout(this);
  auto* title = new QLabel("机器人健康检测台 V2");
  QFont font = title->font();
  font.setBold(true);
  title->setFont(font);
  root->addWidget(title);
  topic_ = new QLineEdit("/fastlio_health_monitor/diagnostics");
  topic_->setPlaceholderText("诊断 Topic，回车切换");
  root->addWidget(topic_);
  connect(topic_, &QLineEdit::returnPressed, this,
          &HealthPanel::subscribeTopic);
  auto* scroll = new QScrollArea;
  scroll->setWidgetResizable(true);
  scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  setMinimumWidth(480);
  setMinimumHeight(560);
  auto* content = new QWidget;
  rows_ = new QVBoxLayout(content);
  rows_->setAlignment(Qt::AlignTop);
  scroll->setWidget(content);
  root->addWidget(scroll);
  timeout_.setInterval(250);
  connect(&timeout_, &QTimer::timeout, this, &HealthPanel::checkTimeout);
  timeout_.start();
}
// RViz插件初始化后接入诊断Topic，避免在节点句柄尚未可用时订阅。
void HealthPanel::onInitialize() { subscribeTopic(); }
// 保存诊断Topic到RViz布局，使下次打开仍订阅同一输出。
void HealthPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
  config.mapSetValue("DiagnosticsTopic", topic_->text());
}
// 从RViz布局恢复诊断Topic，并刷新订阅。
void HealthPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
  QString t;
  if (config.mapGetString("DiagnosticsTopic", &t)) {
    topic_->setText(t);
    subscribeTopic();
  }
}
// 重新订阅诊断流并清理旧接收时间；切换Topic不沿用旧模块的新鲜度。
void HealthPanel::subscribeTopic() {
  sub_.shutdown();
  for (auto it = modules_.begin(); it != modules_.end(); ++it) it->received = 0;
  try {
    sub_ = nh_.subscribe(topic_->text().toStdString(), 10,
                         &HealthPanel::callback, this);
  } catch (const ros::InvalidNameException& e) {
    updateModule("overall", 2, QString::fromUtf8(e.what()), "ERROR");
  }
  Q_EMIT configChanged();
}
// ROS诊断回调缓存最新模块状态，再通过Qt线程安全机制更新显示。
void HealthPanel::callback(
    const diagnostic_msgs::DiagnosticArray::ConstPtr& msg) {
  for (const auto& s : msg->status) {
    QString id = value(s, "module_id", QString::fromStdString(s.hardware_id));
    if (s.name == "FAST-LIO health")
      id = "overall";
    else if (s.name == "FAST-LIO localization output")
      id = "localization";
    else if (id.isEmpty())
      id = QString::fromStdString(s.name);
    QString state = value(s, "operating_state", "");
    QMetaObject::invokeMethod(this, "updateModule", Qt::QueuedConnection,
                              Q_ARG(QString, id), Q_ARG(int, int(s.level)),
                              Q_ARG(QString, details(s)),
                              Q_ARG(QString, state));
  }
}
// 把等级和生命周期映射为颜色/文字；NO_DATA灰色，不显示假正常。
void HealthPanel::style(QLabel* b, int level, const QString& state) {
  bool inactive = state == "STOPPED" || state == "STARTING" ||
                  state == "STOPPING" || state == "PAUSED" || state == "NO_DATA";
  QString color = level >= 2   ? "#b42332"
                  : inactive   ? "#59636e"
                  : level == 1 ? "#a15c00"
                               : "#167a45";
  QString text = level >= 2   ? "ERROR"
                 : state == "NO_DATA" ? QString("无数据")
                 : inactive   ? state
                 : level == 1 ? "WARNING"
                              : "OK";
  if (!state.isEmpty() && !inactive) text += " / " + state;
  b->setText(text);
  b->setStyleSheet(
      QString("QLabel { color:%1; background:#edf1f5; border:1px solid %1; "
              "border-radius:4px; padding:5px; font-weight:bold; }")
          .arg(color));
}
// 更新某个模块卡片的状态、数值与原因；使用诊断键而非固定模块索引。
void HealthPanel::updateModule(const QString& id, int level,
                               const QString& text, const QString& state) {
  if (!modules_.contains(id)) {
    auto* g = new QGroupBox(id == "overall" ? "总体健康" : id);
    auto* layout = new QVBoxLayout(g);
    Row r;
    r.badge = new QLabel("等待");
    r.details = new QLabel;
    r.details->setTextFormat(Qt::PlainText);
    r.details->setWordWrap(true);
    r.details->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    r.badge->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
    r.details->setTextInteractionFlags(Qt::TextSelectableByMouse);
    layout->addWidget(r.badge);
    layout->addWidget(r.details);
    rows_->addWidget(g);
    modules_[id] = r;
  }
  auto& r = modules_[id];
  r.received = now();
  style(r.badge, level, state);
  r.details->setText(text);
}
// 检测诊断流自身的接收超时；监控程序失联时面板不能继续显示旧的健康状态。
void HealthPanel::checkTimeout() {
  for (auto it = modules_.begin(); it != modules_.end(); ++it)
    if (it->received == 0 || now() - it->received > 2) {
      style(it->badge, 2, "STALE");
      if (!it->details->text().startsWith("诊断失联"))
        it->details->setText("诊断失联：旧状态已失效\n" + it->details->text());
    }
}
}  // namespace fastlio_health_monitor
PLUGINLIB_EXPORT_CLASS(fastlio_health_monitor::HealthPanel, rviz::Panel)
