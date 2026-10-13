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
class HealthPanel : public rviz::Panel {
  Q_OBJECT
 public:
  explicit HealthPanel(QWidget* parent = nullptr);
  void onInitialize() override;
  void save(rviz::Config config) const override;
  void load(const rviz::Config& config) override;
 private Q_SLOTS:
  void subscribeTopic();
  void updateModule(const QString& id, int level, const QString& details,
                    const QString& state);
  void checkTimeout();

 private:
  struct Row {
    QLabel* badge = nullptr;
    QLabel* details = nullptr;
    double received = 0;
  };
  void callback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg);
  void style(QLabel* label, int level, const QString& state);
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  QLineEdit* topic_ = nullptr;
  QVBoxLayout* rows_ = nullptr;
  QMap<QString, Row> modules_;
  QTimer timeout_;
};
}  // namespace fastlio_health_monitor
