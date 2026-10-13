#pragma once
#include <fastlio_health_monitor/health_core.h>
#include <ros/ros.h>
namespace robot_health {
class IInputAdapter {
 public:
  using SampleSink = std::function<void(const Sample&)>;
  virtual ~IInputAdapter() = default;
  virtual void reconnect() = 0;
  virtual void poll(double now) = 0;
};
class AdapterFactory {
 public:
  static std::unique_ptr<IInputAdapter> create(ros::NodeHandle& nh,
                                               const ModuleConfig& config,
                                               IInputAdapter::SampleSink sink);
};
}  // namespace robot_health
