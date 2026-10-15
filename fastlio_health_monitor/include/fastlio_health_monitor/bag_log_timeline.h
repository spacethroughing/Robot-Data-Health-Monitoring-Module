#pragma once

#include <fastlio_health_monitor/event_recorder.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace robot_health {
// Metadata only, always open read-only. A missing/unindexed bag leaves replay
// timestamps unavailable; failures must still be recordable in wall/steady time.
inline LogTimeline bagLogTimeline(const std::string& path) {
  LogTimeline timeline;
  if (path.empty() || !ros::Time::isSimTime()) return timeline;
  try {
    rosbag::Bag bag(path, rosbag::bagmode::Read);
    rosbag::View view(bag);
    if (!view.size()) return timeline;
    timeline.begin_ns = view.getBeginTime().toNSec();
    timeline.end_ns = view.getEndTime().toNSec();
    timeline.current_ns = []() -> uint64_t {
      return ros::Time::isSimTime() && ros::Time::isValid()
                 ? ros::Time::now().toNSec() : 0;
    };
  } catch (const std::exception& e) {
    ROS_WARN_STREAM("Bag timestamp metadata unavailable for logging: " << e.what());
  }
  return timeline;
}
}  // namespace robot_health
