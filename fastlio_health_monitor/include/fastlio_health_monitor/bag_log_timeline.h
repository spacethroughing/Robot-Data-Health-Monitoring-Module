/**
 * rosbag回放时间到日志时间的可选桥接层。
 *
 * 用法：回放节点在创建EventRecorder前调用bagLogTimeline(bag_path)，将返回的
 * LogTimeline传给记录器。此函数只读bag的起止元数据；回放期间记录器通过回调
 * 读取ROS /clock当前位置，并计算相对bag起点的bag_elapsed_sec。
 *
 * 真实机器人没有bag时间轴时返回空时间提供器，日志会把bag_elapsed_sec记录为
 * null；不根据系统时间或消息频率伪造bag时间。函数定义在头文件并标记inline，
 * 所以多个.cpp包含本文件时不会产生重复定义。
 */
#pragma once

#include <fastlio_health_monitor/event_recorder.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace robot_health {
/**
 * 为日志创建只读bag时间提供器。
 * @param path 正在回放的bag路径；空路径或未启用ROS仿真时间时返回空时间轴。
 * @return begin_ns/end_ns为bag中的首末记录时间；current_ns读取有效的ROS /clock。
 *         如果bag缺失、未索引、为空或元数据无法读取，current_ns保持为空。
 * 打开bag始终使用只读模式，不会修复、重写或修改bag文件。失败通过ROS警告报告，
 * 但不阻止记录其他时间字段和健康事件。
 */
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
