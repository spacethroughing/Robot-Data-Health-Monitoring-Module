#!/usr/bin/env bash
# 作用：给已有Noetic的Ubuntu20.04安装C++构建与标准消息依赖；不修改ROS软件源。
# 数据流：选择gui/headless -> 核对系统/ROS -> 明确apt包列表 -> 安装。
set -euo pipefail
mode="${1:-gui}"
case "$mode" in gui|headless) ;; *) printf '%s\n' '用法: bash scripts/install_dependencies.sh gui|headless' >&2; exit 2;; esac
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 20.04 ]]; then
  printf '%s\n' '本安装脚本适用于已安装ROS Noetic的Ubuntu 20.04。当前系统不匹配。' >&2; exit 2
fi
if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  printf '%s\n' '未找到ROS Noetic，请先准备与项目一致的ROS1环境。' >&2; exit 2
fi
# 标准模式不安装FAST-LIO或Livox；--livox构建需要用户已有的livox_ros_driver。
packages=(build-essential cmake python3 python3-empy python3-catkin-pkg python3-rospkg python3-yaml
  libeigen3-dev libyaml-cpp-dev libssl-dev libboost-filesystem-dev libboost-system-dev
  ros-noetic-catkin ros-noetic-roscpp ros-noetic-rosbag ros-noetic-rosgraph-msgs
  ros-noetic-roslaunch ros-noetic-rosbash ros-noetic-rospack ros-noetic-rosnode
  ros-noetic-rostopic ros-noetic-rosservice ros-noetic-rosparam ros-noetic-rosmaster
  ros-noetic-std-msgs ros-noetic-std-srvs ros-noetic-sensor-msgs ros-noetic-nav-msgs
  ros-noetic-geometry-msgs ros-noetic-diagnostic-msgs ros-noetic-message-generation ros-noetic-message-runtime)
if [[ "$mode" == gui ]]; then packages+=(qtbase5-dev ros-noetic-rviz ros-noetic-pluginlib); fi
# 只有执行本脚本才会使用sudo安装；交付验收没有替用户运行apt。
sudo apt-get update
sudo apt-get install --no-install-recommends "${packages[@]}"
