#!/usr/bin/env bash
set -euo pipefail
# 作用：作为被托管定位进程的启动入口；未配置前返回78，防止占位命令被当成真实启动成功。
# 数据流：控制服务RUN -> 此脚本exec真实定位launch -> 控制器核对真实节点名 -> 健康生命周期。
# 下面只提供写法，必须改成机器人的真实工作区、包名与launch：
# source /opt/ros/noetic/setup.bash
# source /home/robot/robot_ws/devel/setup.bash
# exec roslaunch your_localization_package your_real_localization.launch
printf '%s\n' '尚未配置真实定位启动命令。请编辑本脚本，并同步runtime_localization.example.yaml的supervised_ros_node。' >&2
exit 78
