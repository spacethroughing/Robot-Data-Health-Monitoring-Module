#!/usr/bin/env bash
# 安装数据流：读取本交付目录 -> 复制到一个新工作区 -> 加载现有ROS -> 在目标机编译。
# 不复制本机二进制，不覆盖已有同名源码；不同CPU架构都在目标机编译。
set -eo pipefail
bundle_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# 支持两种分发目录：GitHub仓库根目录直接含fastlio_health_monitor，桌面交付包则放在src/下。
if [[ -d "$bundle_dir/src/fastlio_health_monitor" ]]; then
  monitor_source_dir="$bundle_dir/src/fastlio_health_monitor"
elif [[ -d "$bundle_dir/fastlio_health_monitor" ]]; then
  monitor_source_dir="$bundle_dir/fastlio_health_monitor"
else
  printf '%s\n' "未找到fastlio_health_monitor源码包：$bundle_dir" >&2
  exit 2
fi
workspace_dir="${HOME}/robot_health_ws_20261019"
mode=ON
livox=OFF
jobs=2
# 参数仅选择构建能力；具体传感器Topic/检测阈值由安装后的YAML决定。
while (($#)); do
  case "$1" in
    --workspace) workspace_dir="$2"; shift 2;;
    --headless) mode=OFF; shift;;
    --gui) mode=ON; shift;;
    --livox) livox=ON; shift;;
    --jobs) jobs="$2"; shift 2;;
    *) printf '%s\n' '用法: install_workspace.sh [--workspace /绝对路径] [--gui|--headless] [--livox] [--jobs 2]' >&2; exit 2;;
  esac
done
# 避免相对路径和零并发数量造成安装位置/构建行为不明确。
[[ "$workspace_dir" = /* ]] || { printf '%s\n' '工作区请使用绝对路径。' >&2; exit 2; }
[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || { printf '%s\n' 'jobs必须为正整数。' >&2; exit 2; }
if [[ -e "$workspace_dir/src/fastlio_health_monitor" ]]; then
  printf '%s\n' "目标已有源码：$workspace_dir/src/fastlio_health_monitor；请换一个新工作区路径。" >&2
  exit 2
fi
mkdir -p "$workspace_dir/src"
cp -a "$monitor_source_dir" "$workspace_dir/src/"
# Livox可选构建需要继承用户已加载的驱动工作区；标准构建只需要Noetic。
if [[ "$livox" == ON ]]; then
  source /opt/ros/noetic/setup.bash --extend
else
  source /opt/ros/noetic/setup.bash
fi
cd "$workspace_dir"
# 默认不自动跑测试；用户可按测试说明显式构建/执行。交付前已经另行验收。
catkin_make -j"$jobs" -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_ENABLE_TESTING=OFF -DBUILD_HEALTH_RVIZ_PANEL="$mode" -DBUILD_HEALTH_LIVOX_ADAPTER="$livox"
printf '编译完成。每个新终端先加载：\nsource /opt/ros/noetic/setup.bash\nsource %q/devel/setup.bash\n' "$workspace_dir"
