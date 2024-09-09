#!/bin/bash

set -eux

JOB_COUNT=$1

# 現状のディレクトリがautowareまたはpilot-autoというプレフィックスを持つことを確認する
current_dir=$(basename $(pwd))
if [[ ! $current_dir =~ ^(autoware|pilot-auto) ]]; then
    echo "This script must be run in a directory with a prefix of autoware or pilot-auto."
    exit 1
fi

rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

# 存在する場合はCOLCON_IGNOREファイルを作成する
if [ -e ./src/tools/planning/autoware_planning_data_analyzer/ ]; then
  touch ./src/tools/planning/autoware_planning_data_analyzer/COLCON_IGNORE
fi
if [ -e ./src/tools/planning/planning_debug_tools/ ]; then
  touch ./src/tools/planning/planning_debug_tools/COLCON_IGNORE
fi

MAKEFLAGS="-j${JOB_COUNT}" colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  --continue-on-error

# 後処理
if [[ $current_dir =~ ^(autoware) ]]; then
  cp $(dirname $0)/autoware.rviz ./src/launcher/autoware_launch/autoware_launch/rviz/autoware.rviz
fi
$(dirname $0)/check_git_diff.sh
