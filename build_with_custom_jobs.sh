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

# なぜか再ビルドだとエラー起きまくるので一度削除
rm -rf \
    build/autoware_remaining_distance_time_calculator \
    install/autoware_remaining_distance_time_calculator

MAKEFLAGS="-j8" colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  --continue-on-error \
  --packages-up-to openscenario_interpreter

MAKEFLAGS="-j${JOB_COUNT}" colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  --continue-on-error

# 後処理
if [[ $current_dir =~ ^(autoware) ]]; then
  cp $(dirname $0)/autoware.rviz ./src/launcher/autoware_launch/autoware_launch/rviz/autoware.rviz
fi
$(dirname $0)/check_git_diff.sh
