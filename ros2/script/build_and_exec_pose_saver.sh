#!/bin/bash

set -eux

SAVE_DIR=$(readlink -f $1)

cd $(dirname $0)/../

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 読み込み
set +eux
source ./install/setup.bash
set -eux

ros2 run pose_saver pose_saver.py --ros-args --param save_directory:=$SAVE_DIR