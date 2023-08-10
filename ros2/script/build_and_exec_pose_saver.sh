#!/bin/bash

set -eux

cd $(dirname $0)/../

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 読み込み
set +eux
source ./install/setup.bash
set -eux

ros2 run pose_saver pose_saver.py
