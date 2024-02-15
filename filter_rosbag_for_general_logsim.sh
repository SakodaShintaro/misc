#!/bin/bash

set -eux

TARGET_ROSBAG=$(readlink -f $1)

# 読み込み
set +eux
source $HOME/autoware/install/setup.bash
set -eux

ros2 bag filter ${TARGET_ROSBAG} -o ${TARGET_ROSBAG}_filtered -i \
  /localization/util/downsample/pointcloud \
  /localization/twist_estimator/twist_with_covariance
