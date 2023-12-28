#!/bin/bash

set -eux

TARGET_ROSBAG=$(readlink -f $1)
TARGET_TOPIC=$2

# 読み込み
set +eux
source $HOME/autoware/install/setup.bash
set -eux

ros2 bag filter ${TARGET_ROSBAG} -o ${TARGET_ROSBAG}_filtered -x ${TARGET_TOPIC}
