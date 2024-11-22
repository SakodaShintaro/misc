#!/bin/bash

set -eux

TARGET_ROSBAG=$(readlink -f $1)
INCLUDE_OR_EXCLUDE=$2
TARGET_TOPIC=$3

# 読み込み
set +eux
source $HOME/autoware/install/setup.bash
set -eux

rm -rf ${TARGET_ROSBAG}_filtered

ros2 bag filter ${TARGET_ROSBAG} -o ${TARGET_ROSBAG}_filtered -${INCLUDE_OR_EXCLUDE} ${TARGET_TOPIC}
python3 ~/misc/python_lib/delete_0_topics.py ${TARGET_ROSBAG}_filtered
