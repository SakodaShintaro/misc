#!/bin/bash
set -eux

TARGET_ROSBAG_DIR=$(readlink -f $1)

set +eux
source ~/autoware/install/setup.bash
set -eux

cd $TARGET_ROSBAG_DIR/..
python3 ~/work/system-performance-evaluation/analysis/system_monitor_report/main.py $TARGET_ROSBAG_DIR
cd -
