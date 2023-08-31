#!/bin/bash
set -eux

ROSBAG=$(readlink -f $1)
SAVE_DIR=$(readlink -f $2)

cd $(dirname $0)

mkdir -p $SAVE_DIR

for i in $(seq -w 000 999); do
    ~/misc/ros2/script/exec_log_sim_and_evaluation.sh \
        $ROSBAG \
        $SAVE_DIR/trial_$i
    sleep 3
done
