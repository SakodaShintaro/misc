#!/bin/bash
set -eux

result_dir=$1

set +eux
source ~/autoware/install/setup.bash
set -eux

ros2 run autoware_localization_evaluation_scripts analyze_rosbags_parallel.py \
   $result_dir \
   --parallel_num 8 \
   --topic_subject "/localization/kinematic_state" \
   --topic_reference "/localization/reference_kinematic_state"
