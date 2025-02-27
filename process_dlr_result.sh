#!/bin/bash
set -ux

set +ux
source ~/autoware/install/setup.bash
set -ux

script_dir=$(readlink -f $(dirname $0))

result_dir=$1
dir_list=$(find $result_dir -mindepth 1 -maxdepth 1 -type d | sort)

for dir in $dir_list; do
  target_rosbag=$dir/result_bag
  compare_result_dir=$(dirname $target_rosbag)/compare_trajectories
  mkdir -p $compare_result_dir

  # plot localization result
  python3 $script_dir/python_lib/plot_localization_result.py $target_rosbag

  # plot diagnostics
  ros2 run autoware_localization_evaluation_scripts plot_diagnostics.py $target_rosbag

  # compare trajectories
  dir_name=$(basename $dir)
  python3 $script_dir/python_lib/extract_pose_from_rosbag.py \
    $target_rosbag \
    --save_dir $compare_result_dir \
    --target_topics "/localization/kinematic_state" \
                    "/localization/reference_kinematic_state"
  python3 $script_dir/python_lib/compare_trajectories.py \
    $compare_result_dir/localization__kinematic_state.tsv \
    $compare_result_dir/localization__reference_kinematic_state.tsv

  elapsed_time=$(date -ud "@$SECONDS" +"%T")
done
