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

  # plot localization result
  python3 $script_dir/python_lib/plot_localization_result.py $target_rosbag

  # plot diagnostics
  python3 $script_dir/python_lib/plot_diagnostics.py $target_rosbag

  # compare trajectories
  dir_name=$(basename $dir)
  python3 $script_dir/python_lib/extract_pose_from_rosbag.py \
    --rosbag_path=$target_rosbag \
    --target_topic="/localization/kinematic_state" \
    --output_dir=$(dirname $target_rosbag)/compare_trajectories
  python3 $script_dir/python_lib/extract_pose_from_rosbag.py \
    --rosbag_path=$target_rosbag \
    --target_topic="/localization/reference_kinematic_state" \
    --output_dir=$(dirname $target_rosbag)/compare_trajectories
  python3 $script_dir/python_lib/compare_trajectories.py \
    $(dirname $target_rosbag)/compare_trajectories/localization__kinematic_state.tsv \
    $(dirname $target_rosbag)/compare_trajectories/localization__reference_kinematic_state.tsv

  elapsed_time=$(date -ud "@$SECONDS" +"%T")
done
