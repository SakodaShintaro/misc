#!/bin/bash
set -eux

rosbag_path=$(readlink -f $1)
mkdir -p $2
output_dir=$(readlink -f $2)

pose_topic=/localization/kinematic_state

set +eux
source ~/autoware/install/setup.bash
set -eux

# poseを抽出
ros2 run autoware_localization_evaluation_scripts extract_values_from_rosbag.py \
  $rosbag_path \
  --target_topics=$pose_topic \
  --save_dir=$output_dir/images

# 画像を抽出
python3 $HOME/misc/python_lib/extract_images_from_rosbag.py \
  $rosbag_path \
  $output_dir/images

# poseをカメラposeに変換
python3 $HOME/misc/python_lib/calc_camera_pose.py \
  $output_dir/images \
  --pose_topic_name=$pose_topic

# colmap形式に変換
python3 $HOME/misc/python_lib/convert_tsv_to_cameras_txt.py \
  $output_dir
