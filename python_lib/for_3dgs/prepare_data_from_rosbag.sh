#!/bin/bash
set -eux

rosbag_path=$(readlink -f $1)
mkdir -p $2
output_dir=$(readlink -f $2)

pose_topic=/awsim/ground_truth/localization/kinematic_state

# 画像を抽出
python3 $HOME/misc/python_lib/extract_images_from_rosbag.py \
  $rosbag_path \
  $output_dir/images

# poseを抽出
python3 $HOME/misc/python_lib/extract_pose_from_rosbag.py \
  --rosbag_path=$rosbag_path \
  --target_topic=$pose_topic \
  --output_dir=$output_dir/images

# poseをカメラposeに変換
python3 $HOME/misc/python_lib/calc_camera_pose.py \
  $output_dir/images \
  --pose_topic_name=$pose_topic

# colmap形式に変換
python3 $HOME/misc/python_lib/convert_tsv_to_cameras_txt.py \
  $output_dir
