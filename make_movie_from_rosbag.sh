#!/bin/bash
set -eux

TARGET_ROSBAG=$(readlink -f $1)
TARGET_TOPIC=${2:-camera0}

# 読み込み
set +eux
source $HOME/autoware/install/setup.bash
set -eux

python3 ~/misc/python_lib/extract_images_from_rosbag.py \
  $TARGET_ROSBAG \
  $TARGET_ROSBAG/../images \
  --target_camera_name=$TARGET_TOPIC \
  --skip_num=5

# python3 ~/misc/python_lib/extract_valuess_from_rosbag.py
# python3 ~/misc/python_lib/calc_camera_pose.py
# python3 ~/misc/python_lib/plot_camera_and_pose.py
# ~/misc/ffmpeg_lib/make_mp4_from_unsequential_png.sh
