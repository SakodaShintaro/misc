#!/bin/bash

set -eux

# 読み込み
set +eux
source $HOME/autoware/install/setup.bash
set -eux

TARGET=$(readlink -f $1)

# ros2 bag filter ${TARGET} -o ${TARGET}_filtered -x \
#     "/planning/.*" \
#     "/control/.*" \
#     "/perception/.*" \
#     "/localization/.*" \
#     "/sensing/lidar/concatenated/pointcloud" \
#     "/tf"

ros2 bag filter ${TARGET} -o ${TARGET}_filtered --include \
    "/sensing/camera/traffic_light/camera_info" \
    "/sensing/camera/traffic_light/image_raw" \
    "/sensing/gnss/pose" \
    "/sensing/gnss/pose_with_covariance" \
    "/sensing/imu/tamagawa/imu_raw" \
    "/sensing/lidar/top/pointcloud_raw" \
    "/sensing/lidar/top/pointcloud_raw_ex" \
    "/vehicle/status/velocity_status" \
    "/awsim/ground_truth/localization/kinematic_state" \
    "/awsim/ground_truth/vehicle/pose" \
    "/initialpose" \
    "/clock"
