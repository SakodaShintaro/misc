#!/bin/bash
set -eux

set +eux
source $HOME/autoware/install/setup.bash
set -eux

TOPICS=(
    "/sensing/camera/traffic_light/camera_info"
    "/sensing/camera/traffic_light/image_raw"
    "/sensing/gnss/pose_with_covariance"
    "/sensing/imu/tamagawa/imu_raw"
    "/sensing/lidar/top/pointcloud_raw_ex"
    "/vehicle/status/velocity_status"
    "/awsim/ground_truth/localization/kinematic_state"
    "/awsim/ground_truth/vehicle/pose"
    "/localization/pose_twist_fusion_filter/biased_pose_with_covariance"
    "/localization/kinematic_state"
    "/localization/util/downsample/pointcloud"
    "/initialpose"
    "/tf_static"
)

CURRENT_TIME=$(date +"%Y%m%d_%H%M%S")
ros2 bag record -o "$CURRENT_TIME" --use-sim-time "${TOPICS[@]}"
