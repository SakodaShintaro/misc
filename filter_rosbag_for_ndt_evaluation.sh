#!/bin/bash

set -eux

# 読み込み
set +eux
source $HOME/autoware/install/setup.bash
set -eux

TARGET=$(readlink -f $1)

ros2 bag filter ${TARGET} -o ${TARGET}_filtered --include \
    "/awsim/ground_truth/localization/kinematic_state" \
    "/localization/kinematic_state" \
    "/localization/util/downsample/pointcloud" \
    "/sensing/vehicle_velocity_converter/twist_with_covariance" \
    "/sensing/imu/imu_data" \
    "/tf_static" \
    "/sensing/gnss/pose_with_covariance" \
    "/initialpose"
