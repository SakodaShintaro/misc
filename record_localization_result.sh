#!/bin/bash
set -eux

SAVE_DIR=$1

set +eux
source $HOME/autoware/install/setup.bash
set -eux

ros2 bag record -o "$SAVE_DIR" --use-sim-time \
  /diagnostics \
  /localization/kinematic_state \
  /localization/pose_estimator/exe_time_ms \
  /localization/pose_estimator/iteration_num \
  /localization/pose_estimator/pose \
  /localization/pose_estimator/pose_with_covariance \
  /localization/pose_estimator/transform_probability \
  /localization/pose_estimator/nearest_voxel_transformation_likelihood \
  /localization/pose_estimator/initial_to_result_relative_pose \
  /localization/pose_estimator/ndt_marker \
  /localization/pose_twist_fusion_filter/pose \
  /localization/pose_twist_fusion_filter/kinematic_state \
  /localization/pose_twist_fusion_filter/pose_instability_detector/debug/diff_pose \
  /localization/util/downsample/pointcloud
