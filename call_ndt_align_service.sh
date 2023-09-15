#!/bin/bash
set -eux

# 読み込み
set +eux
source $HOME/autoware/install/setup.bash
set -eux

ros2 service call /localization/pose_estimator/ndt_align_srv tier4_localization_msgs/srv/PoseWithCovarianceStamped \
"{
  pose_with_covariance: {
    header: {frame_id: 'map'},
    pose: {
      pose: {
        position: {x: 0.0, y: 0.0, z: 0.0},
        orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
      },
      covariance: [0.25, 0.0,  0.0, 0.0, 0.0, 0.0,
                   0.0,  0.25, 0.0, 0.0, 0.0, 0.0,
                   0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
                   0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
                   0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
                   0.0,  0.0,  0.0, 0.0, 0.0, 0.06853891909122467]
    }
  }
}"
