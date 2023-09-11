#!/bin/bash
set -eu

TOPIC_NAME="/localization/initialization_state"

# トピックがpublishされるまで待機
while ! ros2 topic list | grep -q "$TOPIC_NAME"; do
  echo "Waiting for $TOPIC_NAME to be published..."
  sleep 1.0
done

ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
  header: { frame_id: "map" },
  pose: {
    pose: {
      position: { x: 81377.34375, y: 49916.89453125, z: 41.205413818359375 },
      orientation: { x: 0.00017802136426325887, y: -0.007339125499129295, z: 0.3006163239479065, w: 0.9537169337272644 }
    },
    covariance: [0.25, 0.0,  0.0, 0.0, 0.0, 0.0,
                 0.0,  0.25, 0.0, 0.0, 0.0, 0.0,
                 0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
                 0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
                 0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
                 0.0,  0.0,  0.0, 0.0, 0.0, 0.06853891909122467]
  }
}'

# AWSIM initial GNSS
# ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
#   header: { frame_id: "map" },
#   pose: {
#     pose: {
#       position: { x: 81377.976562, y: 49917.347656, z: 43.032684 },
#       orientation: { x: 0.000000, y: 0.000000, z: 0.500000, w: 0.000000 }
#     },
#     covariance: [1.0, 0.0, 0.0,  0.0,  0.0,  0.0,
#                  0.0, 1.0, 0.0,  0.0,  0.0,  0.0,
#                  0.0, 0.0, 0.01, 0.0,  0.0,  0.0,
#                  0.0, 0.0, 0.0,  0.01, 0.0,  0.0,
#                  0.0, 0.0, 0.0,  0.0,  0.01, 0.0,
#                  0.0, 0.0, 0.0,  0.0,  0.0,  10.0]
#   }
# }'
