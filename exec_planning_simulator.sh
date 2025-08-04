#!/bin/bash

set -eux

# スクリプトが終了する際にすべてのバックグラウンドプロセスを停止するためのトラップを設定
trap "kill 0" EXIT

# 読み込み
set +eux
source $HOME/autoware/install/setup.bash
set -eux

# planning_simulator
ros2 launch autoware_launch planning_simulator.launch.xml \
  map_path:=$HOME/autoware_map/sample-map-planning \
  vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit &

# 立ち上がるまで待つ
while ! ros2 service type /planning/scenario_planning/scenario_selector/get_parameters; do
  echo "waiting for /planning/scenario_planning/scenario_selector/get_parameters"
  sleep 4
done

# 安定性のためにさらに少し待つ
sleep 5

# initialpose
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{
  header: { frame_id: "map" },
  pose: {
    pose: {
      position: { x: 3734.4, y: 73680.015625, z: 0.0 },
      orientation: { x: 0.0, y: 0.0, z: 0.25517065791082694, w: 0.9668960312987926 }
    },
    covariance: [0.25, 0.0,  0.0, 0.0, 0.0, 0.0,
                 0.0,  0.25, 0.0, 0.0, 0.0, 0.0,
                 0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
                 0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
                 0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
                 0.0,  0.0,  0.0, 0.0, 0.0, 0.06853891909122467]
  }
}'
sleep 1

# goal
# ros2 topic pub -1 /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{
#   header: {
#     stamp: {sec: 181, nanosec: 289995947},
#     frame_id: 'map'},
#   pose: {
#     position: { x: 3771, y: 73699.0, z: 0.0 },
#     orientation: { x: 0.0, y: 0.0, z: 0.25517065791082694, w: 0.9668960312987926 }
#   }
# }'
ros2 topic pub -1 /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{
  header: {
    stamp: {sec: 181, nanosec: 289995947},
    frame_id: 'map'},
  pose: {
    position: { x: 3713.93994140625, y: 73769.125, z: 0.0 },
    orientation: { x: 0.0, y: 0.0, z: 0.24573643403017062, w: 0.9693366829900412 }
  }
}'
sleep 1

# ros2 topic pub -1 /simulation/dummy_perception_publisher/object_info tier4_simulation_msgs/msg/DummyObject '{
#   header: {
#     stamp: {sec: 181, nanosec: 289995947},
#     frame_id: 'map'},
#   id: { uuid: [224, 179, 108, 65, 32, 224, 108, 69, 5, 199, 73, 132, 26, 165, 235, 142] },
#   initial_state: {
#     pose_covariance: {
#       pose: {
#         position: { x: 3768.849609375, y: 73727.6171875, z: 0.0 },
#         orientation: { x: 0.0, y: 0.0, z: -0.5259276490507606, w: 0.850529310467276 }
#       },
#       covariance: [0.0009, 0.0,  0.0, 0.0, 0.0, 0.0,
#                    0.0,  0.0009, 0.0, 0.0, 0.0, 0.0,
#                    0.0,  0.0, 0.0009, 0.0, 0.0, 0.0,
#                    0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
#                    0.0,  0.0,  0.0, 0.0, 0.0, 0.0,
#                    0.0,  0.0,  0.0, 0.0, 0.0, 0.007]
#     },
#     twist_covariance: {
#       twist: {
#         linear: { x: 0.0, y: 0.0, z: 0.0 },
#         angular: { x: 0.0, y: 0.0, z: 0.0 }
#       },
#       covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#     },
#     accel_covariance: {
#       accel: {
#         linear: { x: 0.0, y: 0.0, z: 0.0 },
#         angular: { x: 0.0, y: 0.0, z: 0.0 }
#       },
#       covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#     }
#   },
#   classification: {
#     label: 3,
#     probability: 1.0
#   },
#   shape: {
#     type: 0,
#     footprint: { points: [] },
#     dimensions: {
#       x: 10.5,
#       y: 2.5,
#       z: 3.5
#     }
#   },
#   max_velocity: 33.29999923706055,
#   min_velocity: -33.29999923706055,
#   action: 0
# }'
sleep 1

# engage
ros2 topic pub /autoware/engage autoware_vehicle_msgs/msg/Engage "engage: true" -1
sleep 50

# logを確認
dir_path=$(find ~/.ros/log -maxdepth 1 -type d | sort | tail -n 1)
log_path=$dir_path/launch.log
cat $log_path | grep "ERROR"
cat $log_path | grep "died"
