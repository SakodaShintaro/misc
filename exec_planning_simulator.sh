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
ros2 topic pub -1 /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{
  header: {
    stamp: {sec: 181, nanosec: 289995947},
    frame_id: 'map'},
  pose: {
    position: { x: 3771, y: 73699.0, z: 0.0 },
    orientation: { x: 0.0, y: 0.0, z: 0.25517065791082694, w: 0.9668960312987926 }
  }
}'
sleep 1

# engage
ros2 topic pub /autoware/engage autoware_vehicle_msgs/msg/Engage "engage: true" -1
sleep 25

# logを確認
dir_path=$(find ~/.ros/log -maxdepth 1 -type d | sort | tail -n 1)
log_path=$dir_path/launch.log
cat $log_path | grep "ERROR"
cat $log_path | grep "died"
