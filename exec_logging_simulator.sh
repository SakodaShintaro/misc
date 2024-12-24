#!/bin/bash

set -eux

# スクリプトが終了する際にすべてのバックグラウンドプロセスを停止するためのトラップを設定
trap "kill 0" EXIT

# 読み込み
set +eux
source $HOME/autoware/install/setup.bash
set -eux

# Autowareをバックグラウンドで起動
ros2 launch autoware_launch logging_simulator.launch.xml \
    map_path:=$HOME/autoware_map/sample-map-rosbag \
    vehicle_model:=sample_vehicle \
    sensor_model:=sample_sensor_kit &

# 立ち上がるまで待つ
# サービス呼び出しを立ち上がりの確認とする
ros2 service call /localization/pose_estimator/trigger_node std_srvs/srv/SetBool "{data: false}"

# rosbagをリプレイ
ros2 bag play $HOME/autoware_map/sample-rosbag/sample.db3 -r 0.5 -s sqlite3

# logを確認
dir_path=$(find ~/.ros/log -maxdepth 1 -type d | sort | tail -n 1)
log_path=$dir_path/launch.log
cat $log_path | grep "ERROR"
cat $log_path | grep "died"
