#!/bin/bash

set -eux

# スクリプトが終了する際にすべてのバックグラウンドプロセスを停止するためのトラップを設定
trap "kill 0" EXIT

# 読み込み
set +eux
source $HOME/autoware/install/setup.bash
set -eux

# 実行前にマージとビルドが必要
cd $HOME/autoware/src/sensor_kit/external/awsim_sensor_kit_launch/awsim_sensor_kit_launch
~/misc/merge_from_url.sh https://github.com/knzo25/awsim_sensor_kit_launch/tree/test/awsim_nebula_integration
cd $HOME/autoware
~/misc/build_with_custom_jobs.sh 2
~/misc/change_autoware_for_localization.sh

# AWSIMをバックグラウンドで起動
$HOME/Downloads/shinjuku_binary_1_lidar_2024_07_25/AWSIM/AWSIM.x86_64 \
    --json_path $HOME/Downloads/shinjuku_binary_1_lidar_2024_07_25/AWSIM/config.json &

# Autowareを起動
ros2 launch autoware_launch e2e_simulator.launch.xml \
    map_path:=$HOME/Downloads/nishishinjuku_autoware_map_divided \
    vehicle_model:=sample_vehicle \
    sensor_model:=awsim_sensor_kit \
    launch_sensing_driver:=true \
    use_sim_time:=false
