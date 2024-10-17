#!/bin/bash
set -eux

awsim_type=${1:-"1"}

# スクリプトが終了する際にすべてのバックグラウンドプロセスを停止するためのトラップを設定
trap "kill 0" EXIT

# 読み込み
set +eux
source $HOME/autoware/install/setup.bash
set -eux

cd $HOME/autoware
~/misc/change_autoware_for_localization.sh

# AWSIMをバックグラウンドで起動
if [ $awsim_type = "1" ]; then
    $HOME/Downloads/AWSIM_v1.3.0/AWSIM.x86_64 &
elif [ $awsim_type = "3" ]; then
    $HOME/Downloads/shinuku_binary_3_lidars_2024_07_30/AWSIM/AWSIM.x86_64 &
else
    echo "Invalid argument: $awsim_type"
    exit 1
fi

# Autowareを起動
ros2 launch autoware_launch e2e_simulator.launch.xml \
    map_path:=$HOME/Downloads/nishishinjuku_autoware_map_divided \
    vehicle_model:=sample_vehicle \
    sensor_model:=awsim_sensor_kit
