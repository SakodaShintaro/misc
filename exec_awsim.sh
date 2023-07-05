#!/bin/bash

set -eux

# スクリプトが終了する際にすべてのバックグラウンドプロセスを停止するためのトラップを設定
trap "kill 0" EXIT

# 読み込み
set +eux
source $HOME/autoware/install/setup.bash
set -eux

# AWSIMをバックグラウンドで起動
$HOME/Downloads/AWSIM_v1.1.0/AWSIM_demo.x86_64 &

# Autowareを起動
ros2 launch autoware_launch e2e_simulator.launch.xml \
    map_path:=$HOME/Downloads/nishishinjuku_autoware_map \
    vehicle_model:=sample_vehicle \
    sensor_model:=awsim_sensor_kit
