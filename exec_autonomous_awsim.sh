#!/bin/bash
set -eux

awsim_type=${1:-"1"}
autoware_path=${2:-"$HOME/autoware"}

# スクリプトが終了する際にすべてのバックグラウンドプロセスを停止するためのトラップを設定
trap "~/misc/kill_autoware.sh && kill 0" EXIT

# 読み込み
set +eux
source $autoware_path/install/setup.bash
set -eux

cd $autoware_path
~/misc/change_autoware_for_localization.sh

# AWSIMをバックグラウンドで起動
if [ $awsim_type = "1" ]; then
    $HOME/Downloads/AWSIM_v1.3.0/AWSIM.x86_64 &
elif [ $awsim_type = "3" ]; then
    $HOME/Downloads/shinuku_binary_3_lidars_2024_07_30/AWSIM/AWSIM.x86_64 &
elif [ $awsim_type = "with_config" ]; then
    $HOME/Downloads/shinuku_binary_3_lidars_2024_07_30/AWSIM/AWSIM.x86_64 --json_path $HOME/misc/awsim_config.json &
else
    echo "Invalid argument: $awsim_type"
    exit 1
fi

# Autowareを起動
gnome-terminal --window -- ros2 launch autoware_launch e2e_simulator.launch.xml \
    map_path:=$HOME/Downloads/nishishinjuku_autoware_map_divided \
    vehicle_model:=sample_vehicle \
    sensor_model:=awsim_sensor_kit

# ループを実行
sleep 40
python3 ~/misc/python_lib/loop_manager_nishishinjuku.py
