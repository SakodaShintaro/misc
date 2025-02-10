#!/bin/bash

set -eux

# スクリプトが終了する際にすべてのバックグラウンドプロセスを停止するためのトラップを設定
trap "kill 0" EXIT

# 読み込み
set +eux
source ./install/setup.bash
set -eux

# cd $HOME/autoware/src/sensor_component/external/nebula/
# git checkout v0.2.2
# cd $HOME/autoware/src/sensor_kit/external/awsim_sensor_kit_launch
# ~/misc/merge_from_url.sh https://github.com/SakodaShintaro/awsim_sensor_kit_launch/tree/fix/add_udp_only
# cd $HOME/autoware/src/sensor_kit/sample_sensor_kit_launch
# ~/misc/merge_from_url.sh https://github.com/SakodaShintaro/sample_sensor_kit_launch/tree/fix/add_udp_only
# cd $HOME/autoware
# ~/misc/build_with_custom_jobs.sh 2

# AWSIMをバックグラウンドで起動
$HOME/Downloads/shinjuku_binary_1_lidar_2024_07_25/AWSIM/AWSIM.x86_64 \
    --json_path $HOME/Downloads/shinjuku_binary_1_lidar_2024_07_25/AWSIM/config.json &

# Autowareを起動
ros2 launch autoware_launch e2e_simulator.launch.xml \
    map_path:=$HOME/Downloads/nishishinjuku_autoware_map_divided \
    vehicle_model:=sample_vehicle \
    sensor_model:=awsim_sensor_kit \
    launch_sensing_driver:=true \
    use_sim_time:=false &

# 立ち上がるまで待つ
# サービス呼び出しを立ち上がりの確認とする
ros2 service call /localization/pose_twist_fusion_filter/trigger_node std_srvs/srv/SetBool "{data: false}"

# 初期位置推定のステータスが3になるまで待つ
while true; do
    status=$(ros2 topic echo /localization/initialization_state --once | grep -oP 'state: \K\d')
    if [ $status -eq 3 ]; then
        break
    fi
    sleep 2
done

# 安定性のため少し待つ
sleep 5

# ループ開始
python3 ~/misc/python_lib/goal_manager.py --goals_yaml ~/misc/awsim_goal/goals_long.yaml --loop_num 0
