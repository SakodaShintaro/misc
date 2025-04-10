#!/bin/bash
set -eux

goal_type=${1:-"none"} # choice: "none", "short", "medium", "long", "inf"
awsim_type=${2:-"1"}

# スクリプトが終了する際にすべてのバックグラウンドプロセスを停止するためのトラップを設定
trap "kill 0" EXIT

# 読み込み
set +eux
source ./install/setup.bash
set -eux

# AWSIMをバックグラウンドで起動
if [ $awsim_type = "1" ]; then
    $HOME/Downloads/AWSIM_v1.3.1/AWSIM.x86_64 --json_path $HOME/misc/awsim_config.json &
elif [ $awsim_type = "3" ]; then
    $HOME/Downloads/shinuku_binary_3_lidars_2024_07_30/AWSIM/AWSIM.x86_64 --json_path $HOME/misc/awsim_config.json &
else
    echo "Invalid argument: $awsim_type"
    exit 1
fi

# Autowareを起動
ros2 launch autoware_launch e2e_simulator.launch.xml \
    map_path:=$HOME/Downloads/nishishinjuku_autoware_map_divided \
    vehicle_model:=sample_vehicle \
    sensor_model:=awsim_sensor_kit &

# サービス呼び出しを立ち上がりの確認とする
ros2 service call /localization/pose_twist_fusion_filter/trigger_node std_srvs/srv/SetBool "{data: false}"

# 安定性のため少し待つ
sleep 15

# ゴールを設定
if [ $goal_type = "none" ]; then
    # wait until the end
    wait
elif [ $goal_type = "short" ]; then
    python3 ~/misc/python_lib/goal_manager.py --goals_yaml ~/misc/awsim_goal/goals_short.yaml
elif [ $goal_type = "medium" ]; then
    python3 ~/misc/python_lib/goal_manager.py --goals_yaml ~/misc/awsim_goal/goals_medium.yaml
elif [ $goal_type = "long" ]; then
    python3 ~/misc/python_lib/goal_manager.py --goals_yaml ~/misc/awsim_goal/goals_long.yaml
elif [ $goal_type = "inf" ]; then
    python3 ~/misc/python_lib/goal_manager.py --goals_yaml ~/misc/awsim_goal/goals_long.yaml --loop_num 0
else
    echo "Invalid argument: $goal_type"
    exit 1
fi
