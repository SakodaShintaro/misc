#!/bin/bash

set -eux

# スクリプトが終了する際にすべてのバックグラウンドプロセスを停止するためのトラップを設定
trap "kill 0" EXIT

# 実行するmap・rosbagへのパス
MAP=$(readlink -f $1)
ROSBAG=$(readlink -f $2)
SAVE_DIR=${3:-$HOME/data/misc/$(date +"%Y%m%d_%H%M%S")_general_logging}

mkdir -p $SAVE_DIR
SAVE_DIR=$(realpath $SAVE_DIR)

# このディレクトリに移動
cd $(dirname $0)

# 読み込み
set +eux
source $HOME/autoware/install/setup.bash
set -eux

# Autowareをバックグラウンドで起動
ros2 launch autoware_launch logging_simulator.launch.xml \
    map_path:=$MAP \
    pose_source:=ndt \
    vehicle_model:=sample_vehicle \
    sensor_model:=sample_sensor_kit \
    vehicle:=false \
    system:=false \
    map:=true \
    sensing:=false \
    localization:=true \
    perception:=false \
    planning:=false \
    control:=false &

# 立ち上がるまで待つ
# サービス呼び出しを立ち上がりの確認とする
ros2 service call /localization/pose_twist_fusion_filter/trigger_node std_srvs/srv/SetBool "{data: false}"

# 安定性のため少し待つ
sleep 3

# 保存
./record_localization_result.sh $SAVE_DIR/result_bag &

# rosbagをリプレイ
ros2 bag play ${ROSBAG} -r 1.0 --clock 200

# 終了
./kill_autoware.sh
