#!/bin/bash

set -eux

# スクリプトが終了する際にすべてのバックグラウンドプロセスを停止するためのトラップを設定
trap "kill 0" EXIT

# 結果を保存する位置
SAVE_DIR=$(readlink -f $1)

# このディレクトリに移動
cd $(dirname $0)

# saverを起動
./build_and_exec_pose_saver.sh $SAVE_DIR &

# 読み込み
set +eux
source $HOME/autoware/install/setup.bash
set -eux

# Autowareをバックグラウンドで起動
ros2 launch autoware_launch logging_simulator.launch.xml \
    map_path:=$HOME/Downloads/nishishinjuku_autoware_map \
    vehicle_model:=sample_vehicle \
    sensor_model:=awsim_sensor_kit &

# 立ち上がるまで待つ
# サービス呼び出しを立ち上がりの確認とする
ros2 service call /localization/pose_estimator/trigger_node std_srvs/srv/SetBool "{data: false}"

# rosbagをリプレイ
ros2 bag play $HOME/data/rosbag/AWSIM/rosbag2_2023_08_17-18_03_10_filtered -s sqlite3

# 終了
../../kill_autoware.sh

# 評価
python3 ../python/compare_trajectories.py \
    $SAVE_DIR/localization.tsv \
    $SAVE_DIR/ground_truth.tsv