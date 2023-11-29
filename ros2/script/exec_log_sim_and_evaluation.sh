#!/bin/bash

set -eux

# スクリプトが終了する際にすべてのバックグラウンドプロセスを停止するためのトラップを設定
trap "kill 0" EXIT

# 実行するrosbagへのパス
ROSBAG=$(readlink -f $1)

# 結果を保存する位置
SAVE_DIR=$(readlink -f $2)

# このディレクトリに移動
cd $(dirname $0)

# 読み込み
set +eux
source $HOME/autoware/install/setup.bash
set -eux

# Autowareをバックグラウンドで起動
ros2 launch autoware_launch logging_simulator.launch.xml \
    map_path:=$HOME/Downloads/nishishinjuku_autoware_map \
    pose_source:=ndt \
    vehicle_model:=sample_vehicle \
    sensor_model:=awsim_sensor_kit \
    perception:=false \
    planning:=false \
    control:=false &

# 立ち上がるまで待つ
# サービス呼び出しを立ち上がりの確認とする
ros2 service call /localization/pose_twist_fusion_filter/trigger_node std_srvs/srv/SetBool "{data: false}"

# 安定性のため少し待つ
sleep 5

# 保存
ros2 bag record -o $SAVE_DIR/result_rosbag --use-sim-time /localization/pose_twist_fusion_filter/pose /localization/pose_estimator/pose &

# rosbagをリプレイ
ros2 bag play ${ROSBAG} -r 0.75 -s sqlite3 --remap /localization/pose_twist_fusion_filter/biased_pose_with_covariance:=/null

# 終了
../../kill_autoware.sh

# gtが無ければgtを生成
if [ ! -e $SAVE_DIR/ground_truth.tsv ]; then
    python3 ../python/extract_gt_pose_from_rosbag.py \
        --rosbag_path=$ROSBAG \
        --target_topic="/awsim/ground_truth/vehicle/pose" \
        --output_dir=$SAVE_DIR
fi

# rosbagからtsvに変換
python3 ../python/extract_gt_pose_from_rosbag.py \
    --rosbag_path=$SAVE_DIR/result_rosbag \
    --target_topic="/localization/pose_twist_fusion_filter/pose" \
    --output_dir=$SAVE_DIR
python3 ../python/extract_gt_pose_from_rosbag.py \
    --rosbag_path=$SAVE_DIR/result_rosbag \
    --target_topic="/localization/pose_estimator/pose" \
    --output_dir=$SAVE_DIR

# 評価
python3 ../python/compare_trajectories.py \
    $SAVE_DIR/localization__pose_twist_fusion_filter__pose.tsv \
    $SAVE_DIR/awsim__ground_truth__vehicle__pose.tsv
python3 ../python/compare_trajectories.py \
    $SAVE_DIR/localization__pose_estimator__pose.tsv \
    $SAVE_DIR/awsim__ground_truth__vehicle__pose.tsv
