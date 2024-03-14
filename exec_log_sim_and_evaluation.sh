#!/bin/bash

set -eux

# スクリプトが終了する際にすべてのバックグラウンドプロセスを停止するためのトラップを設定
trap "kill 0" EXIT

# 実行するmapへのパス
MAP=$(readlink -f $1)

# 実行するrosbagへのパス
ROSBAG=$(readlink -f $2)

# 実行する手法(ndt or artag)
METHOD=$3

# 結果を保存する位置
SAVE_DIR=$(readlink -f $4)

# このディレクトリに移動
cd $(dirname $0)

# 読み込み
set +eux
source $HOME/autoware/install/setup.bash
set -eux

# Autowareをバックグラウンドで起動
ros2 launch autoware_launch logging_simulator.launch.xml \
    map_path:=$MAP \
    pose_source:=$METHOD \
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

# 保存先作成
mkdir -p $SAVE_DIR

# 保存
./record_localization_result.sh $SAVE_DIR/result_rosbag &

# CPU利用率の表示
mpstat 1 > $SAVE_DIR/cpu_usage.txt &

# rosbagをリプレイ
# 【以前のコマンド】
# ros2 bag play ${ROSBAG} -r 0.75 -s sqlite3 --remap /localization/pose_twist_fusion_filter/biased_pose_with_covariance:=/null
# 【データ入れ替え後のコマンド】
ros2 bag play ${ROSBAG} --rate 1.0 \
  --storage sqlite3 \
  --clock 200 \
  --remap /localization/pose_twist_fusion_filter/biased_pose_with_covariance:=/null

# 終了
./kill_autoware.sh

# gtが無ければgtを生成
if [ ! -e $SAVE_DIR/ground_truth.tsv ]; then
    python3 python_lib/extract_pose_from_rosbag.py \
        --rosbag_path=$ROSBAG \
        --target_topic="/awsim/ground_truth/vehicle/pose" \
        --output_dir=$SAVE_DIR
fi

# rosbagからtsvに変換
python3 python_lib/extract_pose_from_rosbag.py \
    --rosbag_path=$SAVE_DIR/result_rosbag \
    --target_topic="/localization/pose_twist_fusion_filter/pose" \
    --output_dir=$SAVE_DIR
python3 python_lib/extract_pose_from_rosbag.py \
    --rosbag_path=$SAVE_DIR/result_rosbag \
    --target_topic="/localization/pose_estimator/pose" \
    --output_dir=$SAVE_DIR

# 評価
python3 python_lib/compare_trajectories.py \
    $SAVE_DIR/localization__pose_twist_fusion_filter__pose.tsv \
    $SAVE_DIR/awsim__ground_truth__vehicle__pose.tsv
python3 python_lib/compare_trajectories.py \
    $SAVE_DIR/localization__pose_estimator__pose.tsv \
    $SAVE_DIR/awsim__ground_truth__vehicle__pose.tsv

# exe_time_msをプロット
python3 python_lib/plot_exe_time_ms.py $SAVE_DIR/result_rosbag

# その他プロット
python3 python_lib/plot_localization_result.py $SAVE_DIR/result_rosbag
