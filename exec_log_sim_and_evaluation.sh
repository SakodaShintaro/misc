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

# rviz
LAUNCH_RVIZ=${5:-true}

# 読み込み
set +eux
source ./install/setup.bash
set -eux

# Autowareをバックグラウンドで起動
ros2 launch autoware_launch logging_simulator.launch.xml \
    map_path:=$MAP \
    pose_source:=$METHOD \
    vehicle_model:=sample_vehicle \
    sensor_model:=awsim_sensor_kit \
    rviz:=${LAUNCH_RVIZ} \
    perception:=false \
    planning:=false \
    control:=false \
    launch_system_monitor:=true &

# 立ち上がるまで待つ
# サービス呼び出しを立ち上がりの確認とする
ros2 service call /localization/pose_twist_fusion_filter/trigger_node std_srvs/srv/SetBool "{data: false}"

# 安定性のため少し待つ
sleep 5

# 保存先作成
mkdir -p $SAVE_DIR

# 保存
~/misc/record_localization_result.sh $SAVE_DIR/result_rosbag &

# rosbagをリプレイ
# 【以前のコマンド】
# ros2 bag play ${ROSBAG} -r 0.75 -s sqlite3 --remap /localization/pose_twist_fusion_filter/biased_pose_with_covariance:=/null
# 【データ入れ替え後のコマンド】
ros2 bag play ${ROSBAG} --rate 1.0 \
  --storage sqlite3 \
  --clock 200 \
  --remap /localization/pose_twist_fusion_filter/biased_pose_with_covariance:=/null

# 終了
~/misc/kill_autoware.sh

# gtが無ければgtを生成
if [ ! -e $SAVE_DIR/ground_truth.tsv ]; then
    python3 ~/misc/python_lib/extract_pose_from_rosbag.py \
        --rosbag_path=$ROSBAG \
        --target_topics="/awsim/ground_truth/localization/kinematic_state" \
        --output_dir=$SAVE_DIR
fi

# rosbagからtsvに変換
python3 ~/misc/python_lib/extract_pose_from_rosbag.py \
    --rosbag_path $SAVE_DIR/result_rosbag \
    --output_dir $SAVE_DIR \
    --target_topics "/localization/kinematic_state" \
                    "/localization/pose_twist_fusion_filter/pose" \
                    "/localization/pose_estimator/pose_with_covariance"

# 評価
python3 ~/misc/python_lib/compare_trajectories.py \
    $SAVE_DIR/localization__kinematic_state.tsv \
    $SAVE_DIR/awsim__ground_truth__localization__kinematic_state.tsv
python3 ~/misc/python_lib/compare_trajectories.py \
    $SAVE_DIR/localization__pose_twist_fusion_filter__pose.tsv \
    $SAVE_DIR/awsim__ground_truth__localization__kinematic_state.tsv
python3 ~/misc/python_lib/compare_trajectories.py \
    $SAVE_DIR/localization__pose_estimator__pose_with_covariance.tsv \
    $SAVE_DIR/awsim__ground_truth__localization__kinematic_state.tsv

# system_monitor_report
cd $SAVE_DIR
python3 ~/work/system-performance-evaluation/analysis/system_monitor_report/main.py $SAVE_DIR/result_rosbag/
cd -

# その他プロット
python3 ~/misc/python_lib/plot_localization_result.py $SAVE_DIR/result_rosbag
ros2 run autoware_localization_evaluation_scripts plot_diagnostics.py $SAVE_DIR/result_rosbag
