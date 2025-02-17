#!/bin/bash

set -eux

# スクリプトが終了する際にすべてのバックグラウンドプロセスを停止するためのトラップを設定
trap "kill 0" EXIT

# 読み込み
set +eux
source ./install/setup.bash
set -eux

ros2 launch scenario_test_runner scenario_test_runner.launch.py \
  architecture_type:=awf/universe/20250130 \
  record:=false \
  scenario:='$(find-pkg-share scenario_test_runner)/scenario/sample.yaml' \
  sensor_model:=sample_sensor_kit \
  vehicle_model:=sample_vehicle

# logを確認
dir_path=$(find ~/.ros/log -maxdepth 1 -type d | sort | tail -n 1)
log_path=$dir_path/launch.log
cat $log_path | grep "ERROR"
cat $log_path | grep "died"
