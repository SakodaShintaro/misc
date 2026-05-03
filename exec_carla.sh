#!/bin/bash
set -eux

# スクリプトが終了する際にすべてのバックグラウンドプロセスを停止するためのトラップを設定
trap "kill 0" EXIT

# 読み込み
set +eux
source $HOME/autoware/install/setup.bash
set -eux

ros2 launch autoware_launch e2e_simulator.launch.xml \
  map_path:=$HOME/autoware_data/maps/odaibatest \
  vehicle_model:=sample_vehicle \
  sensor_model:=carla_sensor_kit \
  simulator_type:=carla \
  use_e2e_planning:=false \
  planning_setting:=diffusion_planner
