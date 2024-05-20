#!/bin/bash

set -eux

# 実行するmap・rosbagへのパス
MAP=$(readlink -f $1)
ROSBAG=$(readlink -f $2)
SAVE_DIR=${3:-$HOME/data/misc/$(date +"%Y%m%d_%H%M%S")_convergence_evaluator}

mkdir -p $SAVE_DIR
SAVE_DIR=$(realpath $SAVE_DIR)

# 読み込み
set +eux
source $HOME/autoware/install/setup.bash
set -eux

# 実行
ros2 launch convergence_evaluator convergence_evaluator.launch.xml \
  map_path:=$MAP \
  input_bag_file_path:=$ROSBAG \
  result_path:=$SAVE_DIR

# SAVE_DIR以下に20240520171301という形で保存される
# それを直接知る術がないので、妥協案として最新のディレクトリを取得する
SAVE_DIR=$(ls -td $SAVE_DIR/* | head -n 1)

# 可視化
ros2 run convergence_evaluator convergence_result_analyzer.py \
  --input_file_path=$SAVE_DIR \
  --map=$MAP
