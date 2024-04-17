#!/bin/bash
set -eux

# 実行するディレクトリ
TARGET_DIR=$(readlink -f $1)
# 保存先
SAVE_DIR=$(readlink -f $2)

# 読み込み
set +eux
source $HOME/autoware/install/setup.bash
set -eux

# exec_general_logging_simulator.shをTARGET_DIRのサブディレクトリ全てについて回す
dir_list=$(find $TARGET_DIR -mindepth 1 -maxdepth 1 -type d)
dir_list=$(echo "$dir_list" | sort)
for dir in $dir_list; do
  NAME=$(basename $dir)
  MAP=$(readlink -f $dir/map)
  ROSBAG=$(readlink -f $dir/input_bag)
  # 内部で実行されるkill 0で全体が止まらないようにする
  (
    set -m
    ~/misc/exec_general_logging_simulator.sh $MAP $ROSBAG $SAVE_DIR/$NAME || true
  )
done
