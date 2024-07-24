#!/bin/bash
set -eux

target_dir=$(readlink -f $1)

# 読み込み
set +eux
source $HOME/autoware/install/setup.bash
set -eux

set +e

# target_dir以下にあるディレクトリ全てについて解析
dir_list=$(find $target_dir -mindepth 1 -maxdepth 1 -type d | sort)

for dir in $dir_list; do
  python3 ~/misc/python_lib/parse_rosbag_diagnostics.py $dir/result_bag
  python3 ~/misc/python_lib/plot_localization_result.py $dir/result_bag
done
