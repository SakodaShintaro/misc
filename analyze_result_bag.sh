#!/bin/bash
set -eux

target_dir=$(readlink -f $1)

# 読み込み
set +eux
source $HOME/autoware/install/setup.bash
set -eux

# target_dir以下にあるディレクトリ全てについて解析
result_bag_list=$(find $target_dir -name "result_bag" | sort)

for result_bag in $result_bag_list; do
  python3 ~/misc/python_lib/parse_rosbag_diagnostics.py $result_bag
  python3 ~/misc/python_lib/plot_localization_result.py $result_bag
done

# 結果をまとめる
target_file_name=("diagnostics_ekf_localizer.png" "diagnostics_ndt_scan_matcher.png" "diagnostics_pose_instability_detector.png")
for file_name in ${target_file_name[@]}; do
  without_ext=${file_name%.*}
  mkdir -p $target_dir/$without_ext
  for result_bag in $result_bag_list; do
    cp $result_bag/../diagnostics_result/$file_name $target_dir/$without_ext/${without_ext}_$(basename $(readlink -f $result_bag/../)).png
  done
done

target_file_name=("localization_result.png")
for file_name in ${target_file_name[@]}; do
  without_ext=${file_name%.*}
  mkdir -p $target_dir/$without_ext
  for result_bag in $result_bag_list; do
    cp $result_bag/../localization_result/$file_name $target_dir/$without_ext/${without_ext}_$(basename $(readlink -f $result_bag/../)).png
  done
done
