#!/bin/bash

set -eux

target_dir=$(readlink -f $1)
script_path=$(readlink -f $(dirname $0)/make_mp4_from_unsequential_png.sh)

# target_dir以下のすべてのpngファイルの親ディレクトリの集合を作成
dir_set=$(find "$target_dir" -type f -name '*.png' -exec dirname {} \; | sort | uniq)

for dir in $dir_set; do
    bash "$script_path" "$dir"
done
