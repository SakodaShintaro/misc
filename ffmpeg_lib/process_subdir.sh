#!/bin/bash

set -eux

target_dir=$(readlink -f $1)
script_path=$(readlink -f $(dirname $0)/make_mp4_from_unsequential_png.sh)

target_dir_list=$(find $target_dir -mindepth 1 -maxdepth 1 -type d | sort)

for dir in $target_dir_list; do
    bash $script_path $dir
done
