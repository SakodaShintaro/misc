#!/bin/bash
set -eux

root_dir=$(readlink -f $1)
target_filename=$2

path_list=$(find $root_dir -name $target_filename | sort)

if [ $(echo $path_list | wc -w) -eq 0 ]; then
    echo "No file found."
    exit 1
fi

save_dir=$root_dir/flattened
mkdir -p $save_dir

for path in $path_list; do
    # pathの2つ上のディレクトリ名を取得
    prefix=$(basename $(dirname $(dirname $path)))
    cp $path $save_dir/${prefix}_${target_filename}
done
