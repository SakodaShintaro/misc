#!/bin/bash
set -eux

target_dir=$(readlink -f $1)
suffix=$2

file_list=$(find $target_dir -name "*.png" | sort)

for file in $file_list; do
    new_file="${file%.png}_$suffix.png"
    mv "$file" "$new_file"
done
