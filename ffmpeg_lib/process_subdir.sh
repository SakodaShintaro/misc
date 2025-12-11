#!/bin/bash

set -eux

target_dir=$(readlink -f "$1")
script_path=$(readlink -f "$(dirname "$0")/make_mp4_from_unsequential_png.sh")

declare -A processed_dirs=()

# findからの出力を逐次処理し、同じディレクトリは1度だけ処理する
while IFS= read -r -d '' png_file; do
    dir=$(dirname "$png_file")
    if [[ -n "${processed_dirs[$dir]+x}" ]]; then
        continue
    fi
    processed_dirs["$dir"]=1
    bash "$script_path" "$dir"
done < <(find "$target_dir" -type f -name '*.png' -print0)
