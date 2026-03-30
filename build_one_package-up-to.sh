#!/bin/bash

set -eux

# 現状のディレクトリがautowareまたはpilot-autoというプレフィックスを持つことを確認する
current_dir=$(basename $(pwd))
if [[ ! $current_dir =~ ^(autoware|pilot-auto) ]]; then
    echo "This script must be run in a directory with a prefix of autoware or pilot-auto."
    exit 1
fi

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to $1
