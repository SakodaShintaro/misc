#!/bin/bash

set -eux

# 現状のディレクトリがautowareというプレフィックスを持つことを確認する
if [[ ! $(basename $(pwd)) =~ ^autoware ]]; then
    echo "This script must be run in a directory with a prefix of autoware."
    exit 1
fi

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to $1
