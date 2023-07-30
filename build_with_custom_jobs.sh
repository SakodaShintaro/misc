#!/bin/bash

set -eux

JOB_COUNT=$1

# 現状のディレクトリがautowareというプレフィックスを持つことを確認する
if [[ ! $(basename $(pwd)) =~ ^autoware ]]; then
    echo "This script must be run in a directory with a prefix of autoware."
    exit 1
fi

MAKEFLAGS="-j${JOB_COUNT}" colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release