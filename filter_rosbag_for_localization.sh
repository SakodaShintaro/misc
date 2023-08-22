#!/bin/bash

set -eux

# 読み込み
set +eux
source $HOME/autoware/install/setup.bash
set -eux

TARGET=$(readlink -f $1)

ros2 bag filter ${TARGET} -o ${TARGET}_filtered -x \
    "/planning/.*" \
    "/control/.*" \
    "/perception/.*" \
    "/localization/.*" \
    "/sensing/lidar/concatenated/pointcloud" \
    "/tf"
