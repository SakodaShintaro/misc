#!/bin/bash
set -eux

TARGET_ROSBAG=$(readlink -f $1)
TARGET_DIR=$(dirname $TARGET_ROSBAG)

echo "
output_bags:
- uri: /${TARGET_DIR}_mcap
  storage_id: mcap
  all: true
" > config.yaml

ros2 bag convert -i $TARGET_ROSBAG -o config.yaml

rm config.yaml
