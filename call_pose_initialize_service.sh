#!/bin/bash
set -eux

NUM=$1

# 読み込み
set +eux
source $HOME/autoware/install/setup.bash
set -eux

REQUEST_DATA="{
  pose: []
}"

for i in $(seq 1 $NUM); do
    echo "Sending request $i..."
    ros2 service call /localization/initialize autoware_adapi_v1_msgs/srv/InitializeLocalization "$REQUEST_DATA"
    sleep 1
done
