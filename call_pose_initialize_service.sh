#!/bin/bash
set -eux

# 読み込み
set +eux
source $HOME/autoware/install/setup.bash
set -eux

REQUEST_DATA="{
  pose: []
}"

for i in {1..99}; do
    echo "Sending request $i..."
    ros2 service call /localization/initialize autoware_adapi_v1_msgs/srv/InitializeLocalization "$REQUEST_DATA"
    sleep 1
done
