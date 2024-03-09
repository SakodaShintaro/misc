#!/bin/bash
set -eux

# 現状のディレクトリがautowareまたはpilot-autoというプレフィックスを持つことを確認する
current_dir=$(basename $(pwd))
if [[ ! $current_dir =~ ^(autoware|pilot-auto) ]]; then
    echo "This script must be run in a directory with a prefix of autoware or pilot-auto."
    exit 1
fi

# ./autoware/src/launcher/autoware_launch/autoware_launch/config/planning/preset/default_preset.yaml
# ./autoware/src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/common/common.param.yaml

sed -i 's|input_traffic_light_topic_name" default="/perception/traffic_light_recognition/traffic_signals"|input_traffic_light_topic_name" default="/perception/traffic_light_recognition/traffic_signals_dummy"|' \
  ./src/universe/autoware.universe/launch/tier4_planning_launch/launch/scenario_planning/lane_driving/behavior_planning/behavior_planning.launch.xml
