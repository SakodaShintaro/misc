#!/bin/bash
set -eux

# 現状のディレクトリがautowareまたはpilot-autoというプレフィックスを持つことを確認する
current_dir=$(basename $(pwd))
if [[ ! $current_dir =~ ^(autoware|pilot-auto) ]]; then
    echo "This script must be run in a directory with a prefix of autoware or pilot-auto."
    exit 1
fi

sed -i '/launch_traffic_light_module/,/default/s/true/false/' \
  ./src/launcher/autoware_launch/autoware_launch/config/planning/preset/default_preset.yaml

sed -i 's/max_vel: 4.17/max_vel: 22.2/' \
  ./src/launcher/autoware_launch/autoware_launch/config/planning/scenario_planning/common/common.param.yaml

cd ./src/sensor_kit/external/awsim_sensor_kit_launch/
~/misc/merge_from_url.sh https://github.com/knzo25/awsim_sensor_kit_launch/tree/chore/imu_corrector_refactor
