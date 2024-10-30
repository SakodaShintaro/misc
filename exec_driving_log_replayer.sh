#!/bin/bash
set -eux

version=${1:-1}

if [ $version -eq 1 ]; then
    dlr simulation run -p localization -l "play_rate:=1.0"
elif [ $version -eq 2 ]; then
    set +eux
    source ~/autoware/install/setup.bash
    set -eux
    ros2 launch driving_log_replayer_v2 driving_log_replayer_v2.launch.py scenario_path:=$HOME/log_evaluator_data/localization/sample/scenario.yaml
fi
