#!/bin/bash
set -eux

result_dir=$1

set +eux
source ~/autoware/install/setup.bash
set -eux

ros2 run autoware_localization_evaluation_scripts plot_diagnostics.py \
   $result_dir
