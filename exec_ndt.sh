#!/bin/bash
set -eux

~/misc/exec_log_sim_and_evaluation.sh \
  $HOME/Downloads/nishishinjuku_autoware_map_divided \
  $HOME/data/rosbag/AWSIM/20240814_awsim_3types/20240730_short/ \
  ndt \
  $HOME/data/misc/$(date +"%Y%m%d_%H%M%S")_ndt
