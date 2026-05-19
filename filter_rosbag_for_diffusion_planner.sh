#!/bin/bash

set -eux

# 読み込み
set +eux
source $HOME/autoware/install/setup.bash
set -eux

TARGET=$(readlink -f $1)

ros2 bag filter ${TARGET} -o ${TARGET}_filtered --include \
    "/localization/kinematic_state" \
    "/localization/acceleration" \
    "/perception/object_recognition/tracking/objects" \
    "/planning/mission_planning/route" \
    "/vehicle/status/turn_indicators_status" \
    "/perception/traffic_light_recognition/traffic_signals" \
    "/planning/planning_factors/diffusion_planner" \
    "/planning/trajectory_generator/neural_network_based_planner/diffusion_planner_node/debug/lane_marker" \
    "/planning/trajectory_generator/neural_network_based_planner/diffusion_planner_node/debug/linestring_marker" \
    "/planning/trajectory_generator/neural_network_based_planner/diffusion_planner_node/debug/route_marker" \
    "/planning/trajectory_generator/neural_network_based_planner/diffusion_planner_node/output/predicted_objects" \
    "/planning/trajectory_generator/neural_network_based_planner/diffusion_planner_node/output/trajectory" \
    "/tf" \
    "/perception/obstacle_segmentation/pointcloud" \
    "/autoware/state" \
    "/planning/planning_factors/diffusion_planner" \
    "/planning/planning_factors/diffusion_planner" \
    "/planning/planning_factors/modifier_obstacle_stop" \
    "/planning/planning_factors/trajectory_validator"
