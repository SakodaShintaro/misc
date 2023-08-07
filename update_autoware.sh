#!/bin/bash

set -eux

git pull
vcs import src < autoware.repos
vcs import src < simulator.repos
vcs pull src
vcs export src --exact > my_autoware.repos
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
MAKEFLAGS="-j1" colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
cp $(dirname $0)/autoware.rviz ./src/launcher/autoware_launch/autoware_launch/rviz/autoware.rviz
