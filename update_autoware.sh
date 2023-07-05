#!/bin/bash

set -eux

git pull
vcs import src < autoware.repos
vcs pull src
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
