#!/bin/bash

set -eux

$(dirname $0)/reset_autoware.sh
git pull
vcs import src < autoware.repos
vcs import src < simulator.repos
vcs pull src
vcs export src --exact > my_autoware.repos
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
MAKEFLAGS="-j2" colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
$(dirname $0)/check_git_diff.sh
