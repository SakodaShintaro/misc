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
$(dirname $0)/build_with_custom_jobs.sh 2
