#!/bin/bash

set -eux

JOB_COUNT=${1:-2}

$(dirname $0)/reset_autoware.sh
git checkout main
git pull
vcs import src < autoware.repos
vcs import src < simulator.repos
vcs import src < tools.repos
vcs pull src
vcs export src --exact > my_autoware_$(date +"%Y%m%d").repos
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
$(dirname $0)/build_with_custom_jobs.sh ${JOB_COUNT}
$(dirname $0)/change_autoware_for_localization.sh
