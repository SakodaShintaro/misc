#!/bin/bash
set -eux

USE_NIGHTLY=${1:-true}

# 現状のディレクトリがautowareというプレフィックスを持つことを確認する
current_dir=$(basename $(pwd))
if [[ ! $current_dir =~ ^(autoware) ]]; then
    echo "This script must be run in a directory with a prefix of autoware."
    exit 1
fi

$(dirname $0)/reset_autoware.sh
git checkout main
git pull
vcs import src < autoware.repos
vcs import src < simulator.repos
vcs import src < tools.repos
if [ $USE_NIGHTLY = true ]; then
    vcs import src < autoware-nightly.repos
    vcs import src < simulator-nightly.repos
    vcs import src < tools-nightly.repos
fi
vcs pull src
vcs export src --exact > my_autoware_$(date +"%Y%m%d").repos
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
$(dirname $0)/build_with_custom_jobs.sh 2
$(dirname $0)/change_autoware_for_localization.sh
