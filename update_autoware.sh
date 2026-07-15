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
vcs import --recursive src < repositories/autoware.repos
vcs import --recursive src < repositories/simulator.repos
vcs import --recursive src < repositories/tools.repos
if [ $USE_NIGHTLY = true ]; then
    vcs import --recursive src < repositories/autoware-nightly.repos
    vcs import --recursive src < repositories/simulator-nightly.repos
    vcs import --recursive src < repositories/tools-nightly.repos
fi
# 公式 repositories/simulator.repos に含まれない perception_eval / driving_log_replayer 等を追加で取得する
vcs import --recursive src < $(dirname $0)/simulator.repos
vcs pull src
vcs export src --exact > my_autoware_$(date +"%Y%m%d").repos
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
$(dirname $0)/build_with_custom_jobs.sh 1
