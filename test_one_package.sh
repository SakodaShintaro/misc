#!/bin/bash
set -eux

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON --packages-select $1
colcon test --packages-select $1
colcon test-result --all --verbose | grep $1
