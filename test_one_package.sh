#!/bin/bash
set -eux

# [need]
# sudo apt-get install -y lcov python3-colcon-lcov-result

colcon build --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=ON \
    -DCMAKE_CXX_FLAGS='-fprofile-arcs -ftest-coverage' \
    -DCMAKE_C_FLAGS='-fprofile-arcs -ftest-coverage' \
  --packages-select $1
colcon test --return-code-on-test-failure --packages-select $1
colcon lcov-result --packages-select $1
colcon test-result --all --verbose | grep $1
