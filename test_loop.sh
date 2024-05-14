#!/bin/bash
set -eux

TEST_TARGET_PKG=$1
LOOP_NUM=${2:-100}

colcon build --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=ON \
    -DCMAKE_CXX_FLAGS='-fprofile-arcs -ftest-coverage' \
    -DCMAKE_C_FLAGS='-fprofile-arcs -ftest-coverage' \
  --packages-select $TEST_TARGET_PKG

for ((i=1; i<=LOOP_NUM; i++))
do
  colcon test --packages-select ${TEST_TARGET_PKG} --event-handlers console_cohesion+ --return-code-on-test-failure
  echo "Finish test $i"
done
