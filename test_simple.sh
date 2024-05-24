#!/bin/bash
set -eux

TEST_TARGET_PKG=$1

colcon build --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=ON \
  --packages-select ${TEST_TARGET_PKG}

colcon test --event-handlers console_cohesion+ \
  --return-code-on-test-failure \
  --packages-select ${TEST_TARGET_PKG}
