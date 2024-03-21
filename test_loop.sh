#!/bin/bash
set -eux

TEST_TARGET_PKG=$1
LOOP_NUM=${2:-100}

for ((i=1; i<=LOOP_NUM; i++))
do
  colcon test --packages-select ${TEST_TARGET_PKG} --event-handlers console_cohesion+ --return-code-on-test-failure
  echo "Finish test $i"
done
