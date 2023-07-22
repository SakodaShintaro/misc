#!/bin/bash

set -eux

# SCRIPT_PATH=$(readlink -f $0)
# SCRIPT_DIR=$(dirname ${SCRIPT_PATH})
# TARGET_DIR=${SCRIPT_DIR}/../

clang-format -i $(find ${TARGET_DIR} -name "*.hpp")
clang-format -i $(find ${TARGET_DIR} -name "*.cpp")
