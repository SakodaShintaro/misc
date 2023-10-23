#!/bin/bash
set -eux

TARGET_DIR=$1

# 現状のディレクトリがautowareまたはpilot-autoというプレフィックスを持つことを確認する
current_dir=$(basename $(pwd))
if [[ ! $current_dir =~ ^(autoware|pilot-auto) ]]; then
    echo "This script must be run in a directory with a prefix of autoware or pilot-auto."
    exit 1
fi

set +eux
export CPLUS_INCLUDE_PATH=/usr/include/c++/11:/usr/include/x86_64-linux-gnu/c++/11:$CPLUS_INCLUDE_PATH
set -eux

fdfind -e cpp -e hpp --full-path ${TARGET_DIR} | xargs -P $(nproc) -I{} clang-tidy -p build/ {}
