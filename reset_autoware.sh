#!/bin/bash

# 現状のディレクトリがautowareというプレフィックスを持つことを確認する
if [[ ! $(basename $(pwd)) =~ ^autoware ]]; then
    echo "This script must be run in a directory with a prefix of autoware."
    exit 1
fi

# .gitを全てreset
REPOSITORIES=$(find . -type d -name ".git")
for git_dir in $REPOSITORIES; do
    repo_dir=$(dirname "$git_dir")
    (
        cd "$repo_dir"
        git reset --hard HEAD
        git clean -df
    )
done

# 変更ファイルを差し戻す
cp $(dirname $0)/simulator.repos ./
cp $(dirname $0)/autoware.rviz ./src/launcher/autoware_launch/autoware_launch/rviz/autoware.rviz
