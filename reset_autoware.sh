#!/bin/bash

# 現状のディレクトリがautowareまたはpilot-autoというプレフィックスを持つことを確認する
current_dir=$(basename $(pwd))
if [[ ! $current_dir =~ ^(autoware|pilot-auto) ]]; then
    echo "This script must be run in a directory with a prefix of autoware or pilot-auto."
    exit 1
fi

# .gitを全てreset
REPOSITORIES=$(find . -type d -name ".git")
for git_dir in $REPOSITORIES; do
    repo_dir=$(dirname "$git_dir")
    (
        cd "$repo_dir"
        git reset --hard @{u}
        git clean -df
        # default_branch=$(git symbolic-ref refs/remotes/origin/HEAD | sed 's@^refs/remotes/origin/@@')
        # git checkout "$default_branch"
        # git pull
    )
done

git reset --hard @{u}
git clean -df

# 変更ファイルを差し戻す
if [[ $current_dir =~ ^(autoware) ]]; then
    cat $(dirname $0)/simulator.repos >> ./simulator.repos
fi
