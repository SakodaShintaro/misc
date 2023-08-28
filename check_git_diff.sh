#!/bin/bash

function check_git_diff() {
  local dir=$1
  if [ -d "$dir/.git" ]; then
    # echo "Checking $dir"
    git_diff_output=$(git -C "$dir" diff --name-only)

    if [ -n "$git_diff_output" ]; then
      echo "Diff found in $(readlink -f $dir)"
      echo "--$git_diff_output"
    fi
  fi
}

function traverse() {
  for d in "$1"/*; do
    if [ -d "$d" ]; then
      check_git_diff "$d"
      traverse "$d"
    fi
  done
}

# 再帰的に調査
traverse "./"
