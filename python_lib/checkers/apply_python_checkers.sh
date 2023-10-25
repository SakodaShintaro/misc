#!/bin/bash
set -eux

cd $(dirname $0)

find "." -type f -name "*.py" | while read -r file; do
    black "$file"
    flake8 "$file"
    mypy --strict "$file"
done
