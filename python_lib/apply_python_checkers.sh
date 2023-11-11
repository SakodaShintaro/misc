#!/bin/bash
set -eux

pwd

find "." -type f -name "*.py" | while read -r file; do
    black "$file"
    isort "$file"
    flake8 --ignore=Q000,I100,I201,E203 --max-line-length=88 "$file"
    mypy --strict --ignore-missing-imports "$file"
done
