#!/bin/bash
set -eux

# VSCodeにバンドルされているClaude Codeを使用する
binaries=$(find ~/.vscode/extensions/ -name "claude" -type f -executable)
if [ -z "$binaries" ]; then
    binaries=$(find ~/.vscode-server/extensions/ -name "claude" -type f -executable)
fi

if [ -z "$binaries" ]; then
    echo "Claude executable not found in VSCode extensions."
    exit 1
fi

latest_binary=$(echo "$binaries" | sort -V | tail -n 1)
echo "Using Claude executable: $latest_binary"

$latest_binary --dangerously-skip-permissions
