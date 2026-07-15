#!/bin/bash
set -eux

binaries=""
if [ -d ~/.vscode/extensions ]; then
    binaries="$binaries"$'\n'$(find ~/.vscode/extensions/ -name "claude" -type f -executable)
fi
if [ -d ~/.vscode-server/extensions ]; then
    binaries="$binaries"$'\n'$(find ~/.vscode-server/extensions/ -name "claude" -type f -executable)
fi
binaries=$(echo "$binaries" | grep -v '^$' || true)

if [ -z "$binaries" ]; then
    echo "Claude executable not found in VSCode extensions."
    exit 1
fi

latest_binary=$(echo "$binaries" | sort -V | tail -n 1)
echo "Using Claude executable: $latest_binary"

$latest_binary --dangerously-skip-permissions
