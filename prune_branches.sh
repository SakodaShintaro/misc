#!/bin/bash
# Delete local branches that no longer exist on origin.
#
# Usage:
#   delete_local_only_branches.sh [-n]
#     -n  dry run (only show what would be deleted)

set -euo pipefail

dry_run=false

while getopts "n" opt; do
    case "${opt}" in
        n) dry_run=true ;;
        *) echo "usage: $(basename "$0") [-n]" >&2; exit 1 ;;
    esac
done

git rev-parse --is-inside-work-tree > /dev/null

echo "Fetching origin (with prune)..."
git fetch --prune origin

current=$(git rev-parse --abbrev-ref HEAD)

# Branch names that still exist on origin.
remote_branches=$(git for-each-ref --format='%(refname:strip=3)' refs/remotes/origin/ | grep -v '^HEAD$' || true)

to_delete=()
while read -r branch; do
    [ -n "${branch}" ] || continue
    if grep -qxF "${branch}" <<< "${remote_branches}"; then
        continue
    fi
    if [ "${branch}" = "${current}" ]; then
        echo "skip: '${branch}' is the current branch"
        continue
    fi
    to_delete+=("${branch}")
done < <(git for-each-ref --format='%(refname:short)' refs/heads/)

if [ "${#to_delete[@]}" -eq 0 ]; then
    echo "Nothing to delete."
    exit 0
fi

echo
echo "Local branches without a counterpart on origin:"
for branch in "${to_delete[@]}"; do
    echo "  ${branch}  ($(git log -1 --format='%h %s' "${branch}"))"
done
echo

if "${dry_run}"; then
    echo "Dry run: nothing was deleted."
    exit 0
fi

for branch in "${to_delete[@]}"; do
    git branch -D "${branch}"
done
