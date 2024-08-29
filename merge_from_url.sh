#!/bin/bash

# 引数からGitHubの完全なURLを取得
URL=$1
FORCE=${2:-false}

# URLからGitHubのベースURL、リポジトリ名、ブランチ名を抽出
GITHUB_URL=$(echo "$URL" | grep -oP 'https://github.com/\K[^/]+/[^/]+')
BRANCH_NAME=$(echo "$URL" | grep -oP 'tree/\K.*' | sed 's/\//\//g')

# リポジトリ名とユーザ名を抽出
REPO_NAME=$(echo "$GITHUB_URL" | cut -d'/' -f2)
USER_NAME=$(echo "$GITHUB_URL" | cut -d'/' -f1)

# 現在のディレクトリ名を取得
CURRENT_DIR_NAME=$(basename "$(pwd)")

# 現在のディレクトリ名とREPO_NAMEが一致するか確認
if [ "$FORCE" != "true" ] && [ "$CURRENT_DIR_NAME" != "$REPO_NAME" ]; then
    echo "Error: Current directory does not match the repository name ($REPO_NAME)."
    exit 1
fi

# リモート名を設定（ユーザ名をリモート名として使用）
REMOTE_NAME="$USER_NAME"

# このリモートが既に存在するか確認
REMOTE_EXISTS=$(git remote | grep "^${REMOTE_NAME}$" | wc -l)

# リモートが存在しない場合は追加
if [ "$REMOTE_EXISTS" -eq "0" ]; then
    echo "Adding remote '$REMOTE_NAME' with URL 'https://github.com/$GITHUB_URL.git'"
    git remote add "$REMOTE_NAME" "https://github.com/$GITHUB_URL.git"
fi

# リモートから最新情報をフェッチ
git fetch "$REMOTE_NAME"

# マージ
git merge --no-ff --no-edit "$REMOTE_NAME/$BRANCH_NAME"
