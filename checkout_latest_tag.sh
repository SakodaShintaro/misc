#!/bin/bash

set -eu

ROSBAG_PATH=$1

# 取得
git fetch origin

# ROSバッグ情報を取得
BAG_INFO=$(ros2 bag info $ROSBAG_PATH)

# 開始日付を取得（'Start:'の行から）
START_DATE=$(echo "$BAG_INFO" | grep 'Start:' | awk '{print $2, $3, $4}')

# 日付の形式を変換（'Nov 14 2023'を'2023-11-14'に）
TARGET_DATE=$(date -d "$START_DATE" '+%Y-%m-%d')

# 変換された日付を出力
echo "target_date=$TARGET_DATE"

# リリースタグを取得
TAG_LIST=$(git tag --list --sort=-v:refname --format='%(refname:short)  %(creatordate:short)')
echo "tag_list"
echo "$TAG_LIST"

# 指定日付前のみを抽出
TAG_LIST_BEFORE=$(echo "$TAG_LIST" | awk -v date="$TARGET_DATE" '$2 <= date')

# 最新のリリースタグを取得
LATEST_TAG=$(echo "$TAG_LIST_BEFORE" | head -1 | cut -d ' ' -f 1)
echo ""
echo "latest_tag=$LATEST_TAG"

# チェックアウト
git checkout $LATEST_TAG

# コミット情報表示
git show -s
