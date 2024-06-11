#!/bin/bash

TOPIC_LIST=$(ros2 topic list)

for topic in $TOPIC_LIST; do
  # /clockは除外
  if [[ $topic == "/clock" ]]; then
    continue
  fi

  msg=$(ros2 topic info -v $topic)
  # msgに
  #   Publisher count: 1
  #   Node name: rosbag2_player
  # というメッセージがある場合のみ表示
  if [[ $msg =~ "Publisher count: 1" ]] && [[ $msg =~ "Node name: rosbag2_player" ]]; then
    echo "  $topic \\"
  fi

done
