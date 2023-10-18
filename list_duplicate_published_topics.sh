#!/bin/bash

TOPIC_LIST=$(ros2 topic list)

for topic in $TOPIC_LIST; do
  msg=$(ros2 topic info -v $topic)
  publisher_count=$(echo "$msg" | awk -F': ' '/Publisher count:/ {print $2}')

  if (( publisher_count >= 2 )); then
    echo "  $topic:=/null \\"
  fi
done
