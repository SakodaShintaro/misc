#!/bin/bash

TOPIC_LIST=$(ros2 topic list)

for topic in $TOPIC_LIST; do
  msg=$(ros2 topic info -v $topic)
  if [[ $msg =~ "Publisher count: 2" ]]; then
    echo "  $topic:=/null \\"
    # echo $msg
    # echo ""
  fi
done
