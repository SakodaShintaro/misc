#!/bin/bash
set -eu

TOPIC_NAME="/localization/initialization_state"

# トピックがpublishされるまで待機
while ! ros2 topic list | grep -q "$TOPIC_NAME"; do
  echo "Waiting for $TOPIC_NAME to be published..."
  sleep 1.0
done

# トピックからのメッセージを監視し、stateが3になったら再生速度を通常に戻す
ros2 topic echo $TOPIC_NAME | while read line; do
  if echo $line | grep -q "state: 3"; then
    echo "State is 3, setting playback rate to normal."
    ros2 service call /rosbag2_player/set_rate rosbag2_interfaces/SetRate "rate: 1.0"
    break
  fi
done
