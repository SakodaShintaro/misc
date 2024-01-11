""" rosbagから特定トピックのタイムスタンプを調べるスクリプト
"""

import argparse
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('rosbag_path', type=str)
    parser.add_argument('target_topic', type=str)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    rosbag_path = args.rosbag_path
    target_topic = args.target_topic

    reader = Reader(rosbag_path)
    reader.open()
    timestamp_rosbag_list = []
    timestamp_msg_list = []
    for connection, timestamp, raw_data in reader.messages():
        topic_name = connection.topic
        if topic_name != target_topic:
            continue
        msg = deserialize_cdr(raw_data, connection.msgtype)
        timestamp /= 1e9
        timestamp_msg = msg.header.stamp
        timestamp_msg = timestamp_msg.sec + timestamp_msg.nanosec / 1e9
        timestamp_rosbag_list.append(timestamp)
        timestamp_msg_list.append(timestamp_msg)

    timestamp_msg_list = np.array(timestamp_msg_list)
    timestamp_rosbag_list = np.array(timestamp_rosbag_list)
    diff = (timestamp_rosbag_list - timestamp_msg_list) * 1000

    # plot
    plt.plot(diff, label='timestamp')
    plt.title(target_topic)
    plt.xlabel("Frame number")
    plt.ylabel("diff timestamp[msec]")
    plt.show()
