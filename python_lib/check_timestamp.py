""" rosbagから特定トピックのタイムスタンプを調べるスクリプト
"""

import argparse
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import pandas as pd
import matplotlib.pyplot as plt
import os


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
    df = pd.DataFrame(columns=['timestamp'])
    for connection, timestamp, raw_data in reader.messages():
        topic_name = connection.topic
        if topic_name != target_topic:
            continue
        msg = deserialize_cdr(raw_data, connection.msgtype)
        df = df.append({'timestamp': timestamp}, ignore_index=True)
        if len(df) >= 500:
            break

    print(df.head())

    # plot differential
    plt.plot(df['timestamp'].diff() / 1e9, label='timestamp')
    plt.title(target_topic)
    plt.xlabel("Frame number")
    plt.ylabel("diff timestamp[sec]")
    plt.legend()
    plt.show()
