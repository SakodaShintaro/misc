""" AWSIMで作ったGT付きrosbagの正常性をチェックするためのスクリプト
"""

import argparse
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import pandas as pd
import matplotlib.pyplot as plt


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('rosbag_path', type=str)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    rosbag_path = args.rosbag_path

    reader = Reader(rosbag_path)
    reader.open()
    df = pd.DataFrame(columns=['timestamp', 'x', 'y',
                      'z', 'qw', 'qx', 'qy', 'qz'])
    for connection, timestamp, raw_data in reader.messages():
        topic_name = connection.topic
        if topic_name != "/awsim/ground_truth/vehicle/pose":
            continue
        msg = deserialize_cdr(raw_data, connection.msgtype)
        df = df.append({
            'timestamp': timestamp,
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z,
            'qw': msg.pose.orientation.w,
            'qx': msg.pose.orientation.x,
            'qy': msg.pose.orientation.y,
            'qz': msg.pose.orientation.z,
        }, ignore_index=True)

    print(df.head())

    # plot xy
    plt.plot(df['x'], df['y'])
    plt.axis('equal')
    plt.xlabel('x[m]')
    plt.ylabel('y[m]')
    plt.show()
    plt.close()

    # plot differential
    plt.plot(df['x'].diff(), label='x')
    plt.plot(df['y'].diff(), label='y')
    plt.plot(df['z'].diff(), label='z')
    plt.plot(df['timestamp'].diff() / 1e9, label='timestamp')
    plt.legend()
    plt.show()
    plt.close()
