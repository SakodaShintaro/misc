""" AWSIMで作ったGT付きrosbagからGTを抽出するスクリプト
"""

import argparse
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import pandas as pd
import matplotlib.pyplot as plt


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('rosbag_path', type=str)
    parser.add_argument('output_dir', type=str)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    rosbag_path = args.rosbag_path
    output_dir = args.output_dir

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
    plt.savefig(f'{output_dir}/ground_truth_trajectory.png',
                bbox_inches='tight', pad_inches=0.05, dpi=300)
    plt.close()

    # plot differential
    plt.plot(df['x'].diff(), label='x')
    plt.plot(df['y'].diff(), label='y')
    plt.plot(df['z'].diff(), label='z')
    plt.plot(df['timestamp'].diff() / 1e9, label='timestamp')
    plt.xlabel("Frame number")
    plt.ylabel("diff (x,y,z[m], timestamp[sec])")
    plt.legend()
    plt.savefig(f'{output_dir}/ground_truth_differential.png',
                bbox_inches='tight', pad_inches=0.05, dpi=300)
    plt.close()

    df["sec"] = (df["timestamp"] * 1e-9).astype(int)
    df["nanosec"] = (df["timestamp"] % 1e9).astype(int)
    df = df.drop(columns=['timestamp'])
    df = df.reindex(columns=['sec', 'nanosec', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
    df.to_csv(f"{output_dir}/ground_truth_p.tsv", index=False, sep='\t')
    print(df)
