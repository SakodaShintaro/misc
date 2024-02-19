""" rosbagからPoseStampedのPoseを抽出するスクリプト
"""

import argparse
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import pandas as pd
import matplotlib.pyplot as plt
import os


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rosbag_path', type=str, required=True)
    parser.add_argument('--target_topic', type=str, required=True)
    parser.add_argument('--output_dir', type=str, required=True)
    return parser.parse_args()


def extract_pose_data(msg, msg_type):
    if msg_type == 'geometry_msgs/msg/Pose':
        return msg
    elif msg_type == 'geometry_msgs/msg/PoseStamped':
        return msg.pose
    elif msg_type == 'geometry_msgs/msg/PoseWithCovarianceStamped':
        return msg.pose.pose
    elif msg_type == 'nav_msgs/msg/Odometry':
        return msg.pose.pose
    else:
        raise ValueError(f"Unsupported message type: {msg_type}")


if __name__ == "__main__":
    args = parse_args()
    rosbag_path = args.rosbag_path
    target_topic = args.target_topic
    output_dir = args.output_dir

    os.makedirs(output_dir, exist_ok=True)

    reader = Reader(rosbag_path)
    reader.open()
    df = pd.DataFrame(columns=['timestamp', 'sec', 'nanosec', 'x', 'y',
                      'z', 'qw', 'qx', 'qy', 'qz'])
    for connection, _timestamp, raw_data in reader.messages():
        topic_name = connection.topic
        if topic_name != target_topic:
            continue
        msg = deserialize_cdr(raw_data, connection.msgtype)
        sec = msg.header.stamp.sec
        nanosec = msg.header.stamp.nanosec
        timestamp = sec + nanosec / 1e9
        pose = extract_pose_data(msg, connection.msgtype)
        new_row = pd.DataFrame([{
            'timestamp': timestamp,
            'sec': sec,
            'nanosec': nanosec,
            'x': pose.position.x,
            'y': pose.position.y,
            'z': pose.position.z,
            'qw': pose.orientation.w,
            'qx': pose.orientation.x,
            'qy': pose.orientation.y,
            'qz': pose.orientation.z,
        }])
        df = pd.concat([df, new_row], ignore_index=True)

    save_name = "__".join(target_topic.split('/')[1:])

    # plot xy
    plt.plot(df['x'], df['y'])
    plt.axis('equal')
    plt.xlabel('x[m]')
    plt.ylabel('y[m]')
    plt.savefig(f'{output_dir}/{save_name}_trajectory.png',
                bbox_inches='tight', pad_inches=0.05, dpi=300)
    plt.close()

    # plot differential
    plt.plot(df['x'].diff(), label='x')
    plt.plot(df['y'].diff(), label='y')
    plt.plot(df['z'].diff(), label='z')
    plt.plot(df['timestamp'].diff(), label='timestamp')
    plt.xlabel("Frame number")
    plt.ylabel("diff (x,y,z[m], timestamp[sec])")
    plt.legend()
    plt.savefig(f'{output_dir}/{save_name}_differential.png',
                bbox_inches='tight', pad_inches=0.05, dpi=300)
    plt.close()

    df = df.drop(columns=['timestamp'])
    df = df.reindex(columns=['sec', 'nanosec', 'x',
                    'y', 'z', 'qx', 'qy', 'qz', 'qw'])
    df.to_csv(f"{output_dir}/{save_name}.tsv", index=False, sep='\t', float_format='%.9f')
