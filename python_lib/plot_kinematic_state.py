import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import argparse
import matplotlib.pyplot as plt
import numpy as np
import pathlib
import pandas as pd


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('rosbag_path', type=pathlib.Path)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    rosbag_path = args.rosbag_path
    serialization_format = 'cdr'
    storage_options = rosbag2_py.StorageOptions(
        uri=str(rosbag_path), storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {
        topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    target_topics = [
        "/localization/pose_twist_fusion_filter/kinematic_state",
    ]
    storage_filter = rosbag2_py.StorageFilter(topics=target_topics)
    reader.set_filter(storage_filter)

    ekf_pose_list = list()

    while reader.has_next():
        (topic, data, timestamp_rosbag) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        timestamp_header = int(msg.header.stamp.sec) + \
            int(msg.header.stamp.nanosec) * 1e-9
        if topic == "/localization/pose_twist_fusion_filter/kinematic_state":
            pose = msg.pose.pose
            twist = msg.twist.twist
            ekf_pose_list.append({
                'timestamp': timestamp_header,
                'pose_x': pose.position.x,
                'pose_y': pose.position.y,
                'pose_z': pose.position.z,
                'quat_w': pose.orientation.w,
                'quat_x': pose.orientation.x,
                'quat_y': pose.orientation.y,
                'quat_z': pose.orientation.z,
                'twist_linear_x': twist.linear.x,
                'twist_linear_y': twist.linear.y,
                'twist_linear_z': twist.linear.z,
                'twist_angular_x': twist.angular.x,
                'twist_angular_y': twist.angular.y,
                'twist_angular_z': twist.angular.z,
            })
        else:
            assert False, f"Unknown topic: {topic}"

    df_ekf_pose = pd.DataFrame(ekf_pose_list)

    df_ekf_pose['timestamp'] -= df_ekf_pose['timestamp'].iloc[0]

    # rad to degree
    df_ekf_pose['twist_angular_x'] = np.rad2deg(df_ekf_pose['twist_angular_x'])
    df_ekf_pose['twist_angular_y'] = np.rad2deg(df_ekf_pose['twist_angular_y'])
    df_ekf_pose['twist_angular_z'] = np.rad2deg(df_ekf_pose['twist_angular_z'])

    # rosbag path may be the path to the db3 file, or it may be the path to the directory containing it
    save_dir = (rosbag_path.parent if rosbag_path.is_dir()
                else rosbag_path.parent.parent) / "kinematic_state"
    save_dir.mkdir(exist_ok=True)

    # plot
    plt.figure(figsize=(10, 10))
    plt.subplot(3, 2, 1)
    plt.plot(df_ekf_pose['timestamp'], df_ekf_pose['twist_linear_x'],
             label='twist_linear_x')
    plt.plot(df_ekf_pose['timestamp'], df_ekf_pose['twist_linear_y'],
             label='twist_linear_y')
    plt.plot(df_ekf_pose['timestamp'], df_ekf_pose['twist_linear_z'],
             label='twist_linear_z')
    plt.legend()
    plt.xlabel('times[s]')
    plt.ylabel('twist_linear[m/s]')
    plt.grid()

    plt.subplot(3, 2, 3)
    plt.plot(df_ekf_pose['timestamp'], df_ekf_pose['twist_angular_x'],
             label='twist_angular_x')
    plt.plot(df_ekf_pose['timestamp'], df_ekf_pose['twist_angular_y'],
             label='twist_angular_y')
    plt.plot(df_ekf_pose['timestamp'], df_ekf_pose['twist_angular_z'],
             label='twist_angular_z')
    plt.legend()
    plt.xlabel('times[s]')
    plt.ylabel('twist_angular[deg/s]')
    plt.grid()

    plt.subplot(3, 2, 5)
    plt.plot(df_ekf_pose['timestamp'], df_ekf_pose['pose_z'])
    plt.xlabel('times[s]')
    plt.ylabel('z[m]')
    plt.grid()

    plt.subplot(3, 2, 4)
    plt.plot(df_ekf_pose['pose_x'], df_ekf_pose['pose_y'])
    plt.xlabel('x[m]')
    plt.ylabel('y[m]')
    plt.axis('equal')
    plt.grid()
    # スタート位置にS, ゴール位置にGを表示
    plt.text(df_ekf_pose['pose_x'].iloc[0], df_ekf_pose['pose_y'].iloc[0], 'S',
             ha='center', va='bottom', fontsize=10, color='black')
    plt.text(df_ekf_pose['pose_x'].iloc[-1], df_ekf_pose['pose_y'].iloc[-1], 'G',
             ha='center', va='bottom', fontsize=10, color='black')

    plt.tight_layout()

    save_path = save_dir / "kinematic_state.png"
    plt.savefig(save_path, bbox_inches='tight', pad_inches=0.05)
    print(f"Saved to {save_path}")
