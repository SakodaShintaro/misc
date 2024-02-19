import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import argparse
import matplotlib.pyplot as plt
import numpy as np
import pathlib
import pandas as pd
from scipy.spatial.transform import Rotation, Slerp


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('rosbag_path', type=pathlib.Path)
    return parser.parse_args()


def interpolate_pose(df_pose: pd.DataFrame, target_timestamp: pd.Series) -> pd.DataFrame:
    """ Interpolate each pose in df_pose to match the timestamp in target_timestamp
    Constraints)
    * df_pose and target_timestamp must be sorted by timestamp
    * df_pose must have timestamps with a larger interval than target_timestamp
      * i.e. df_pose[0] <= target_timestamp[0] and target_timestamp[-1] <= df_pose[-1]
    * len(df_pose) > len(target_timestamp)
    出力)
    * DataFrame with same columns as df_pose and length same as target_timestamp
    """
    POSITIONS_KEY = ['x', 'y', 'z']
    ORIENTATIONS_KEY = ['qw', 'qx', 'qy', 'qz']
    target_index = 0
    df_index = 0
    data_dict = {
        'x': [],
        'y': [],
        'z': [],
        'qx': [],
        'qy': [],
        'qz': [],
        'qw': [],
        'timestamp': [],
    }
    while df_index < len(df_pose) - 1 and target_index < len(target_timestamp):
        curr_time = df_pose.iloc[df_index]['timestamp']
        next_time = df_pose.iloc[df_index + 1]['timestamp']
        target_time = target_timestamp[target_index]

        # Find a df_index that includes target_time
        if not (curr_time <= target_time <= next_time):
            df_index += 1
            continue

        curr_weight = (next_time - target_time) / (next_time - curr_time)
        next_weight = 1.0 - curr_weight

        curr_position = df_pose.iloc[df_index][POSITIONS_KEY]
        next_position = df_pose.iloc[df_index + 1][POSITIONS_KEY]
        target_position = curr_position * curr_weight + next_position * next_weight

        curr_orientation = df_pose.iloc[df_index][ORIENTATIONS_KEY]
        next_orientation = df_pose.iloc[df_index + 1][ORIENTATIONS_KEY]
        curr_r = Rotation.from_quat(curr_orientation)
        next_r = Rotation.from_quat(next_orientation)
        slerp = Slerp([curr_time, next_time],
                      Rotation.concatenate([curr_r, next_r]))
        target_orientation = slerp([target_time]).as_quat()[0]

        data_dict['timestamp'].append(target_timestamp[target_index])
        data_dict['x'].append(target_position[0])
        data_dict['y'].append(target_position[1])
        data_dict['z'].append(target_position[2])
        data_dict['qw'].append(target_orientation[0])
        data_dict['qx'].append(target_orientation[1])
        data_dict['qy'].append(target_orientation[2])
        data_dict['qz'].append(target_orientation[3])
        target_index += 1
    result_df = pd.DataFrame(data_dict)
    return result_df


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
        "/localization/pose_twist_fusion_filter/pose_instability_detector/debug/diff_pose",
        "/localization/pose_twist_fusion_filter/kinematic_state",
    ]
    storage_filter = rosbag2_py.StorageFilter(topics=target_topics)
    reader.set_filter(storage_filter)

    pose_instability_diff_pose_list = list()
    ekf_pose_list = list()

    while reader.has_next():
        (topic, data, timestamp_rosbag) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        timestamp_header = int(msg.header.stamp.sec) + \
            int(msg.header.stamp.nanosec) * 1e-9
        if topic == "/localization/pose_twist_fusion_filter/pose_instability_detector/debug/diff_pose":
            pose = msg.pose
            pose_instability_diff_pose_list.append({
                'timestamp': timestamp_header,
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z,
                'qw': pose.orientation.w,
                'qx': pose.orientation.x,
                'qy': pose.orientation.y,
                'qz': pose.orientation.z,
            })
        elif topic == "/localization/pose_twist_fusion_filter/kinematic_state":
            pose = msg.pose.pose
            ekf_pose_list.append({
                'timestamp': timestamp_header,
                'x': pose.position.x,
                'y': pose.position.y,
                'z': pose.position.z,
                'qw': pose.orientation.w,
                'qx': pose.orientation.x,
                'qy': pose.orientation.y,
                'qz': pose.orientation.z,
            })
        else:
            assert False, f"Unknown topic: {topic}"

    print(f"{len(pose_instability_diff_pose_list)=}")
    print(f"{len(ekf_pose_list)=}")

    df_pose_instability_diff_pose = pd.DataFrame(
        pose_instability_diff_pose_list)
    df_ekf_pose = pd.DataFrame(ekf_pose_list)

    df_ekf_pose = interpolate_pose(
        df_ekf_pose, df_pose_instability_diff_pose['timestamp'])
    print(f"{len(df_pose_instability_diff_pose)=}")
    print(f"{len(df_ekf_pose)=}")

    # rosbag path may be the path to the db3 file, or it may be the path to the directory containing it
    save_dir = rosbag_path.parent if rosbag_path.is_dir() else rosbag_path.parent.parent

    # plot pose_instability_diff_pose
    plt.plot(df_pose_instability_diff_pose['timestamp'],
             df_pose_instability_diff_pose['x'], label='x')
    plt.plot(df_pose_instability_diff_pose['timestamp'],
             df_pose_instability_diff_pose['y'], label='y')
    plt.plot(df_pose_instability_diff_pose['timestamp'],
             df_pose_instability_diff_pose['z'], label='z')
    plt.xlabel("timestamp")
    plt.ylabel("pose_instability_diff_pose[m]")
    plt.legend()
    save_path = save_dir / "pose_instability_diff_pose.png"
    plt.savefig(save_path, bbox_inches='tight', pad_inches=0.05)
    print(f"Saved to {save_path}")
    plt.close()

    # plot norm of pose_instability_diff_pose on trajectory of ekf_pose
    diff_pose = df_pose_instability_diff_pose[['x', 'y', 'z']].values
    diff_pose_norm = np.linalg.norm(diff_pose, axis=1)

    plt.scatter(df_ekf_pose['x'], df_ekf_pose['y'],
                c=diff_pose_norm, cmap='viridis')
    plt.colorbar(label='pose_instability_diff_pose[m]')
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.axis('equal')
    save_path = save_dir / "pose_instability_diff_pose_on_ekf_pose.png"
    plt.savefig(save_path, bbox_inches='tight', pad_inches=0.05)
    print(f"Saved to {save_path}")
    plt.close()
