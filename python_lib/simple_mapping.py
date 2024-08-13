"""点群を持つrosbagとposeのtsvファイルからシンプルにマッピングするスクリプト"""

import argparse
from pathlib import Path
import pandas as pd
from interpolate_pose import interpolate_pose
from parse_functions import parse_stamp
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py.point_cloud2 import read_points
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
import open3d as o3d
from tqdm import tqdm


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("rosbag_path", type=Path)
    parser.add_argument("points_topic", type=str)
    parser.add_argument("pose_tsv", type=Path)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    rosbag_path = args.rosbag_path
    points_topic = args.points_topic
    pose_tsv = args.pose_tsv

    # poseを取得
    df_pose = pd.read_csv(pose_tsv, delimiter="\t")
    print(df_pose.head())

    serialization_format = "cdr"
    storage_options = rosbag2_py.StorageOptions(
        uri=str(rosbag_path), storage_id="sqlite3"
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format,
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {
        topic_types[i].name: topic_types[i].type for i in range(len(topic_types))
    }

    storage_filter = rosbag2_py.StorageFilter(topics=[points_topic])
    reader.set_filter(storage_filter)
    xyz_all = []

    pose_index = 0
    progress = tqdm(total=len(df_pose))

    while reader.has_next():
        (topic, data, timestamp_rosbag) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        points = read_points(msg)
        xyz = np.array([(p["x"], p["y"], p["z"]) for p in points], dtype=np.float32)
        xyz = xyz[::100]
        timestamp_header = parse_stamp(msg.header.stamp)
        while (
            pose_index < len(df_pose)
            and df_pose.iloc[pose_index]["timestamp"] < timestamp_header
        ):
            pose_index += 1
            progress.update(1)
        if pose_index >= len(df_pose):
            break
        curr_pose = df_pose.iloc[pose_index]

        t = curr_pose[["position.x", "position.y", "position.z"]].values
        q = curr_pose[
            ["orientation.x", "orientation.y", "orientation.z", "orientation.w"]
        ].values
        r = Rotation.from_quat(q)
        xyz = (r.apply(xyz) + t).astype(np.float32)
        xyz_all.append(xyz)

    # open3dで可視化
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.vstack(xyz_all))
    o3d.visualization.draw_geometries([pcd])

