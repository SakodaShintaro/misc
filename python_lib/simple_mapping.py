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
from scipy.spatial.transform import Rotation
import open3d as o3d
from tqdm import tqdm


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("rosbag_path", type=Path)
    parser.add_argument("points_topic", type=str)
    parser.add_argument("pose_tsv", type=Path)
    parser.add_argument("--save_npy", action="store_true")
    return parser.parse_args()


def save_npy(args):
    args = parse_args()
    rosbag_path = args.rosbag_path
    points_topic = args.points_topic
    _ = args.pose_tsv

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

    save_dir = rosbag_path.parent / "sensor_pcd"
    save_dir.mkdir(exist_ok=True, parents=True)

    while reader.has_next():
        (topic, data, timestamp_rosbag) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        points = read_points(msg)
        xyz = np.array([(p["x"], p["y"], p["z"]) for p in points], dtype=np.float32)
        timestamp_header = parse_stamp(msg.header.stamp)
        save_path = save_dir / f"{timestamp_header}.npy"
        np.save(save_path, xyz)


def mapping(args):
    args = parse_args()
    rosbag_path = args.rosbag_path
    _ = args.points_topic
    pose_tsv = args.pose_tsv
    df_pose = pd.read_csv(pose_tsv, delimiter="\t")

    save_dir = rosbag_path.parent / "sensor_pcd"
    file_list = sorted(list(save_dir.glob("*.npy")))
    timestamp_list = [int(f.stem) for f in file_list]
    timestamp_list = pd.Series(timestamp_list)
    cond = (df_pose.iloc[0]["timestamp"] <= timestamp_list) & (
        timestamp_list <= df_pose.iloc[-1]["timestamp"]
    )
    timestamp_list = timestamp_list[cond]

    df_interpolated_pose = interpolate_pose(df_pose, timestamp_list)

    print(len(timestamp_list), len(df_interpolated_pose))

    xyz_all = []
    for i, timestamp in enumerate(tqdm(timestamp_list)):
        file_path = save_dir / f"{timestamp}.npy"
        xyz = np.load(file_path)
        xyz = xyz[::100]
        pose = df_interpolated_pose.iloc[i]
        r = Rotation.from_quat(
            [
                pose["orientation.x"],
                pose["orientation.y"],
                pose["orientation.z"],
                pose["orientation.w"],
            ]
        )
        xyz = r.apply(xyz) + pose[["position.x", "position.y", "position.z"]].values
        xyz_all.append(xyz)

    # open3dで可視化
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.vstack(xyz_all))
    o3d.visualization.draw_geometries([pcd])

    save_path = save_dir.parent / "mapped.npy"
    np.save(save_path, np.vstack(xyz_all))


if __name__ == "__main__":
    args = parse_args()

    if args.save_npy:
        save_npy(args)

    mapping(args)
