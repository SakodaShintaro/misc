#!/usr/bin/env python3

import argparse
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
import cv2
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import os
import rosbag2_py
import pandas as pd
from parse_functions import parse_stamp, parse_CameraInfo, parse_msg
from collections import defaultdict
from tf2_ros import Buffer
from builtin_interfaces.msg import Time
from extract_images_and_poses_from_rosbag import transform_pose_base_link_2_camera
import numpy as np
from copy import deepcopy


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("path_to_rosbag", type=str)
    parser.add_argument("output_dir", type=str)
    parser.add_argument("--use_cvt_color", action="store_true")
    parser.add_argument("--use_timestamp_as_filename", action="store_true")
    parser.add_argument(
        "--pose_topic", type=str, default="/localization/kinematic_state"
    )
    return parser.parse_args()


def create_reader(input_bag_dir: str, storage_id: str):
    storage_options = rosbag2_py.StorageOptions(
        uri=input_bag_dir, storage_id=storage_id
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader, storage_options, converter_options


if __name__ == "__main__":
    args = parse_args()
    path_to_rosbag = args.path_to_rosbag
    output_dir = args.output_dir
    use_cvt_color = args.use_cvt_color
    pose_topic = args.pose_topic

    reader, storage_options, converter_options = create_reader(
        path_to_rosbag, "sqlite3"
    )
    topic_types = reader.get_all_topics_and_types()
    type_map = {
        topic_types[i].name: topic_types[i].type for i in range(len(topic_types))
    }

    camera_topic_list = [
        topic.name for topic in topic_types if topic.name.startswith("/sensing/camera")
    ]

    os.makedirs(output_dir, exist_ok=True)

    bridge = CvBridge()

    index_images = 0

    camera_info_df_dict = defaultdict(list)
    pose_df_dict = defaultdict(list)
    tf_buffer = Buffer()

    df_pose = None

    while reader.has_next():
        (topic, data, t) = reader.read_next()

        if topic == "/tf_static":
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            for transform in msg.transforms:
                tf_buffer.set_transform_static(transform, "default_authority")
                print(transform)
            continue

        if topic == pose_topic:
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            data_dict = parse_msg(msg, msg_type)
            if df_pose is None:
                df_pose = pd.DataFrame([data_dict])
            else:
                df_pose = pd.concat([df_pose, pd.DataFrame([data_dict])], join="inner")

        if df_pose is None:
            continue

        if topic not in camera_topic_list:
            continue

        elements = topic.split("/")
        camera_name = elements[3]

        msg_type = get_message(type_map[topic])

        if msg_type == CameraInfo:
            msg = deserialize_message(data, msg_type)
            camera_info = parse_CameraInfo(msg)
            camera_info_df_dict[camera_name].append(camera_info)
            continue

        image_msg = deserialize_message(data, msg_type)

        camera_frame = image_msg.header.frame_id
        transform = tf_buffer.lookup_transform(
            target_frame="base_link", source_frame=camera_frame, time=Time()
        )

        timestamp_header = parse_stamp(image_msg.header.stamp)
        index = np.searchsorted(df_pose["timestamp"].values, timestamp_header)
        if index == 0:
            continue
        pose_row = deepcopy(df_pose.iloc[index - 1 : index])
        transform_pose_base_link_2_camera(pose_row, transform)

        cv_image = (
            bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
            if msg_type == Image
            else bridge.compressed_imgmsg_to_cv2(image_msg)
        )
        if use_cvt_color:
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        save_dir = f"{output_dir}/{camera_name}"
        os.makedirs(save_dir, exist_ok=True)
        save_path = (
            f"{save_dir}/{timestamp_header:018d}.png"
            if args.use_timestamp_as_filename
            else f"{save_dir}/{index_images:08d}.png"
        )
        cv2.imwrite(save_path, cv_image)
        row_dict = deepcopy(pose_row.iloc[0]).to_dict()
        row_dict["timestamp"] = int(timestamp_header)
        pose_df_dict[camera_name].append(row_dict)
        print(f"\rtimestamp = {t}, {len(pose_df_dict[camera_name])=}", end="")
        index_images += 1

    print()
    for camera_name, df in pose_df_dict.items():
        print(df)
        df = pd.DataFrame(df)
        df["timestamp"] = df["timestamp"].astype("int64")
        df.to_csv(f"{output_dir}/pose_{camera_name}.tsv", index=False, sep="\t")
        print(f"{camera_name} pose: {len(df)} msgs")

    for camera_name, df in camera_info_df_dict.items():
        df = pd.DataFrame(df)
        df.to_csv(f"{output_dir}/camera_info_{camera_name}.tsv", index=False, sep="\t")
        print(f"{camera_name} camera_info: {len(df)} msgs")
