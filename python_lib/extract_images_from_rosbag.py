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
from parse_functions import parse_stamp, parse_CameraInfo
from collections import defaultdict
from tf2_ros import Buffer
from builtin_interfaces.msg import Time


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("path_to_rosbag", type=str)
    parser.add_argument("output_dir", type=str)
    parser.add_argument("--use_cvt_color", action="store_true")
    parser.add_argument("--max_num", type=int, default=1000000000)
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
    max_num = args.max_num

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
    print(f"{camera_topic_list=}")

    os.makedirs(output_dir, exist_ok=True)

    bridge = CvBridge()

    camera_info_df_dict = defaultdict(list)
    tf_buffer = Buffer()
    transform_dict = dict()

    while reader.has_next():
        (topic, data, t) = reader.read_next()

        if topic == "/tf_static":
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            for transform in msg.transforms:
                tf_buffer.set_transform_static(transform, "default_authority")
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
        try:
            transform = tf_buffer.lookup_transform(
                target_frame="base_link", source_frame=camera_frame, time=Time()
            )
        except Exception as e:
            print(f"!{e}")
            continue
        transform_dict[camera_name] = transform

        timestamp_header = parse_stamp(image_msg.header.stamp)

        cv_image = (
            bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
            if msg_type == Image
            else bridge.compressed_imgmsg_to_cv2(image_msg)
        )
        if use_cvt_color:
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        save_dir = f"{output_dir}/{camera_name}"
        os.makedirs(save_dir, exist_ok=True)
        save_path = f"{save_dir}/{timestamp_header:018d}.png"
        cv2.imwrite(save_path, cv_image)
        print(f"\rtimestamp = {t}", end="")

        if len(camera_info_df_dict[camera_name]) >= max_num:
            break

    print()
    for camera_name, transform in transform_dict.items():
        save_path = f"{output_dir}/transform_{camera_name}.tsv"
        df = pd.DataFrame(
            {
                "x": [transform.transform.translation.x],
                "y": [transform.transform.translation.y],
                "z": [transform.transform.translation.z],
                "qw": [transform.transform.rotation.w],
                "qx": [transform.transform.rotation.x],
                "qy": [transform.transform.rotation.y],
                "qz": [transform.transform.rotation.z],
            }
        )
        df.to_csv(save_path, index=False, sep="\t")

    for camera_name, df in camera_info_df_dict.items():
        df = pd.DataFrame(df)
        df.to_csv(f"{output_dir}/camera_info_{camera_name}.tsv", index=False, sep="\t")
        print(f"{camera_name} camera_info: {len(df)} msgs")
