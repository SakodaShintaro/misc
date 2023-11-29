#!/usr/bin/env python3

import argparse
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
from rclpy.serialization import deserialize_message
import os
import rosbag2_py
import pandas as pd


def create_reader(input_bag_dir: str, storage_id: str):
    storage_options = rosbag2_py.StorageOptions(
        uri=input_bag_dir, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader, storage_options, converter_options


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("path_to_rosbag", type=str)
    parser.add_argument("output_dir", type=str)
    parser.add_argument("--topic_name", type=str,
                        default="/sensing/camera/traffic_light/image_raw")
    parser.add_argument("--use_cvt_color", action="store_true")
    args = parser.parse_args()

    path_to_rosbag = args.path_to_rosbag
    output_dir = args.output_dir
    topic_name = args.topic_name
    use_cvt_color = args.use_cvt_color

    reader, storage_options, converter_options = create_reader(
        path_to_rosbag, "sqlite3")
    os.makedirs(output_dir, exist_ok=True)

    bridge = CvBridge()

    index_images = 0
    timestamp_list = []
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic != topic_name:
            continue

        image_msg = deserialize_message(data, Image())
        cv_image = bridge.imgmsg_to_cv2(
            image_msg, desired_encoding="passthrough")
        if use_cvt_color:
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        os.makedirs(f"{output_dir}/images", exist_ok=True)
        save_path = f"{output_dir}/images/{index_images:08d}.png"
        cv2.imwrite(save_path, cv_image)
        timestamp_list.append(t)
        print(f"\rtimestamp = {t}", end="")
        index_images += 1

    print()
    df_timestamp = pd.DataFrame(timestamp_list, columns=["timestamp"])
    df_timestamp.to_csv(f"{output_dir}/image_timestamps.tsv", index=False)
