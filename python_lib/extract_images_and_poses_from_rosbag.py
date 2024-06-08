#!/usr/bin/env python3

import argparse
import numpy as np
import cv2
import os
import pandas as pd
from tf2_ros import Buffer
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from interpolate_pose import interpolate_pose
from parse_functions import parse_rosbag
from builtin_interfaces.msg import Time
from tqdm import tqdm


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("path_to_rosbag", type=str)
    parser.add_argument("output_dir", type=str)
    parser.add_argument(
        "--storage_id", type=str, default="sqlite3", choices=["mcap", "sqlite3"]
    )
    parser.add_argument(
        "--image_topic_name",
        type=str,
        default="/sensing/camera/traffic_light/image_raw",
    )
    parser.add_argument(
        "--camera_info_topic_name",
        type=str,
        default="/sensing/camera/traffic_light/camera_info",
    )
    parser.add_argument(
        "--pose_topic_name",
        type=str,
        default="/awsim/ground_truth/vehicle/pose",
    )
    parser.add_argument("--image_interval", type=int, default=1)
    return parser.parse_args()


def save_camera_info_to_opencv_yaml(camera_info, filename):
    fs = cv2.FileStorage(filename, cv2.FileStorage_WRITE)

    fs.write("width", camera_info["width"])
    fs.write("height", camera_info["height"])

    D = np.array(camera_info["D"], dtype=np.float32)
    fs.write("D", D)

    K = np.array(camera_info["K"], dtype=np.float32).reshape((3, 3))
    fs.write("K", K)

    R = np.array(camera_info["R"], dtype=np.float32).reshape((3, 3))
    fs.write("R", R)

    P = np.array(camera_info["P"], dtype=np.float32).reshape((3, 4))
    fs.write("P", P)

    fs.release()


def transform_pose_base_link_2_camera(df_pose: pd.DataFrame, transform):
    # transform pose (camera_link to base_link)
    R_c2b: np.ndarray = Rotation.from_quat(
        [
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ]
    ).as_matrix()
    t_c2b: np.ndarray = np.array(
        [
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ]
    )

    # pose (base_link to map)
    R_b2m: np.ndarray = Rotation.from_quat(
        df_pose[
            ["orientation.x", "orientation.y", "orientation.z", "orientation.w"]
        ].values,
    ).as_matrix()
    t_b2m: np.ndarray = df_pose[["position.x", "position.y", "position.z"]].values

    # transform
    R_c2m: np.ndarray = np.dot(R_b2m, R_c2b)
    t_c2m: np.ndarray = np.dot(R_b2m, t_c2b) + t_b2m
    q_c2m: np.ndarray = Rotation.from_matrix(R_c2m).as_quat()

    df_pose["orientation.x"] = q_c2m[:, 0]
    df_pose["orientation.y"] = q_c2m[:, 1]
    df_pose["orientation.z"] = q_c2m[:, 2]
    df_pose["orientation.w"] = q_c2m[:, 3]
    df_pose["position.x"] = t_c2m[:, 0]
    df_pose["position.y"] = t_c2m[:, 1]
    df_pose["position.z"] = t_c2m[:, 2]


if __name__ == "__main__":
    args = parse_args()
    path_to_rosbag = args.path_to_rosbag
    output_dir = args.output_dir
    storage_id = args.storage_id
    image_topic_name = args.image_topic_name
    camera_info_topic_name = args.camera_info_topic_name
    pose_topic_name = args.pose_topic_name
    image_interval = args.image_interval

    target_topics = [
        image_topic_name,
        camera_info_topic_name,
        pose_topic_name,
        "/tf_static",
    ]

    df_dict = parse_rosbag(path_to_rosbag, target_topics)

    # save rosbag info
    os.makedirs(output_dir, exist_ok=True)
    with open(f"{output_dir}/rosbag_info.txt", "w") as f:
        f.write(f"{path_to_rosbag}\n")

    # save camera_info.yaml
    save_camera_info_to_opencv_yaml(
        df_dict[camera_info_topic_name].iloc[0], f"{output_dir}/camera_info.yaml"
    )

    df_image = df_dict[image_topic_name]
    df_pose = df_dict[pose_topic_name]

    camera_frame = df_image["frame_id"].values[0]
    print(f"{camera_frame=}")

    # transformを取得
    df_tf_static = df_dict["/tf_static"]
    tf_buffer = Buffer()
    for _, row in df_tf_static.iterrows():
        for transform_stamped in row["transforms"]:
            tf_buffer.set_transform_static(transform_stamped, "default_authority")
    transform = tf_buffer.lookup_transform(
        target_frame="base_link", source_frame=camera_frame, time=Time()
    )

    # df_poseを変換する
    # df_pose : base_link in map
    # 欲しいpose : camera_frame in map
    # tf_staticでcamera_frame in base_linkを取得してdf_poseに右からかける
    transform_pose_base_link_2_camera(df_pose, transform)

    image_timestamp_list = df_image["timestamp"].values
    image_list = df_image["image"].values

    min_pose_t = df_pose["timestamp"].min()
    max_pose_t = df_pose["timestamp"].max()
    ok_image_timestamp = (min_pose_t < image_timestamp_list) * (
        image_timestamp_list < max_pose_t
    )
    image_timestamp_list = image_timestamp_list[ok_image_timestamp]
    image_list = image_list[ok_image_timestamp]

    df_pose = interpolate_pose(df_pose, image_timestamp_list)

    os.makedirs(f"{output_dir}/images", exist_ok=True)
    bar = tqdm(total=len(image_list))
    for i, image in enumerate(image_list):
        save_path = f"{output_dir}/images/{i:08d}.png"
        cv2.imwrite(save_path, image)
        bar.update(1)

    df_pose.to_csv(f"{output_dir}/pose.tsv", index=True, sep="\t", float_format="%.12f")

    # plot all of trajectory
    save_path = f"{output_dir}/plot_pose.png"
    plt.plot(df_pose["position.x"], df_pose["position.y"])
    plt.xlabel("x")
    plt.ylabel("y")
    plt.axis("equal")
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    plt.close()
    print(f"Saved to {save_path}")
