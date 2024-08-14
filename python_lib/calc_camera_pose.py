#!/usr/bin/env python3

import argparse
import numpy as np
import cv2
import pandas as pd
from scipy.spatial.transform import Rotation
from interpolate_pose import interpolate_pose
from pathlib import Path
from geometry_msgs.msg import TransformStamped


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--pose_tsv_path", type=Path, default=None)
    parser.add_argument(
        "--pose_topic_name",
        type=str,
        default="/awsim/ground_truth/localization/kinematic_state",
    )
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


def transform_pose_base_link_2_camera(
    df_pose: pd.DataFrame, transform: TransformStamped
) -> pd.DataFrame:
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

    df_result = df_pose.copy()

    df_result["orientation.x"] = q_c2m[:, 0]
    df_result["orientation.y"] = q_c2m[:, 1]
    df_result["orientation.z"] = q_c2m[:, 2]
    df_result["orientation.w"] = q_c2m[:, 3]
    df_result["position.x"] = t_c2m[:, 0]
    df_result["position.y"] = t_c2m[:, 1]
    df_result["position.z"] = t_c2m[:, 2]
    return df_result


if __name__ == "__main__":
    args = parse_args()
    output_dir = args.output_dir
    pose_tsv_path = args.pose_tsv_path
    pose_topic_name = args.pose_topic_name

    save_name = "__".join(pose_topic_name.split("/")[1:])

    pose_tsv_path = output_dir / f"{save_name}.tsv" if pose_tsv_path is None else pose_tsv_path

    df_pose = pd.read_csv(pose_tsv_path, sep="\t")

    dir_list = sorted([p for p in output_dir.glob("*") if p.is_dir()])
    for dir_path in dir_list:
        print(dir_path)
        camera_name = dir_path.name
        df_camera_info = pd.read_csv(
            output_dir / f"camera_info_{camera_name}.tsv", sep="\t"
        )
        df_transform = pd.read_csv(
            output_dir / f"transform_{camera_name}.tsv", sep="\t"
        )
        transform = TransformStamped()
        transform.header.frame_id = "base_link"
        transform.child_frame_id = camera_name
        transform.transform.translation.x = df_transform["x"].values[0]
        transform.transform.translation.y = df_transform["y"].values[0]
        transform.transform.translation.z = df_transform["z"].values[0]
        transform.transform.rotation.w = df_transform["qw"].values[0]
        transform.transform.rotation.x = df_transform["qx"].values[0]
        transform.transform.rotation.y = df_transform["qy"].values[0]
        transform.transform.rotation.z = df_transform["qz"].values[0]

        df_camera_pose = transform_pose_base_link_2_camera(df_pose, transform)

        image_list = sorted(list(dir_path.glob("*.png")))
        timestamp_list = [int(p.stem) for p in image_list]

        # タイムスタンプがdf_camera_poseの範囲外のものは無理やり範囲内に収める
        timestamp_list = [
            max(t, df_camera_pose["timestamp"].values[0]) for t in timestamp_list
        ]

        df_camera_pose = interpolate_pose(df_camera_pose, timestamp_list)
        df_camera_pose.to_csv(
            output_dir / f"pose_{camera_name}.tsv",
            index=True,
            sep="\t",
            float_format="%.12f",
        )
