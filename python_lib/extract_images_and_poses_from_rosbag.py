#!/usr/bin/env python3

import argparse
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
import numpy as np
import cv2
from rclpy.serialization import deserialize_message
import os
import pandas as pd
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
import geometry_msgs
from scipy.spatial.transform import Rotation, Slerp
import matplotlib.pyplot as plt
import rosbag2_py


def save_camera_info_to_opencv_yaml(camera_info, filename):
    fs = cv2.FileStorage(filename, cv2.FileStorage_WRITE)

    fs.write("width", camera_info.width)
    fs.write("height", camera_info.height)

    D = np.array(camera_info.d, dtype=np.float32)
    fs.write("D", D)

    K = np.array(camera_info.k, dtype=np.float32).reshape((3, 3))
    fs.write("K", K)

    R = np.array(camera_info.r, dtype=np.float32).reshape((3, 3))
    fs.write("R", R)

    P = np.array(camera_info.p, dtype=np.float32).reshape((3, 4))
    fs.write("P", P)

    fs.release()


def interpolate_pose_in_time(df: pd.DataFrame, target_timestamp_list: pd.Series) -> pd.DataFrame:
    """ dfというデータフレームを、target_timestamp_listが示すタイムスタンプに合わせて補間する
    制約)
    * dfとtarget_timestamp_listはタイムスタンプでソートされていること
    * dfはtarget_timestamp_listを覆うような前後に広い区間のタイムスタンプを持つこと
    出力)
    * dfと同じカラムを持ち、長さがtarget_timestamp_listと同じであるデータフレーム
    """
    POSITIONS_KEY = ['x', 'y', 'z']
    ORIENTATIONS_KEY = ['qw', 'qx', 'qy', 'qz']
    result_df = pd.DataFrame(columns=df.columns)
    target_index = 0
    df_index = 0
    while df_index < len(df) - 1 and target_index < len(target_timestamp_list):
        curr_time = df.iloc[df_index]['timestamp']
        next_time = df.iloc[df_index + 1]['timestamp']
        target_time = target_timestamp_list[target_index]

        # target_timeを挟み込むようなdf_indexを探す
        if not (curr_time <= target_time <= next_time):
            df_index += 1
            continue

        curr_weight = (next_time - target_time) / (next_time - curr_time)
        next_weight = 1.0 - curr_weight

        curr_position = df.iloc[df_index][POSITIONS_KEY]
        next_position = df.iloc[df_index + 1][POSITIONS_KEY]
        target_position = curr_position * curr_weight + next_position * next_weight

        curr_orientation = df.iloc[df_index][ORIENTATIONS_KEY]
        next_orientation = df.iloc[df_index + 1][ORIENTATIONS_KEY]
        curr_r = Rotation.from_quat(curr_orientation)
        next_r = Rotation.from_quat(next_orientation)
        slerp = Slerp([curr_time, next_time],
                      Rotation.concatenate([curr_r, next_r]))
        target_orientation = slerp([target_time]).as_quat()[0]

        target_row = df.iloc[df_index].copy()
        target_row['timestamp'] = target_timestamp_list[target_index]
        target_row[POSITIONS_KEY] = target_position
        target_row[ORIENTATIONS_KEY] = target_orientation
        result_df = result_df.append(target_row)
        target_index += 1
    return result_df


def create_reader(input_bag_dir: str, storage_id: str):
    storage_options = rosbag2_py.StorageOptions(
        uri=input_bag_dir, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader, storage_options, converter_options


def transform_pose_base_link_2_camera(pose: Pose, target_frame: str, tf_buffer, time):
    # get static transform
    transform = tf_buffer.lookup_transform(
        target_frame="base_link",
        source_frame=target_frame,
        time=time)
    # transform pose
    R1: geometry_msgs.msg.Quaternion = transform.transform.rotation
    R1: np.ndarray = Rotation.from_quat([R1.x, R1.y, R1.z, R1.w]).as_matrix()
    t1: geometry_msgs.msg.Vector3 = transform.transform.translation
    t1: np.ndarray = np.array([t1.x, t1.y, t1.z])

    # pose
    R2: geometry_msgs.msg.Quaternion = pose.orientation
    R2: np.ndarray = Rotation.from_quat([R2.x, R2.y, R2.z, R2.w]).as_matrix()
    t2: geometry_msgs.msg.Vector3 = pose.position
    t2: np.ndarray = np.array([t2.x, t2.y, t2.z])

    # transform
    R: np.ndarray = np.dot(R2, R1)
    t: np.ndarray = np.dot(R2, t1) + t2
    q: np.ndarray = Rotation.from_matrix(R).as_quat()

    # convert to geometry_msgs.msg.Pose
    result_pose = Pose()
    result_pose.orientation.x = q[0]
    result_pose.orientation.y = q[1]
    result_pose.orientation.z = q[2]
    result_pose.orientation.w = q[3]
    result_pose.position.x = t[0]
    result_pose.position.y = t[1]
    result_pose.position.z = t[2]
    return result_pose


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("path_to_rosbag", type=str)
    parser.add_argument("output_dir", type=str)
    parser.add_argument("--storage_id", type=str,
                        default="sqlite3", choices=["mcap", "sqlite3"])
    args = parser.parse_args()

    path_to_rosbag = args.path_to_rosbag
    output_dir = args.output_dir
    storage_id = args.storage_id

    target_frame = "traffic_light_left_camera/camera_optical_link"
    image_topic_name = "/sensing/camera/traffic_light/image_raw"
    camera_info_topic_name = "/sensing/camera/traffic_light/camera_info"
    pose_topic_name = "/localization/pose_twist_fusion_filter/biased_pose_with_covariance"
    image_topic_type = Image()
    pose_topic_type = PoseWithCovarianceStamped()

    reader, storage_options, converter_options = create_reader(
        path_to_rosbag, storage_id)
    os.makedirs(output_dir, exist_ok=True)
    with open(f"{output_dir}/rosbag_info.txt", "w") as f:
        f.write(f"{path_to_rosbag}\n")

    bridge = CvBridge()
    tf_buffer = Buffer()

    index_images_all = 0
    prev_image = None
    image_timestamp_list = list()
    image_list = list()
    columns = ["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"]
    df_pose = pd.DataFrame(columns=columns)
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        t /= 1e9
        if topic == image_topic_name:
            image_msg = deserialize_message(data, image_topic_type)
            if image_topic_type == Image():
                curr_image = bridge.imgmsg_to_cv2(
                    image_msg, desired_encoding="passthrough")
            elif image_topic_type == CompressedImage():
                curr_image = bridge.compressed_imgmsg_to_cv2(
                    image_msg, desired_encoding="passthrough")
            diff = (1 if prev_image is None else np.abs(
                prev_image - curr_image).sum())
            prev_image = curr_image
            index_images_all += 1
            if diff == 0:
                continue
            image_timestamp_list.append(t)
            image_list.append(curr_image)
        elif topic == pose_topic_name:
            pose_msg = deserialize_message(data, pose_topic_type)
            pose = pose_msg.pose.pose if pose_topic_type == PoseWithCovarianceStamped() else pose_msg.pose
            try:
                pose = transform_pose_base_link_2_camera(
                    pose, target_frame, tf_buffer, pose_msg.header.stamp)
            except Exception as e:
                print(e)
                continue
            df_pose.loc[len(df_pose)] = [
                t,
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        elif topic == camera_info_topic_name:
            camera_info = deserialize_message(data, CameraInfo())
            save_camera_info_to_opencv_yaml(
                camera_info, f"{output_dir}/camera_info.yaml")
        elif topic == "/tf" or topic == "/tf_static":
            is_static = (topic == "/tf_static")
            tf_msg = deserialize_message(data, TFMessage())
            for transform_stamped in tf_msg.transforms:
                if is_static:
                    tf_buffer.set_transform_static(
                        transform_stamped, "default_authority")
                else:
                    tf_buffer.set_transform(
                        transform_stamped, "default_authority")

    image_timestamp_list = np.array(image_timestamp_list)
    image_list = np.array(image_list)

    min_pose_t = df_pose["timestamp"].min()
    max_pose_t = df_pose["timestamp"].max()
    ok_image_timestamp = (min_pose_t < image_timestamp_list) * \
        (image_timestamp_list < max_pose_t)
    image_timestamp_list = image_timestamp_list[ok_image_timestamp]
    image_list = image_list[ok_image_timestamp]

    df_pose = interpolate_pose_in_time(df_pose, image_timestamp_list)

    os.makedirs(f"{output_dir}/images", exist_ok=True)
    for i, image in enumerate(image_list):
        save_path = f"{output_dir}/images/{i:08d}.png"
        cv2.imwrite(save_path, image)

    df_pose.to_csv(f"{output_dir}/pose.tsv",
                   index=True, sep="\t", float_format="%.12f")

    # plot all of trajectory
    save_path = f"{output_dir}/plot_pose.png"
    plt.plot(df_pose["x"], df_pose["y"])
    plt.xlabel("x")
    plt.ylabel("y")
    plt.axis("equal")
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    plt.close()
    print(f"Saved to {save_path}")
