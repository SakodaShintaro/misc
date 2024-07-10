"""IMUの角速度バイアス推定と車速のスケール推定の結果を可視化するスクリプト"""

import argparse
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation
from pathlib import Path
from parse_functions import parse_rosbag
from interpolate_pose import interpolate_pose
from tf2_ros import Buffer
from builtin_interfaces.msg import Time
from calc_relative_pose import calc_relative_pose


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("rosbag_path", type=Path)
    parser.add_argument(
        "--topic_imu_raw", type=str, default="/sensing/imu/tamagawa/imu_raw"
    )
    parser.add_argument(
        "--topic_velocity", type=str, default="/vehicle/status/velocity_status"
    )
    return parser.parse_args()


def plot_pose(
    df_pose: pd.DataFrame,
    value_name: str,
    save_dir: Path,
    df_value: pd.DataFrame | None = None,
) -> None:
    df = (
        interpolate_pose(
            df_pose,
            df_value["timestamp"].values,
            POSITIONS_KEY=["position.x", "position.y", "position.z"],
            ORIENTATIONS_KEY=[
                "orientation.x",
                "orientation.y",
                "orientation.z",
                "orientation.w",
            ],
        )
        if df_value is not None
        else df_pose
    )
    color = df_value["data"] if df_value is not None else None
    plt.scatter(df["position.x"], df["position.y"], c=color, label=value_name)
    if df_value is not None:
        plt.colorbar()
    plt.axis("equal")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.legend()
    plt.grid()


if __name__ == "__main__":
    args = parse_args()
    rosbag_path = args.rosbag_path
    topic_imu_raw = args.topic_imu_raw
    topic_velocity = args.topic_velocity

    target_topics = [
        "/tf_static",
        "/localization/kinematic_state",
        topic_imu_raw,
        topic_velocity,
    ]

    df_dict = parse_rosbag(str(rosbag_path), target_topics)

    # rosbag path may be the path to the db3 file, or it may be the path to the directory containing it
    save_dir = (
        rosbag_path.parent if rosbag_path.is_dir() else rosbag_path.parent.parent
    ) / "deviation_estimation_result"
    save_dir.mkdir(exist_ok=True)

    # save as csv
    for topic_name in target_topics:
        df = df_dict[topic_name]
        if len(df) == 0:
            print(f"!{topic_name} is empty")
            continue
        filename = topic_name[1:].replace("/", "_")
        df.to_csv(
            save_dir / f"{filename}.tsv",
            index=False,
            sep="\t",
            float_format="%.9f",
        )

    df_kinematic_state = df_dict["/localization/kinematic_state"]
    df_imu_raw = df_dict[topic_imu_raw]
    df_velocity = df_dict[topic_velocity]

    # transformを取得
    df_tf_static = df_dict["/tf_static"]
    tf_buffer = Buffer()
    for _, row in df_tf_static.iterrows():
        for transform_stamped in row["transforms"]:
            tf_buffer.set_transform_static(transform_stamped, "default_authority")
    transform = tf_buffer.lookup_transform(
        target_frame="base_link", source_frame=df_imu_raw["frame_id"][0], time=Time()
    )
    transform_mat = np.eye(3)
    transform_mat[:3, :3] = Rotation.from_quat(
        [
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ]
    ).as_matrix()
    # transform_mat[:3, 3] = np.array(
    #     [
    #         transform.transform.translation.x,
    #         transform.transform.translation.y,
    #         transform.transform.translation.z,
    #     ]
    # )

    first_pose = df_kinematic_state.iloc[0]

    # 4x4 pose matrix
    curr_pose = np.eye(4)
    curr_pose[:3, 3] = first_pose[["position.x", "position.y", "position.z"]]
    curr_pose[:3, :3] = Rotation.from_quat(
        first_pose[["orientation.x", "orientation.y", "orientation.z", "orientation.w"]]
    ).as_matrix()

    curr_timestamp = first_pose["timestamp"]

    curr_angular_velocity = np.array([0, 0, 0])
    curr_longitudinal_velocity = 0

    curr_index_imu_raw = -1
    curr_index_velocity = -1

    imu_offset = np.array([0.00137, -0.00125, 0.00027])
    scale_factor = 1.01305

    value_list = []
    df_odometry_result = pd.DataFrame(columns=df_kinematic_state.columns)

    while curr_index_imu_raw + 1 < len(df_imu_raw) and curr_index_velocity + 1 < len(
        df_velocity
    ):
        next_timestamp_imu_raw = df_imu_raw["timestamp"].iloc[curr_index_imu_raw + 1]
        next_timestamp_velocity = df_velocity["timestamp"].iloc[curr_index_velocity + 1]

        min_timestamp = min(next_timestamp_imu_raw, next_timestamp_velocity)
        time_diff = (min_timestamp - curr_timestamp) / 1e9

        # Poseを進める
        # 回転
        curr_rotate = Rotation.from_rotvec(
            transform_mat @ (curr_angular_velocity - imu_offset) * time_diff
        ).as_matrix()
        curr_pose[:3, :3] = curr_rotate @ curr_pose[:3, :3]
        # 並進
        curr_pose[:3, 3] += (
            curr_pose[:3, :3]
            @ np.array([curr_longitudinal_velocity * scale_factor, 0, 0])
            * time_diff
        )

        quat = Rotation.from_matrix(curr_pose[:3, :3]).as_quat()

        # 保存
        value_list.append(
            {
                "timestamp": np.int64(curr_timestamp),
                "position.x": curr_pose[0, 3],
                "position.y": curr_pose[1, 3],
                "position.z": curr_pose[2, 3],
                "orientation.x": quat[0],
                "orientation.y": quat[1],
                "orientation.z": quat[2],
                "orientation.w": quat[3],
            }
        )

        # indexを進める
        if min_timestamp == next_timestamp_imu_raw:
            curr_index_imu_raw += 1
            curr_angular_velocity = df_imu_raw.iloc[curr_index_imu_raw][
                ["angular_velocity.x", "angular_velocity.y", "angular_velocity.z"]
            ]
        if min_timestamp == next_timestamp_velocity:
            curr_index_velocity += 1
            curr_longitudinal_velocity = df_velocity.iloc[curr_index_velocity][
                "longitudinal_velocity"
            ]

        curr_timestamp = min_timestamp

        if curr_index_velocity % 800 == 0:
            df_odometry_result = pd.DataFrame(value_list)

            # plot pose
            plt.subplot(3, 1, 1)
            plot_pose(df_kinematic_state, "kinematic_state", save_dir)
            plot_pose(df_odometry_result, "odometry_result", save_dir)

            df_curr_gt = df_kinematic_state[
                df_kinematic_state["timestamp"] <= curr_timestamp
            ]
            # タイムスタンプが一番近いものを取得
            df_curr_pred = pd.merge_asof(
                df_curr_gt,
                df_odometry_result,
                on="timestamp",
                direction="nearest",
            )
            df_curr_pred = df_curr_pred.rename(
                columns={
                    "position.x_y": "position.x",
                    "position.y_y": "position.y",
                    "position.z_y": "position.z",
                    "orientation.x_y": "orientation.x",
                    "orientation.y_y": "orientation.y",
                    "orientation.z_y": "orientation.z",
                    "orientation.w_y": "orientation.w",
                }
            )
            df_relative = calc_relative_pose(df_curr_pred, df_curr_gt)
            plt.subplot(3, 1, 2)
            plt.plot(df_relative["timestamp"], df_relative["position.x"], label="x")
            plt.plot(df_relative["timestamp"], df_relative["position.y"], label="y")
            plt.plot(df_relative["timestamp"], df_relative["position.z"], label="z")
            plt.xlabel("timestamp [ns]")
            plt.ylabel("position.diff [m]")
            plt.legend()
            plt.grid()

            plt.subplot(3, 1, 3)
            plt.plot(df_relative["timestamp"], df_relative["angle.x"], label="x")
            plt.plot(df_relative["timestamp"], df_relative["angle.y"], label="y")
            plt.plot(df_relative["timestamp"], df_relative["angle.z"], label="z")
            plt.xlabel("timestamp [ns]")
            plt.ylabel("angle.diff [deg]")
            plt.legend()
            plt.grid()

            save_path = save_dir / "pose_compare.png"
            plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
            plt.close()
            print(f"saved to {save_path}")
