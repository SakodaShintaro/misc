import argparse
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation
from pathlib import Path
from parse_functions import parse_rosbag


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("rosbag_path", type=Path)
    return parser.parse_args()


THRESHOLD = 0.5


def transform_covariance_from_map_to_base_link(df: pd.DataFrame) -> pd.DataFrame:
    pose_covariance = df[
        [
            "covariance_position.xx",
            "covariance_position.xy",
            "covariance_position.xz",
            "covariance_position.yx",
            "covariance_position.yy",
            "covariance_position.yz",
            "covariance_position.zx",
            "covariance_position.zy",
            "covariance_position.zz",
        ]
    ].values.reshape(-1, 3, 3)

    # map -> base_link
    rotation = Rotation.from_quat(
        df[["orientation.x", "orientation.y", "orientation.z", "orientation.w"]].values
    )
    rotation_matrix = rotation.as_matrix()
    transformed_covariance = (
        rotation_matrix.transpose(0, 2, 1) @ pose_covariance @ rotation_matrix
    )

    df[
        [
            "covariance_position.xx",
            "covariance_position.xy",
            "covariance_position.xz",
            "covariance_position.yx",
            "covariance_position.yy",
            "covariance_position.yz",
            "covariance_position.zx",
            "covariance_position.zy",
            "covariance_position.zz",
        ]
    ] = transformed_covariance.reshape(-1, 9)
    return df


def plot_stddev(df: pd.DataFrame) -> None:
    # x軸にタイムスタンプ, y軸に標準偏差をプロット
    time = (df["timestamp"] - df["timestamp"].iloc[0]) / 1e9
    plt.plot(
        time,
        np.sqrt(df["covariance_position.xx"]),
        label="xx",
    )
    plt.plot(
        time,
        np.sqrt(df["covariance_position.yy"]),
        label="yy",
    )
    plt.plot(
        time,
        THRESHOLD * np.ones_like(time),
        label=f"threshold({THRESHOLD})",
        linestyle="--",
    )

    plt.xlabel("times[s]")
    plt.ylabel("stddev[m]")
    plt.ylim(bottom=0)
    plt.legend()
    plt.grid()


def plot_trajectory(df: pd.DataFrame) -> None:
    # 軌跡にxの標準偏差で色付けしてプロット
    value = np.sqrt(df["covariance_position.xx"])

    plt.scatter(
        df["position.x"],
        df["position.y"],
        c=value,
        cmap="viridis",
        vmin=0,
        vmax=THRESHOLD * 1.5,
    )
    plt.colorbar()
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.axis("equal")
    plt.grid()


if __name__ == "__main__":
    args = parse_args()
    rosbag_path = args.rosbag_path

    target_topics = [
        "/localization/kinematic_state",
        "/localization/pose_estimator/pose_with_covariance",
    ]

    df_dict = parse_rosbag(str(rosbag_path), target_topics)

    # rosbag path may be the path to the db3 file, or it may be the path to the directory containing it
    save_dir = (
        rosbag_path.parent if rosbag_path.is_dir() else rosbag_path.parent.parent
    ) / "pose_covariance"
    save_dir.mkdir(exist_ok=True)

    # save as csv
    for topic_name in target_topics:
        df = df_dict[topic_name]
        if len(df) == 0:
            print(f"!{topic_name} is empty")
            continue
        filename = topic_name.replace("/localization/", "").replace("/", "_")
        df.to_csv(
            save_dir / f"{filename}.tsv",
            index=False,
            sep="\t",
            float_format="%.9f",
        )

    df_kinematic_state = df_dict["/localization/kinematic_state"]
    df_pose_with_covariance = df_dict[
        "/localization/pose_estimator/pose_with_covariance"
    ]

    df_kinematic_state = transform_covariance_from_map_to_base_link(df_kinematic_state)
    df_pose_with_covariance = transform_covariance_from_map_to_base_link(
        df_pose_with_covariance
    )

    plot_stddev(df_kinematic_state)
    save_path = save_dir / "stddev_kinematic_state.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    plt.close()

    plot_stddev(df_pose_with_covariance)
    save_path = save_dir / "stddev_pose_with_covariance.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    plt.close()

    plot_trajectory(df_kinematic_state)
    save_path = save_dir / "trajectory_kinematic_state.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    plt.close()

    plot_trajectory(df_pose_with_covariance)
    save_path = save_dir / "trajectory_pose_with_covariance.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    plt.close()
