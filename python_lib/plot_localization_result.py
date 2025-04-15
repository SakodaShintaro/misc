"""A script that plots the results of localization."""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation

from interpolate_pose import interpolate_pose
from parse_functions import parse_rosbag
from plot_pose_covariance import transform_covariance_from_map_to_base_link


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("rosbag_path", type=Path)
    parser.add_argument("--output_pose_array", action="store_true")
    parser.add_argument("--start_time_from_zero", action="store_true")
    return parser.parse_args()


def plot_pose(
    df_pose: pd.DataFrame,
    value_name: str,
    save_dir: Path,
    df_value: pd.DataFrame | None = None,
) -> None:
    if "timestamp" not in df_pose.columns or (
        df_value is not None and "timestamp" not in df_value.columns
    ):
        return
    if df_value is not None:
        df_value = df_value.sort_values(by="timestamp")
        cond = (df_pose.iloc[0]["timestamp"] <= df_value["timestamp"]) & (
            df_value["timestamp"] <= df_pose.iloc[-1]["timestamp"]
        )
        df_value = df_value[cond]
    df = (
        interpolate_pose(
            df_pose,
            df_value["timestamp"],
        )
        if df_value is not None
        else df_pose
    )
    if df_value is None:
        color = None
    else:
        df_value = df_value[df_value["timestamp"].isin(df["timestamp"])]
        color = df_value["data"]
    plt.scatter(df["position.x"], df["position.y"], c=color, s=0.5)
    if df_value is not None:
        plt.colorbar()
    plt.axis("equal")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.grid()
    save_path = save_dir / f"pose_{value_name}.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    plt.close()
    print(f"saved to {save_path}")


if __name__ == "__main__":
    args = parse_args()
    rosbag_path = args.rosbag_path
    output_pose_array = args.output_pose_array
    start_time_from_zero = args.start_time_from_zero

    target_topics = [
        "/localization/kinematic_state",
        "/localization/pose_estimator/exe_time_ms",
        "/localization/pose_estimator/initial_to_result_relative_pose",
        "/localization/pose_estimator/ndt_marker",
        "/localization/pose_estimator/nearest_voxel_transformation_likelihood",
        "/localization/pose_estimator/pose_with_covariance",
        "/localization/pose_estimator/transform_probability",
        "/localization/pose_twist_fusion_filter/estimated_yaw_bias",
    ]

    df_dict = parse_rosbag(str(rosbag_path), target_topics)

    # rosbag path may be the path to the db3 file, or may be the path to the directory containing it
    save_dir = (
        rosbag_path.parent if rosbag_path.is_dir() else rosbag_path.parent.parent
    ) / "localization_result"
    save_dir.mkdir(exist_ok=True)

    # save as tsv
    for topic_name in target_topics:
        df = df_dict[topic_name]
        if len(df) == 0:
            print(f"!{topic_name} is empty")
            continue
        if start_time_from_zero and "timestamp" in df.columns:
            df["timestamp"] -= df["timestamp"].iloc[0]

        filename = topic_name.replace("/localization/", "").replace("/", "_")
        df.to_csv(
            save_dir / f"{filename}.tsv",
            index=False,
            sep="\t",
            float_format="%.9f",
        )

    df_exe_time_ms = df_dict["/localization/pose_estimator/exe_time_ms"]
    df_nearest_voxel_transformation_likelihood = df_dict[
        "/localization/pose_estimator/nearest_voxel_transformation_likelihood"
    ]
    df_transform_probability = df_dict["/localization/pose_estimator/transform_probability"]
    df_ndt_pose_with_covariance = df_dict["/localization/pose_estimator/pose_with_covariance"]
    df_initial_to_result_relative_pose = df_dict[
        "/localization/pose_estimator/initial_to_result_relative_pose"
    ]
    df_marker = df_dict["/localization/pose_estimator/ndt_marker"]
    df_kinematic_state = df_dict["/localization/kinematic_state"]
    df_estimated_yaw_bias = df_dict["/localization/pose_twist_fusion_filter/estimated_yaw_bias"]

    # Convert covariance to base_link
    if len(df_ndt_pose_with_covariance) > 0:
        df_ndt_pose_with_covariance = transform_covariance_from_map_to_base_link(
            df_ndt_pose_with_covariance,
        )
    if len(df_kinematic_state) > 0:
        df_kinematic_state = transform_covariance_from_map_to_base_link(df_kinematic_state)

    # Calculate the norm of the change amount as "data" column
    if len(df_initial_to_result_relative_pose) > 0:
        df_initial_to_result_relative_pose["data"] = df_initial_to_result_relative_pose.apply(
            lambda x: np.linalg.norm([x["position.x"], x["position.y"], x["position.z"]]),
            axis=1,
        )

    # Increase the vertical size of the plot
    plt.rcParams["figure.figsize"] = 9, 9

    # plot
    PLOT_NUM = 5
    if len(df_exe_time_ms) > 0:
        with (save_dir / "exe_time_ms_mean.txt").open("w") as f:
            f.write(f"{df_exe_time_ms['data'].mean():.1f} [ms]")
        print(f"Average exe_time_ms: {df_exe_time_ms['data'].mean():.1f} [ms]")

    if len(df_initial_to_result_relative_pose) > 0:
        plt.subplot(PLOT_NUM, 1, 1)
        plt.plot(
            df_initial_to_result_relative_pose["timestamp"] / 1e9,
            df_initial_to_result_relative_pose["position.x"],
            label="x",
        )
        plt.plot(
            df_initial_to_result_relative_pose["timestamp"] / 1e9,
            df_initial_to_result_relative_pose["position.y"],
            label="y",
        )
        plt.plot(
            df_initial_to_result_relative_pose["timestamp"] / 1e9,
            df_initial_to_result_relative_pose["position.z"],
            label="z",
        )
        plt.xlabel("time [s]")
        plt.ylabel("diff_position [m]")
        plt.grid()
        plt.legend()

        r = Rotation.from_quat(
            df_initial_to_result_relative_pose[
                ["orientation.x", "orientation.y", "orientation.z", "orientation.w"]
            ].values,
        )
        angle = r.as_euler("xyz", degrees=True)
        plt.subplot(PLOT_NUM, 1, 2)
        plt.plot(
            df_initial_to_result_relative_pose["timestamp"] / 1e9,
            angle[:, 0],
            label="x",
        )
        plt.plot(
            df_initial_to_result_relative_pose["timestamp"] / 1e9,
            angle[:, 1],
            label="y",
        )
        plt.plot(
            df_initial_to_result_relative_pose["timestamp"] / 1e9,
            angle[:, 2],
            label="z",
        )
        plt.xlabel("time [s]")
        plt.ylabel("diff_angle [deg/s]")
        plt.grid()
        plt.legend()

    if len(df_ndt_pose_with_covariance) > 0:
        plt.subplot(PLOT_NUM, 1, 3)
        plt.plot(
            df_ndt_pose_with_covariance["timestamp"] / 1e9,
            np.sqrt(df_ndt_pose_with_covariance["covariance_position.xx"]),
            label="xx",
        )
        plt.plot(
            df_ndt_pose_with_covariance["timestamp"] / 1e9,
            np.sqrt(df_ndt_pose_with_covariance["covariance_position.yy"]),
            label="yy",
        )
        plt.plot(
            df_ndt_pose_with_covariance["timestamp"] / 1e9,
            np.sqrt(df_ndt_pose_with_covariance["covariance_position.zz"]),
            label="zz",
        )
        plt.xlabel("time [s]")
        plt.ylabel("stddev [m]")
        plt.ylim(bottom=0)
        plt.grid()
        plt.legend()

    if len(df_estimated_yaw_bias) > 0:
        plt.subplot(PLOT_NUM, 1, 5)
        plt.plot(
            df_estimated_yaw_bias["timestamp"] / 1e9,
            df_estimated_yaw_bias["data"],
            label="estimated_yaw_bias",
        )
        plt.xlabel("time [s]")
        plt.ylabel("estimated_yaw_bias [rad]")
        plt.grid()

    plt.subplot(PLOT_NUM, 1, 4)
    plt.plot(
        df_kinematic_state["timestamp"] / 1e9,
        df_kinematic_state["linear_velocity.x"],
        label="x",
    )
    plt.plot(
        df_kinematic_state["timestamp"] / 1e9,
        df_kinematic_state["linear_velocity.y"],
        label="y",
    )
    plt.plot(
        df_kinematic_state["timestamp"] / 1e9,
        df_kinematic_state["linear_velocity.z"],
        label="z",
    )
    plt.xlabel("time [s]")
    plt.ylabel("velocity [m/s]")
    plt.grid()
    plt.legend()
    plt.xticks(
        np.arange(
            df_kinematic_state["timestamp"].iloc[0] / 1e9,
            df_kinematic_state["timestamp"].iloc[-1] / 1e9,
            60,
        ),
        rotation=45,
    )

    plt.tight_layout()
    save_path = save_dir / "localization_result.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    plt.close()
    print(f"saved to {save_path}")

    # plot pose
    plot_pose(df_ndt_pose_with_covariance, "ndt", save_dir, df_value=None)
    plot_pose(df_ndt_pose_with_covariance, "exe_time_ms", save_dir, df_value=df_exe_time_ms)
    plot_pose(
        df_ndt_pose_with_covariance,
        "nearest_voxel_transformation_likelihood",
        save_dir,
        df_value=df_nearest_voxel_transformation_likelihood,
    )
    plot_pose(
        df_ndt_pose_with_covariance,
        "transform_probability",
        save_dir,
        df_value=df_transform_probability,
    )
    plot_pose(
        df_ndt_pose_with_covariance,
        "initial_to_result_relative_pose",
        save_dir,
        df_value=df_initial_to_result_relative_pose,
    )

    # if output_pose_array is not given, exit here
    if not output_pose_array:
        sys.exit(0)

    # visualize pose array
    pose_array_result_dir = save_dir / "pose_array"
    yaw_array_result_dir = save_dir / "yaw_array"
    pose_array_result_dir.mkdir(exist_ok=True)
    yaw_array_result_dir.mkdir(exist_ok=True)
    for i, row in df_marker.iterrows():
        curr_df = pd.DataFrame(row["marker"])
        curr_df = curr_df[
            ~(
                (curr_df["position.x"] == 0.0)
                * (curr_df["position.y"] == 0.0)
                * (curr_df["position.z"] == 0.0)
            )
        ]
        THRESHOLD = 30
        if len(curr_df) < THRESHOLD:
            continue
        position_x = curr_df["position.x"].to_numpy()
        position_y = curr_df["position.y"].to_numpy()
        position_x -= position_x[0]
        position_y -= position_y[0]
        orientation = curr_df[
            ["orientation.x", "orientation.y", "orientation.z", "orientation.w"]
        ].to_numpy()
        plt.figure()
        # Color by index (0 ~ 29), and draw arrows from 0 to 1, 1 to 2, ...
        for j in range(len(position_x) - 1):
            diff_x = position_x[j + 1] - position_x[j]
            diff_y = position_y[j + 1] - position_y[j]
            plt.arrow(
                position_x[j],
                position_y[j],
                diff_x,
                diff_y,
                color=plt.cm.jet(j / (len(position_x) - 1)),
                length_includes_head=True,
            )
        plt.title(f"iteration: {len(position_x) - 1}")
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.axis("equal")
        lim = 0.2
        plt.xlim(-lim, lim)
        plt.ylim(-lim, lim)
        plt.grid()
        plt.savefig(pose_array_result_dir / f"{i:08d}.png")
        plt.close()

        # visualize angle
        plt.figure()
        r_list = []
        p_list = []
        y_list = []
        rotation_list = Rotation.from_quat(orientation)
        first_r = rotation_list[0]
        for j in range(len(rotation_list)):
            curr_r = rotation_list[j]
            curr_r = curr_r * first_r.inv()
            r, p, y = curr_r.as_euler("xyz")
            r_list.append(r * 180 / np.pi)
            p_list.append(p * 180 / np.pi)
            y_list.append(y * 180 / np.pi)
        r_list_np = np.array(r_list)
        r_list_np -= r_list[0]
        p_list_np = np.array(p_list)
        p_list_np -= p_list[0]
        y_list_np = np.array(y_list)
        y_list_np -= y_list[0]
        plt.plot(r_list, label="roll")
        plt.plot(p_list, label="pitch")
        plt.plot(y_list, label="yaw")
        plt.xlabel("iteration")
        plt.ylabel("degree")
        plt.ylim(-1, 1)
        plt.grid()
        plt.legend()
        plt.savefig(yaw_array_result_dir / f"{i:08d}.png")
        plt.close()
