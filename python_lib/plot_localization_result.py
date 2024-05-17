import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import argparse
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation
from pathlib import Path
from parse_functions import parse_msg
from interpolate_pose import interpolate_pose
from collections import defaultdict


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("rosbag_path", type=Path)
    return parser.parse_args()


def plot_pose(df_pose: pd.DataFrame, value_name: str, save_dir: Path, df_value=None):
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
    plt.scatter(df["position.x"], df["position.y"], c=color)
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

    target_topics = [
        "/localization/pose_estimator/exe_time_ms",
        "/localization/pose_estimator/initial_to_result_relative_pose",
        "/localization/pose_estimator/iteration_num",
        "/localization/pose_estimator/ndt_marker",
        "/localization/pose_estimator/nearest_voxel_transformation_likelihood",
        "/localization/pose_estimator/pose",
        "/localization/pose_estimator/transform_probability",
        "/localization/pose_twist_fusion_filter/kinematic_state",
    ]

    storage_filter = rosbag2_py.StorageFilter(topics=target_topics)
    reader.set_filter(storage_filter)

    topic_name_to_data = defaultdict(list)
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        if topic in target_topics:
            topic_name_to_data[topic].append(parse_msg(msg, msg_type))

    # rosbag path may be the path to the db3 file, or it may be the path to the directory containing it
    save_dir = (
        rosbag_path.parent if rosbag_path.is_dir() else rosbag_path.parent.parent
    ) / "localization_result"
    save_dir.mkdir(exist_ok=True)

    # save as csv
    for topic_name in target_topics:
        df = pd.DataFrame(topic_name_to_data[topic_name])
        filename = topic_name.replace("/localization/", "").replace("/", "_")
        df.to_csv(
            save_dir / f"{filename}.tsv",
            index=False,
            sep="\t",
            float_format="%.9f",
        )

    df_exe_time_ms = pd.DataFrame(
        topic_name_to_data["/localization/pose_estimator/exe_time_ms"]
    )
    df_nearest_voxel_transformation_likelihood = pd.DataFrame(
        topic_name_to_data[
            "/localization/pose_estimator/nearest_voxel_transformation_likelihood"
        ]
    )
    df_transform_probability = pd.DataFrame(
        topic_name_to_data["/localization/pose_estimator/transform_probability"]
    )
    df_iteration_num = pd.DataFrame(
        topic_name_to_data["/localization/pose_estimator/iteration_num"]
    )
    df_ndt_pose = pd.DataFrame(topic_name_to_data["/localization/pose_estimator/pose"])
    df_initial_to_result_relative_pose = pd.DataFrame(
        topic_name_to_data[
            "/localization/pose_estimator/initial_to_result_relative_pose"
        ]
    )
    df_marker = pd.DataFrame(
        topic_name_to_data["/localization/pose_estimator/ndt_marker"]
    )
    df_kinematic_state = pd.DataFrame(
        topic_name_to_data["/localization/pose_twist_fusion_filter/kinematic_state"]
    )

    # dataとして変動量のノルムを計算
    df_initial_to_result_relative_pose["data"] = (
        df_initial_to_result_relative_pose.apply(
            lambda x: np.linalg.norm(
                [x["position.x"], x["position.y"], x["position.z"]]
            ),
            axis=1,
        )
    )

    # plotの縦サイズを大きくする
    plt.rcParams["figure.figsize"] = 9, 12

    # plot
    plt.subplot(5, 1, 1)
    plt.plot(df_iteration_num["timestamp"], df_iteration_num["data"])
    plt.xlabel("time [s]")
    plt.ylabel("iteration_num")
    plt.ylim(bottom=0)
    plt.grid()

    plt.subplot(5, 1, 2)
    plt.plot(df_exe_time_ms["timestamp"], df_exe_time_ms["data"])
    plt.xlabel("time [s]")
    plt.ylabel("exe_time [ms]")
    plt.ylim(bottom=0)
    plt.grid()
    with open(save_dir / "exe_time_ms_mean.txt", "w") as f:
        f.write(f"{df_exe_time_ms['data'].mean():.1f} [ms]")
    print(f"Average exe_time_ms: {df_exe_time_ms['data'].mean():.1f} [ms]")

    plt.subplot(5, 1, 3)
    plt.plot(
        df_nearest_voxel_transformation_likelihood["timestamp"],
        df_nearest_voxel_transformation_likelihood["data"],
    )
    plt.xlabel("time [s]")
    plt.ylabel("NVTL")
    plt.ylim(bottom=0)
    plt.grid()

    plt.subplot(5, 1, 4)
    plt.plot(df_transform_probability["timestamp"], df_transform_probability["data"])
    plt.xlabel("time [s]")
    plt.ylabel("TP")
    plt.ylim(bottom=0)
    plt.grid()

    plt.subplot(5, 1, 5)
    plt.plot(
        df_initial_to_result_relative_pose["timestamp"],
        df_initial_to_result_relative_pose["position.x"],
        label="x",
    )
    plt.plot(
        df_initial_to_result_relative_pose["timestamp"],
        df_initial_to_result_relative_pose["position.y"],
        label="y",
    )
    plt.plot(
        df_initial_to_result_relative_pose["timestamp"],
        df_initial_to_result_relative_pose["position.z"],
        label="z",
    )
    plt.xlabel("time [s]")
    plt.ylabel("diff_position [m]")
    plt.grid()

    plt.tight_layout()
    save_path = save_dir / "localization_result.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    plt.close()
    print(f"saved to {save_path}")

    # poseの可視化
    plt.rcParams["figure.figsize"] = 9, 9

    # poseの可視化
    plot_pose(df_ndt_pose, "ndt", save_dir, df_value=None)
    plot_pose(df_ndt_pose, "exe_time_ms", save_dir, df_value=df_exe_time_ms)
    plot_pose(df_ndt_pose, "iteration_num", save_dir, df_value=df_iteration_num)
    plot_pose(
        df_ndt_pose,
        "nearest_voxel_transformation_likelihood",
        save_dir,
        df_value=df_nearest_voxel_transformation_likelihood,
    )
    plot_pose(
        df_ndt_pose,
        "transform_probability",
        save_dir,
        df_value=df_transform_probability,
    )
    plot_pose(
        df_ndt_pose,
        "initial_to_result_relative_pose",
        save_dir,
        df_value=df_initial_to_result_relative_pose,
    )

    # pose_arrayを気合で可視化
    plt.rcParams["figure.figsize"] = 9, 9
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
        if len(curr_df) < 30:
            continue
        position_x = curr_df["position.x"].values
        position_y = curr_df["position.y"].values
        position_x -= position_x[0]
        position_y -= position_y[0]
        orientation = curr_df[
            ["orientation.x", "orientation.y", "orientation.z", "orientation.w"]
        ].values
        plt.figure()
        # indexで色付け(0 ~ 29), 0から1, 1から2,... へと矢印を描画
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
        plt.savefig(f'{str(pose_array_result_dir/ f"{i:08d}.png")}')
        plt.close()

        # quaternionを気合で可視化
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
        r_list = np.array(r_list)
        r_list -= r_list[0]
        p_list = np.array(p_list)
        p_list -= p_list[0]
        y_list = np.array(y_list)
        y_list -= y_list[0]
        plt.plot(r_list, label="roll")
        plt.plot(p_list, label="pitch")
        plt.plot(y_list, label="yaw")
        plt.xlabel("iteration")
        plt.ylabel("yaw [deg]")
        plt.ylim(-1, 1)
        plt.grid()
        plt.legend()
        plt.savefig(f'{str(yaw_array_result_dir/ f"{i:08d}.png")}')
        plt.close()
