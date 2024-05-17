import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import argparse
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation
from pathlib import Path
from parse_functions import parse_PoseStamped, parse_Float32Stamped, parse_MarkerArray
from interpolate_pose import interpolate_pose


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("rosbag_path", type=Path)
    return parser.parse_args()


def plot_pose_with_value(df_pose, df_value, value_name, save_dir):
    df = interpolate_pose(df_pose, df_value["timestamp"].values)
    plt.scatter(df["x"], df["y"], c=df_value["data"])
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
    ]

    storage_filter = rosbag2_py.StorageFilter(topics=target_topics)
    reader.set_filter(storage_filter)

    exe_time_ms_array = []
    nearest_voxel_transformation_likelihood_array = []
    transform_probability_array = []
    iteration_num_array = []
    ndt_pose_array = []
    initial_to_result_relative_pose_array = []
    marker_array = []
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        if topic == "/localization/pose_estimator/exe_time_ms":
            exe_time_ms_array.append(parse_Float32Stamped(msg))
        elif (
            topic
            == "/localization/pose_estimator/nearest_voxel_transformation_likelihood"
        ):
            nearest_voxel_transformation_likelihood_array.append(
                parse_Float32Stamped(msg)
            )
        elif topic == "/localization/pose_estimator/transform_probability":
            transform_probability_array.append(parse_Float32Stamped(msg))
        elif topic == "/localization/pose_estimator/iteration_num":
            iteration_num_array.append(parse_Float32Stamped(msg))
        elif topic == "/localization/pose_estimator/initial_to_result_relative_pose":
            initial_to_result_relative_pose_array.append(parse_PoseStamped(msg))
        elif topic == "/localization/pose_estimator/ndt_marker":
            marker_array.append(parse_MarkerArray(msg))
        elif topic == "/localization/pose_estimator/pose":
            ndt_pose_array.append(parse_PoseStamped(msg))

    df_exe_time_ms = pd.DataFrame(exe_time_ms_array)
    df_nearest_voxel_transformation_likelihood = pd.DataFrame(
        nearest_voxel_transformation_likelihood_array
    )
    df_transform_probability = pd.DataFrame(transform_probability_array)
    df_iteration_num = pd.DataFrame(iteration_num_array)
    df_ndt_pose = pd.DataFrame(ndt_pose_array)
    df_initial_to_result_relative_pose = pd.DataFrame(
        initial_to_result_relative_pose_array
    )
    df_marker = pd.DataFrame(marker_array)

    # dataとして変動量のノルムを計算
    df_initial_to_result_relative_pose["data"] = (
        df_initial_to_result_relative_pose.apply(
            lambda x: np.linalg.norm(
                [x["position.x"], x["position.y"], x["position.z"]]
            ),
            axis=1,
        )
    )

    # rosbag path may be the path to the db3 file, or it may be the path to the directory containing it
    save_dir = (
        rosbag_path.parent if rosbag_path.is_dir() else rosbag_path.parent.parent
    ) / "localization_result"
    save_dir.mkdir(exist_ok=True)

    # plotの縦サイズを大きくする
    plt.rcParams["figure.figsize"] = 9, 12

    # plot
    plt.subplot(5, 1, 1)
    plt.plot(df_iteration_num["timestamp"], df_iteration_num["data"])
    plt.xlabel("time [s]")
    plt.ylabel("iteration_num")
    plt.ylim(bottom=0)
    plt.grid()
    df_iteration_num.to_csv(
        save_dir / "iteration_num.tsv", index=False, sep="\t", float_format="%.9f"
    )

    plt.subplot(5, 1, 2)
    plt.plot(df_exe_time_ms["timestamp"], df_exe_time_ms["data"])
    plt.xlabel("time [s]")
    plt.ylabel("exe_time [ms]")
    plt.ylim(bottom=0)
    plt.grid()
    with open(save_dir / "exe_time_ms_mean.txt", "w") as f:
        f.write(f"{df_exe_time_ms['data'].mean():.1f} [ms]")
    print(f"Average exe_time_ms: {df_exe_time_ms['data'].mean():.1f} [ms]")
    df_exe_time_ms.to_csv(
        save_dir / "exe_time_ms.tsv", index=False, sep="\t", float_format="%.9f"
    )

    plt.subplot(5, 1, 3)
    plt.plot(
        df_nearest_voxel_transformation_likelihood["timestamp"],
        df_nearest_voxel_transformation_likelihood["data"],
    )
    plt.xlabel("time [s]")
    plt.ylabel("NVTL")
    plt.ylim(bottom=0)
    plt.grid()
    df_nearest_voxel_transformation_likelihood.to_csv(
        save_dir / "nearest_voxel_transformation_likelihood.tsv",
        index=False,
        sep="\t",
        float_format="%.9f",
    )

    plt.subplot(5, 1, 4)
    plt.plot(df_transform_probability["timestamp"], df_transform_probability["data"])
    plt.xlabel("time [s]")
    plt.ylabel("TP")
    plt.ylim(bottom=0)
    plt.grid()
    df_transform_probability.to_csv(
        save_dir / "transform_probability.tsv",
        index=False,
        sep="\t",
        float_format="%.9f",
    )

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
    df_initial_to_result_relative_pose.to_csv(
        save_dir / "initial_to_result_relative_pose.tsv",
        index=False,
        sep="\t",
        float_format="%.9f",
    )

    plt.tight_layout()
    save_path = save_dir / "localization_result.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    plt.close()
    print(f"saved to {save_path}")

    # poseの可視化
    plt.rcParams["figure.figsize"] = 9, 9
    plt.scatter(df_ndt_pose["position.x"], df_ndt_pose["position.y"], s=1)
    plt.axis("equal")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.grid()
    save_path = save_dir / "ndt_pose.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    plt.close()
    print(f"saved to {save_path}")
    df_ndt_pose.to_csv(save_dir / "ndt_pose.tsv", index=False, sep="\t", float_format="%.9f")

    # 色つきposeの可視化
    df_renamed = df_ndt_pose.rename(
        columns={
            "position.x": "x",
            "position.y": "y",
            "position.z": "z",
            "orientation.x": "qx",
            "orientation.y": "qy",
            "orientation.z": "qz",
            "orientation.w": "qw",
        }
    )

    plot_pose_with_value(df_renamed, df_exe_time_ms, "exe_time_ms", save_dir)
    plot_pose_with_value(df_renamed, df_iteration_num, "iteration_num", save_dir)
    plot_pose_with_value(
        df_renamed,
        df_nearest_voxel_transformation_likelihood,
        "nearest_voxel_transformation_likelihood",
        save_dir,
    )
    plot_pose_with_value(
        df_renamed, df_transform_probability, "transform_probability", save_dir
    )
    plot_pose_with_value(
        df_renamed,
        df_initial_to_result_relative_pose,
        "initial_to_result_relative_pose",
        save_dir,
    )

    # pose_arrayを気合で可視化
    plt.rcParams["figure.figsize"] = 9, 9
    pose_array_result_dir = save_dir / "pose_array"
    yaw_array_result_dir = save_dir / "yaw_array"
    pose_array_result_dir.mkdir(exist_ok=True)
    yaw_array_result_dir.mkdir(exist_ok=True)
    for i, row in df_marker.iterrows():
        curr_df = pd.DataFrame(row["marker"])
        curr_df = curr_df[~((curr_df["position.x"] == 0.0) * (curr_df["position.y"] == 0.0) * (curr_df["position.z"] == 0.0))]
        if len(curr_df) < 30:
            continue
        position_x = curr_df["position.x"].values
        position_y = curr_df["position.y"].values
        position_x -= position_x[0]
        position_y -= position_y[0]
        orientation = curr_df[["orientation.x", "orientation.y", "orientation.z", "orientation.w"]].values
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
