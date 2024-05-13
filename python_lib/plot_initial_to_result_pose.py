import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import argparse
import matplotlib.pyplot as plt
import numpy as np
import pathlib
import pandas as pd
from interpolate_pose import interpolate_pose
from parse_functions import parse_PoseStamped, parse_PoseWithCovarianceStamped


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("rosbag_path", type=pathlib.Path)
    return parser.parse_args()


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
        "/localization/pose_estimator/initial_to_result_relative_pose",
        "/localization/pose_twist_fusion_filter/kinematic_state",
    ]
    storage_filter = rosbag2_py.StorageFilter(topics=target_topics)
    reader.set_filter(storage_filter)

    target_list = [
        "/localization/pose_estimator/initial_to_result_relative_pose",
        "/localization/pose_twist_fusion_filter/kinematic_state",
    ]
    data_dict = {key: [] for key in target_list}

    while reader.has_next():
        (topic, data, timestamp_rosbag) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        timestamp_header = (
            int(msg.header.stamp.sec) + int(msg.header.stamp.nanosec) * 1e-9
        )
        if topic == "/localization/pose_estimator/initial_to_result_relative_pose":
            data_dict[topic].append(parse_PoseStamped(msg))
        elif topic == "/localization/pose_twist_fusion_filter/kinematic_state":
            data_dict[topic].append(parse_PoseWithCovarianceStamped(msg))
        else:
            assert False, f"Unknown topic: {topic}"

    df_pose_initial_to_result_ndt = pd.DataFrame(
        data_dict["/localization/pose_estimator/initial_to_result_relative_pose"]
    )
    df_ekf_pose = pd.DataFrame(
        data_dict["/localization/pose_twist_fusion_filter/kinematic_state"]
    )
    print(f"{len(df_pose_initial_to_result_ndt)=}")
    print(f"{len(df_ekf_pose)=}")

    df_ekf_pose = interpolate_pose(
        df_ekf_pose,
        df_pose_initial_to_result_ndt["timestamp"],
        POSITIONS_KEY=["position.x", "position.y", "position.z"],
        ORIENTATIONS_KEY=[
            "orientation.w",
            "orientation.x",
            "orientation.y",
            "orientation.z",
        ],
    )
    print(f"{len(df_pose_initial_to_result_ndt)=}")
    print(f"{len(df_ekf_pose)=}")

    # rosbag path may be the path to the db3 file, or it may be the path to the directory containing it
    save_dir = (
        rosbag_path.parent if rosbag_path.is_dir() else rosbag_path.parent.parent
    ) / "pose_initial_to_result_ndt"
    save_dir.mkdir(exist_ok=True)

    # plot pose_initial_to_result_ndt
    plt.plot(
        df_pose_initial_to_result_ndt["timestamp"],
        df_pose_initial_to_result_ndt["position.x"],
        label="x",
    )
    plt.plot(
        df_pose_initial_to_result_ndt["timestamp"],
        df_pose_initial_to_result_ndt["position.y"],
        label="y",
    )
    plt.plot(
        df_pose_initial_to_result_ndt["timestamp"],
        df_pose_initial_to_result_ndt["position.z"],
        label="z",
    )
    plt.xlabel("timestamp")
    plt.ylabel("pose_initial_to_result_ndt[m]")
    plt.legend()
    save_path = save_dir / "pose_initial_to_result_ndt.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    print(f"Saved to {save_path}")
    plt.close()

    # plot norm of pose_initial_to_result_ndt on trajectory of ekf_pose
    diff_pose = df_pose_initial_to_result_ndt[["position.x", "position.y", "position.z"]].values
    diff_pose_norm = np.linalg.norm(diff_pose, axis=1)

    plt.scatter(
        df_ekf_pose["position.x"],
        df_ekf_pose["position.y"],
        c=diff_pose_norm,
        cmap="viridis",
    )
    plt.colorbar(label="pose_initial_to_result_ndt[m]")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.axis("equal")
    save_path = save_dir / "pose_initial_to_result_ndt_on_ekf_pose.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    print(f"Saved to {save_path}")
    plt.close()
