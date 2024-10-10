"""A script to parse diagnostics messages from a rosbag file."""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from parse_functions import parse_stamp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("rosbag_path", type=Path)
    parser.add_argument("--utc_time", action="store_true")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    rosbag_path = args.rosbag_path
    utc_time = args.utc_time

    serialization_format = "cdr"
    storage_options = rosbag2_py.StorageOptions(
        uri=str(rosbag_path),
        storage_id="sqlite3",
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format,
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    target_topic = "/diagnostics"
    storage_filter = rosbag2_py.StorageFilter(topics=[target_topic])
    reader.set_filter(storage_filter)

    target_list = [
        "ndt_scan_matcher: scan_matching_status",
        "localization: ekf_localizer",
        "localization_error_monitor: ellipse_error_status",
        "localization: pose_instability_detector",
    ]
    data_dict: dict = {key: [] for key in target_list}

    unique_status_name = set()
    while reader.has_next():
        (topic, data, timestamp_rosbag) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        timestamp_header = parse_stamp(msg.header.stamp)
        if len(msg.status) == 0:
            print(msg)
            raise RuntimeError(f"Message status length is zero: {len(msg.status)=}")
        for status in msg.status:
            for target in target_list:
                if target in status.name:
                    unique_status_name.add(status.name)
                    key_value_map = {kv.key: kv.value for kv in status.values}  # noqa: PD011
                    key_value_map["timestamp_rosbag"] = timestamp_rosbag
                    key_value_map["timestamp_header"] = timestamp_header
                    key_value_map["level"] = int.from_bytes(status.level, "big")
                    key_value_map["message"] = status.message
                    data_dict[target].append(key_value_map)

    print("unique_status_name")
    for name in sorted(unique_status_name):
        print(f"  {name}")

    save_dir = rosbag_path.parent / "diagnostics_result"
    save_dir.mkdir(exist_ok=True)
    for key, data in data_dict.items():
        df = pd.DataFrame(data)
        for col in df.columns:
            if pd.api.types.is_numeric_dtype(df[col]):
                df[col] = df[col].astype(float)
                df[col] = df[col].apply(lambda x: int(x) if x.is_integer() else x)
        filename = key.replace(":", "_").replace(" ", "_")
        df.to_csv(
            save_dir / f"{filename}.tsv",
            index=False,
            sep="\t",
            float_format="%.9f",
        )

    ####################
    # ndt_scan_matcher #
    ####################
    df = pd.DataFrame(data_dict["ndt_scan_matcher: scan_matching_status"])
    if not utc_time:
        df["timestamp_header"] = (df["timestamp_header"] - df["timestamp_header"].min()) / 1e9
        df["timestamp_rosbag"] = (df["timestamp_rosbag"] - df["timestamp_rosbag"].min()) / 1e9

    # plot
    key_list = [
        "execution_time",
        "iteration_num",
        "sensor_points_size",
        "sensor_points_delay_time_sec",
        "skipping_publish_num",
        "transform_probability",
        "transform_probability_diff",
        "nearest_voxel_transformation_likelihood",
        "nearest_voxel_transformation_likelihood_diff",
        "local_optimal_solution_oscillation_num",
        "distance_initial_to_result",
    ]
    plt.figure(figsize=(6.4 * 2, 4.8 * 2))
    for i, key in enumerate(key_list):
        if key not in df.columns:
            print(f"Skip {key}")
            continue
        df[key] = df[key].astype(float)
        df = df.dropna(subset=[key])
        plt.subplot(4, 3, i + 1)
        plt.plot(df["timestamp_header"], df[key], label=key)
        if key == "nearest_voxel_transformation_likelihood":
            plt.plot(
                df["timestamp_header"],
                [2.3 for _ in range(len(df))],
                label="threshold",
                linestyle="dashed",
            )
        plt.xlabel("time [s]")
        plt.title(f"{key}")
        plt.ylim(bottom=min(0, df[key].min()))
        plt.grid()

    plt.tight_layout()
    save_path = save_dir / "diagnostics_ndt_scan_matcher.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    print(f"Saved {save_path}")

    #################
    # ekf_localizer #
    #################
    df = pd.DataFrame(data_dict["localization: ekf_localizer"])
    df = df[df["is_activated"] == "True"]

    if not utc_time:
        df["timestamp_header"] = (df["timestamp_header"] - df["timestamp_header"].min()) / 1e9
        df["timestamp_rosbag"] = (df["timestamp_rosbag"] - df["timestamp_rosbag"].min()) / 1e9

    # plot
    key_list = [
        "pose_mahalanobis_distance",
        "twist_mahalanobis_distance",
        "cov_ellipse_long_axis_size",
        "cov_ellipse_lateral_direction_size",
    ]

    plt.figure(figsize=(6.4 * 2, 4.8 * 2))
    for i, key in enumerate(key_list):
        if key not in df.columns:
            print(f"Skip {key}")
            continue
        df[key] = df[key].astype(float)
        key_threshold = (
            key + "_threshold" if "mahalanobis" in key else key.replace("_size", "_warn_threshold")
        )
        df[key_threshold] = df[key_threshold].astype(float)
        plt.subplot(4, 1, (i + 1))
        plt.plot(df["timestamp_header"], df[key], label=key)
        plt.plot(df["timestamp_header"], df[key_threshold], label=key_threshold)
        plt.xlabel("time [s]")
        plt.title(f"{key}")
        plt.grid()

    plt.tight_layout()
    save_path = save_dir / "diagnostics_ekf_localizer.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    print(f"Saved {save_path}")

    #############################
    # pose_instability_detector #
    #############################
    df = pd.DataFrame(data_dict["localization: pose_instability_detector"])
    if not utc_time:
        df["timestamp_header"] = (df["timestamp_header"] - df["timestamp_header"].min()) / 1e9
        df["timestamp_rosbag"] = (df["timestamp_rosbag"] - df["timestamp_rosbag"].min()) / 1e9

    # 2行に分けて表示する
    plt.figure(figsize=(6.4 * 2, 4.8 * 2))
    # 1行目:position
    plt.subplot(2, 1, 1)
    key_list = [
        "diff_position_x",
        "diff_position_y",
        "diff_position_z",
    ]
    for key in key_list:
        key_value = key + ":value"
        key_threshold = key + ":threshold"
        if key_value not in df.columns:
            print(f"Skip {key}")
            continue
        df[key_value] = df[key_value].astype(float)
        df[key_threshold] = df[key_threshold].astype(float)
        plt.plot(df["timestamp_header"], df[key_value], label=key_value)
        plt.plot(
            df["timestamp_header"],
            df[key_threshold],
            linestyle="dashed",
        )
        plt.plot(
            df["timestamp_header"],
            -df[key_threshold],
            linestyle="dashed",
        )
    plt.legend(bbox_to_anchor=(1.05, 1), loc="upper left", borderaxespad=0)
    plt.xlabel("time [s]")
    plt.ylabel("diff_position [m]")
    plt.grid()

    # 2行目:angle
    plt.subplot(2, 1, 2)
    key_list = [
        "diff_angle_x",
        "diff_angle_y",
        "diff_angle_z",
    ]
    for key in key_list:
        key_value = key + ":value"
        key_threshold = key + ":threshold"
        if key_value not in df.columns:
            print(f"Skip {key}")
            continue
        df[key_value] = df[key_value].astype(float)
        df[key_threshold] = df[key_threshold].astype(float)
        plt.plot(df["timestamp_header"], df[key_value], label=key_value)
        plt.plot(
            df["timestamp_header"],
            df[key_threshold],
            linestyle="dashed",
        )
        plt.plot(
            df["timestamp_header"],
            -df[key_threshold],
            linestyle="dashed",
        )
    plt.legend(bbox_to_anchor=(1.05, 1), loc="upper left", borderaxespad=0)
    plt.xlabel("time [s]")
    plt.ylabel("diff_angle [rad]")
    plt.grid()

    plt.tight_layout()
    save_path = save_dir / "diagnostics_pose_instability_detector.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    print(f"Saved {save_path}")

    ##############################
    # localization_error_monitor #
    ##############################
    df = pd.DataFrame(data_dict["localization_error_monitor: ellipse_error_status"])
    if not utc_time:
        df["timestamp_header"] = (df["timestamp_header"] - df["timestamp_header"].min()) / 1e9
        df["timestamp_rosbag"] = (df["timestamp_rosbag"] - df["timestamp_rosbag"].min()) / 1e9

    # plot
    key_list = [
        "localization_error_ellipse",
        "localization_error_ellipse_lateral_direction",
        "level",
    ]
    plt.figure(figsize=(6.4 * 2, 4.8 * 2))
    for _, key in enumerate(key_list):
        if key not in df.columns:
            print(f"Skip {key}")
            continue
        df[key] = df[key].astype(float)
        plt.plot(df["timestamp_header"], df[key], label=key)
    plt.xlabel("time [s]")
    plt.ylabel("error_ellipse [m]")
    plt.grid()
    plt.legend()
    plt.tight_layout()
    save_path = save_dir / "diagnostics_localization_error_monitor.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    print(f"Saved {save_path}")
