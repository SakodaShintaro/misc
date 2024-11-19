"""A script to parse diagnostics messages from a rosbag file."""

import argparse
import sys
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
    parser.add_argument("--start_time_from_zero", action="store_true")
    parser.add_argument("--check_empty", action="store_true")
    parser.add_argument("--print_status_name_set", action="store_true")
    return parser.parse_args()


def diag_name_to_filename(diag_name: str) -> str:
    return diag_name.replace(":", "_").replace(" ", "_")


if __name__ == "__main__":
    args = parse_args()
    rosbag_path = args.rosbag_path
    start_time_from_zero = args.start_time_from_zero
    check_empty = args.check_empty
    print_status_name_set = args.print_status_name_set

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

    status_name_set = set()

    while reader.has_next():
        (topic, data, timestamp_rosbag) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        timestamp_header = parse_stamp(msg.header.stamp)
        if check_empty and len(msg.status) == 0:
            print(msg)
            raise RuntimeError(f"Message status length is zero: {len(msg.status)=}")
        for status in msg.status:
            status_name_set.add(status.name)
            if status.name in target_list:
                key_value_map = {kv.key: kv.value for kv in status.values}  # noqa: PD011
                key_value_map["timestamp_rosbag"] = timestamp_rosbag
                key_value_map["timestamp_header"] = timestamp_header
                key_value_map["level"] = int.from_bytes(status.level, "big")
                key_value_map["message"] = status.message
                data_dict[status.name].append(key_value_map)

    if print_status_name_set:
        print(status_name_set)
        sys.exit(0)

    save_dir = rosbag_path.parent / "diagnostics_result"
    save_dir.mkdir(exist_ok=True)
    for key, data in data_dict.items():
        df = pd.DataFrame(data)
        for col in df.columns:
            if pd.api.types.is_numeric_dtype(df[col]):
                df[col] = df[col].astype(float)
                df[col] = df[col].apply(lambda x: int(x) if x.is_integer() else x)
        filename = diag_name_to_filename(key)
        df.to_csv(
            save_dir / f"{filename}.tsv",
            index=False,
            sep="\t",
            float_format="%.9f",
        )

    if start_time_from_zero:
        for df in data_dict.values():
            for key_value_map in df:
                key_value_map["timestamp_header"] = (
                    key_value_map["timestamp_header"] - df[0]["timestamp_header"]
                )
                key_value_map["timestamp_rosbag"] = (
                    key_value_map["timestamp_rosbag"] - df[0]["timestamp_rosbag"]
                )

    ####################
    # ndt_scan_matcher #
    ####################
    diag_name = "ndt_scan_matcher: scan_matching_status"
    df = pd.DataFrame(data_dict[diag_name])

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
    save_path = save_dir / f"{diag_name_to_filename(diag_name)}.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    print(f"Saved {save_path}")

    #################
    # ekf_localizer #
    #################
    diag_name = "localization: ekf_localizer"
    df = pd.DataFrame(data_dict[diag_name])
    df = df[df["is_activated"] == "True"]

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
    save_path = save_dir / f"{diag_name_to_filename(diag_name)}.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    print(f"Saved {save_path}")

    #############################
    # pose_instability_detector #
    #############################
    diag_name = "localization: pose_instability_detector"
    df = pd.DataFrame(data_dict[diag_name])

    plt.figure(figsize=(6.4 * 2, 4.8 * 2))
    # row1 position
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

    # row2 angle
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
    save_path = save_dir / f"{diag_name_to_filename(diag_name)}.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    print(f"Saved {save_path}")

    ##############################
    # localization_error_monitor #
    ##############################
    diag_name = "localization_error_monitor: ellipse_error_status"
    df = pd.DataFrame(data_dict[diag_name])
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
    save_path = save_dir / f"{diag_name_to_filename(diag_name)}.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    print(f"Saved {save_path}")
