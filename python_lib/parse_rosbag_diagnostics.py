import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import argparse
import matplotlib.pyplot as plt
import pandas as pd
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("rosbag_path", type=Path)
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

    target_topic = "/diagnostics"
    storage_filter = rosbag2_py.StorageFilter(topics=[target_topic])
    reader.set_filter(storage_filter)

    target_list = ["ndt_scan_matcher", "ekf_localizer"]
    data_dict = {key: [] for key in target_list}

    time_list = []
    while reader.has_next():
        (topic, data, timestamp_rosbag) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        timestamp_header = int(msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec)
        for status in msg.status:
            for target in target_list:
                if target in status.name:
                    key_value_map = {kv.key: kv.value for kv in status.values}
                    key_value_map["timestamp_rosbag"] = timestamp_rosbag
                    key_value_map["timestamp_header"] = timestamp_header
                    data_dict[target].append(key_value_map)

    # ndt_scan_matcher
    df = pd.DataFrame(data_dict["ndt_scan_matcher"])
    """
    execution_time is_local_optimal_solution_oscillation iteration_num lidar_topic_delay_time_sec  ...    state transform_probability timestamp_rosbag timestamp_header
    0       0.720000                                     0             1                   0.127052  ...  Aligned              6.466093      29410390260      29410390260
    1       0.771000                                     0             1                   0.107068  ...  Aligned              6.449519      29490406369      29490406369
    2       0.710000                                     0             1                   0.127048  ...  Aligned              6.489886      29610386175      29610386175
    3       0.641000                                     0             1                   0.122059  ...  Aligned              6.464482      29705397494      29705397494
    4       0.853000                                     0             1                   0.137061  ...  Aligned              6.501288      29820398988      29820398988
    """
    # fix timestamp
    df["timestamp_header"] = df["timestamp_header"].astype(int)
    df["timestamp_header"] -= df["timestamp_header"].iloc[0]
    df["timestamp_header"] /= 1e9

    # plot
    key_list = [
        "execution_time",
        "iteration_num",
        "lidar_topic_delay_time_sec",
        "skipping_publish_num",
        "oscillation_count",
        "is_local_optimal_solution_oscillation",
        "transform_probability",
        "nearest_voxel_transformation_likelihood",
    ]
    plt.figure(figsize=(6.4 * 2, 4.8 * 2))
    for i, key in enumerate(key_list):
        df[key] = df[key].astype(float)
        plt.subplot(4, 2, i + 1)
        plt.plot(df["timestamp_header"], df[key], label=key)
        plt.xlabel("time [s]")
        plt.title(f"{key}")
        plt.ylim(bottom=0)
        plt.grid()

    plt.tight_layout()
    save_path = rosbag_path.parent / "diagnostics_ndt_scan_matcher.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    print(f"Saved {save_path}")

    # ekf_localizer
    df = pd.DataFrame(data_dict["ekf_localizer"])
    df = df[df["is_activated"] == "True"]
    """
    is_activated  timestamp_rosbag  timestamp_header  ... twist_is_passed_mahalanobis_yaw twist_mahalanobis_yaw twist_mahalanobis_yaw_threshold
    68         True       29220424288       29220424288  ...                            True              0.000000                    10000.000000
    69         True       29240390636       29240390636  ...                            True              0.000000                    10000.000000
    70         True       29260386199       29260386199  ...                            True              0.000000                    10000.000000
    71         True       29280389433       29280389433  ...                            True              0.000000                    10000.000000
    72         True       29300399106       29300399106  ...                            True              0.000000                    10000.000000
    """

    # fix timestamp
    df["timestamp_header"] = df["timestamp_header"].astype(int)
    df["timestamp_header"] -= df["timestamp_header"].iloc[0]
    df["timestamp_header"] /= 1e9

    # plot
    key_list = [
        "pose_mahalanobis_distance",
        "twist_mahalanobis_distance",
    ]

    plt.figure(figsize=(6.4 * 1, 4.8 * 1))
    y_max = [0, 0, 0, 0]
    y_min = [0, 0, 0, 0]
    for i, key in enumerate(key_list):
        df[key] = df[key].astype(float)
        key_threshold = key + "_threshold"
        df[key_threshold] = df[key_threshold].astype(float)
        div = i // 2
        mod = i % 2
        area = mod * 2 + div
        y_max[div] = max(y_max[div], df[key].max())
        y_min[div] = min(y_min[div], df[key].min())
        plt.subplot(2, 2, (area + 1))
        plt.plot(df["timestamp_header"], df[key], label=key)
        plt.plot(df["timestamp_header"], df[key_threshold], label=key_threshold)
        plt.xlabel("time [s]")
        plt.title(f"{key}")
        plt.grid()

    plt.tight_layout()
    save_path = rosbag_path.parent / "diagnostics_ekf_localizer.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    print(f"Saved {save_path}")
