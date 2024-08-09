import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import argparse
import matplotlib.pyplot as plt
import pandas as pd
from pathlib import Path
from parse_functions import parse_stamp


def parse_args() -> argparse.Namespace:
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

    target_list = [
        "ndt_scan_matcher: scan_matching_status",
        "localization: ekf_localizer",
        "localization: localization_error_monitor",
        "localization: pose_instability_detector",
    ]
    data_dict: dict = {key: [] for key in target_list}

    unique_status_name = set()
    while reader.has_next():
        (topic, data, timestamp_rosbag) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        timestamp_header = parse_stamp(msg.header.stamp)
        for status in msg.status:
            for target in target_list:
                if target in status.name:
                    unique_status_name.add(status.name)
                    key_value_map = {kv.key: kv.value for kv in status.values}
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
            try:
                df[col] = df[col].astype(float)
                df[col] = df[col].apply(lambda x: int(x) if x.is_integer() else x)
            except ValueError:
                pass
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
    """
    execution_time is_local_optimal_solution_oscillation iteration_num lidar_topic_delay_time_sec  ...    state transform_probability timestamp_rosbag timestamp_header
    0       0.720000                                     0             1                   0.127052  ...  Aligned              6.466093      29410390260      29410390260
    1       0.771000                                     0             1                   0.107068  ...  Aligned              6.449519      29490406369      29490406369
    2       0.710000                                     0             1                   0.127048  ...  Aligned              6.489886      29610386175      29610386175
    3       0.641000                                     0             1                   0.122059  ...  Aligned              6.464482      29705397494      29705397494
    4       0.853000                                     0             1                   0.137061  ...  Aligned              6.501288      29820398988      29820398988
    """

    # plot
    key_list = [
        "execution_time",
        "iteration_num",
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
        plt.subplot(5, 2, i + 1)
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
    """
    is_activated  timestamp_rosbag  timestamp_header  ... twist_is_passed_mahalanobis_yaw twist_mahalanobis_yaw twist_mahalanobis_yaw_threshold
    68         True       29220424288       29220424288  ...                            True              0.000000                    10000.000000
    69         True       29240390636       29240390636  ...                            True              0.000000                    10000.000000
    70         True       29260386199       29260386199  ...                            True              0.000000                    10000.000000
    71         True       29280389433       29280389433  ...                            True              0.000000                    10000.000000
    72         True       29300399106       29300399106  ...                            True              0.000000                    10000.000000
    """

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
            key + "_threshold"
            if "mahalanobis" in key
            else key.replace("_size", "_warn_threshold")
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
    """
    print(df.head())
    diff_position_x:threshold diff_position_x:value diff_position_x:status diff_position_y:threshold diff_position_y:value  ... diff_angle_z:threshold diff_angle_z:value diff_angle_z:status timestamp_rosbag timestamp_header
    0                  1.000000              0.004715                     OK                  1.000000             -0.001417  ...               1.000000          -0.000191                  OK      59381901391      59361782133
    1                  1.000000              0.001659                     OK                  1.000000             -0.000304  ...               1.000000           0.000001                  OK      59881786413      59861793711
    2                  1.000000              0.000014                     OK                  1.000000             -0.000097  ...               1.000000          -0.000026                  OK      60381810160      60361815555
    3                  1.000000             -0.000325                     OK                  1.000000             -0.000363  ...               1.000000          -0.000008                  OK      60881809730      60861722314
    4                  1.000000              0.000125                     OK                  1.000000              0.000154  ...               1.000000          -0.000003                  OK      61381810093      61361766673
    print(df.columns)
    Index(['diff_position_x:threshold', 'diff_position_x:value',
        'diff_position_x:status', 'diff_position_y:threshold',
        'diff_position_y:value', 'diff_position_y:status',
        'diff_position_z:threshold', 'diff_position_z:value',
        'diff_position_z:status', 'diff_angle_x:threshold',
        'diff_angle_x:value', 'diff_angle_x:status', 'diff_angle_y:threshold',
        'diff_angle_y:value', 'diff_angle_y:status', 'diff_angle_z:threshold',
        'diff_angle_z:value', 'diff_angle_z:status', 'timestamp_rosbag',
        'timestamp_header'],
        dtype='object')
    """

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
    df = pd.DataFrame(data_dict["localization: localization_error_monitor"])
    """
    print(df.head())
    localization_error_ellipse localization_error_ellipse_lateral_direction     timestamp_rosbag     timestamp_header  level                                  message
    0                   2.185800                                     0.005914  1722926634905264942  1722926634785096275      2  ellipse size is over the expected range
    1                   2.329596                                     0.005917  1722926634908142198  1722926634805128505      2  ellipse size is over the expected range
    2                   2.488433                                     0.005921  1722926634921237057  1722926634826337806      2  ellipse size is over the expected range
    3                   3.066201                                     0.005941  1722926634944631758  1722926634898547179      2  ellipse size is over the expected range
    4                   3.073270                                     0.005946  1722926634952046863  1722926634909982648      2  ellipse size is over the expected range
    """
    # plot
    key_list = [
        "localization_error_ellipse",
        "localization_error_ellipse_lateral_direction",
        "level",
    ]
    plt.figure(figsize=(6.4 * 2, 4.8 * 2))
    for i, key in enumerate(key_list):
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
