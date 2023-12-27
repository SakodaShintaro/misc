import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import argparse
import matplotlib.pyplot as plt
import numpy as np
import os


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('rosbag_path', type=str)
    return parser.parse_args()


def pretty_print_diagnostics(msg):
    print(
        f"\n[Diagnostic Message @ {msg.header.stamp.sec}.{msg.header.stamp.nanosec}]")
    for status in msg.status:
        print(f"  - Level: {status.level}")
        print(f"    Name: {status.name}")
        print(f"    Message: {status.message}")
        print(f"    Hardware ID: {status.hardware_id}")
        print(f"    Key-Value Pairs:")
        for kv in status.values:
            print(f"      {kv.key}: {kv.value}")


if __name__ == "__main__":
    args = parse_args()
    rosbag_path = args.rosbag_path
    serialization_format = 'cdr'
    storage_options = rosbag2_py.StorageOptions(
        uri=rosbag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {
        topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    target_topic = "/diagnostics"
    storage_filter = rosbag2_py.StorageFilter(topics=[target_topic])
    reader.set_filter(storage_filter)

    time_list = []
    oscillation_num_list = []
    is_oscillation_list = []
    exe_time_list = []
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        skip = (len(msg.status) == 0)
        for status in msg.status:
            skip = skip or ("ndt_scan_matcher" not in status.name)
        if skip:
            continue

        assert len(msg.status) == 1, f"{msg}, {skip}"
        message = msg.status[0].message
        key_value_map = {kv.key: kv.value for kv in msg.status[0].values}

        t /= 1e9
        time_list.append(t)
        print(key_value_map)
        oscillation_num = int(key_value_map["oscillation_count"])
        oscillation_num_list.append(oscillation_num)
        is_oscillation = int(
            key_value_map["is_local_optimal_solution_oscillation"])
        is_oscillation_list.append(is_oscillation)
        exe_time = float(key_value_map["execution_time"])
        exe_time_list.append(exe_time)

    time_list = np.array(time_list)
    oscillation_num_list = np.array(oscillation_num_list)
    is_oscillation_list = np.array(is_oscillation_list)

    time_list -= time_list[0]

    # 3行でプロット
    plt.subplot(3, 1, 1)
    plt.plot(time_list, oscillation_num_list, label="oscillation_num")
    plt.ylabel("oscillation_num")

    plt.subplot(3, 1, 2)
    plt.plot(time_list, is_oscillation_list, label="is_oscillation", color="green")
    plt.ylabel("is_oscillation")

    plt.subplot(3, 1, 3)
    plt.plot(time_list, exe_time_list, label="execution_time", color="red")
    plt.ylabel("execution_time [ms]")
    plt.xlabel("time [ms]")

    plt.tight_layout()

    save_dir = os.path.dirname(rosbag_path) if os.path.isfile(
        rosbag_path) else rosbag_path
    save_path = f"{save_dir}/oscillation.png"
    plt.savefig(save_path, bbox_inches='tight', pad_inches=0.05)
    plt.close()
    print(f"Saved to {save_path}")
