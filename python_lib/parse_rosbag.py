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
    gyro_bias_message = []
    estimated_gyro_bias_x = [0]
    estimated_gyro_bias_y = [0]
    estimated_gyro_bias_z = [0]
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        skip = False
        for status in msg.status:
            skip = "bias" not in status.name
        if skip:
            continue

        assert len(msg.status) == 1
        message = msg.status[0].message
        key_value_map = {kv.key: kv.value for kv in msg.status[0].values}

        t /= 1e9
        time_list.append(t)
        gyro_bias_message.append(key_value_map["gyro_bias"])
        estimated_gyro_bias_x.append(
            float(key_value_map.get("estimated_gyro_bias_x", estimated_gyro_bias_x[-1])))
        estimated_gyro_bias_y.append(
            float(key_value_map.get("estimated_gyro_bias_y", estimated_gyro_bias_y[-1])))
        estimated_gyro_bias_z.append(
            float(key_value_map.get("estimated_gyro_bias_z", estimated_gyro_bias_z[-1])))

    time_list = np.array(time_list)
    gyro_bias_message = np.array(gyro_bias_message)
    estimated_gyro_bias_x = np.array(estimated_gyro_bias_x)
    estimated_gyro_bias_y = np.array(estimated_gyro_bias_y)
    estimated_gyro_bias_z = np.array(estimated_gyro_bias_z)

    estimated_gyro_bias_x = estimated_gyro_bias_x[1:]
    estimated_gyro_bias_y = estimated_gyro_bias_y[1:]
    estimated_gyro_bias_z = estimated_gyro_bias_z[1:]

    time_list -= time_list[0]

    plt.plot(time_list, estimated_gyro_bias_x, label="estimated gyro bias x")
    plt.plot(time_list, estimated_gyro_bias_y, label="estimated gyro bias y")
    plt.plot(time_list, estimated_gyro_bias_z, label="estimated gyro bias z")

    THRESHOLD = 0.0015
    plt.plot(time_list, +np.ones_like(time_list) * THRESHOLD, "--", color="red", label="threshold")
    plt.plot(time_list, -np.ones_like(time_list) * THRESHOLD, "--", color="red", label="threshold")

    plt.legend()
    plt.xlabel("time [s]")
    plt.ylabel("gyro bias diff [rad/s]")
    save_dir = os.path.dirname(rosbag_path)
    save_path = f"{save_dir}/estimated_gyro_bias.png"
    plt.savefig(save_path, bbox_inches='tight', pad_inches=0.05)
    plt.close()
    print(f"Saved to {save_path}")
