import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import argparse
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('rosbag_path', type=str)
    return parser.parse_args()


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

    target_topic = "/localization/pose_estimator/exe_time_ms"
    storage_filter = rosbag2_py.StorageFilter(topics=[target_topic])
    reader.set_filter(storage_filter)

    name_to_appear_num = defaultdict(int)
    name_to_time_list = defaultdict(list)

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        print(msg)
        timestamp = int(msg.stamp.sec) + int(msg.stamp.nanosec) * 1e-9
        print(timestamp)
        for status in msg.status:
            name_to_appear_num[status.name] += 1
            name_to_time_list[status.name].append(timestamp)

    for key, value in name_to_appear_num.items():
        print(f"{key}: {value}")

    n = len(name_to_time_list)
    col = 3
    row = (n + col - 1) // col

    for i, (key, value) in enumerate(name_to_time_list.items()):
        plt.subplot(row, col, i + 1)
        diff = np.diff(value)
        plt.plot(diff)
        plt.title(key)
        plt.xlabel('Index')
        plt.ylabel('Diff(s)')
    plt.show()
