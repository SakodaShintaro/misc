import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import argparse
import matplotlib.pyplot as plt
import os
import pandas as pd


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('rosbag_path', type=str)
    parser.add_argument('target_topic', type=str)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    rosbag_path = args.rosbag_path
    target_topic = args.target_topic

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

    storage_filter = rosbag2_py.StorageFilter(topics=[target_topic])
    reader.set_filter(storage_filter)

    time_list = []
    value_list = []
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        t_from_msg = msg.stamp.sec * 1e9 + msg.stamp.nanosec
        t_from_msg /= 1e9
        time_list.append(t_from_msg)
        value_list.append(msg.data)

    df = pd.DataFrame()
    df['time'] = time_list
    df['value'] = value_list
    df['time'] -= df['time'][0]

    save_dir = os.path.dirname(rosbag_path) if os.path.isfile(
        rosbag_path) else rosbag_path
    save_name = target_topic[1:].replace('/', '__')
    df.to_csv(f'{save_dir}/{save_name}.csv', index=False)

    # plot
    plt.plot(df['time'], df['value'])
    plt.xlabel('time [s]')
    plt.ylabel(target_topic)
    plt.grid()
    plt.savefig(f'{save_dir}/{save_name}.png')
    plt.close()
    print(f'saved {save_dir}/{save_name}.png')
