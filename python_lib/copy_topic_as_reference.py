"""Copy a topic from a rosbag to another rosbag as a reference topic."""

import argparse
import pathlib

import pandas as pd
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("source1_rosbag_path", type=pathlib.Path)
    parser.add_argument("source2_rosbag_path", type=pathlib.Path)
    parser.add_argument("output_rosbag_path", type=pathlib.Path)
    parser.add_argument("--target_topic", type=str, default="/localization/kinematic_state")
    parser.add_argument("--output_topic_prefix", type=str, default="reference_")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    source1_rosbag_path = args.source1_rosbag_path
    source2_rosbag_path = args.source2_rosbag_path
    output_rosbag_path = args.output_rosbag_path
    target_topic = args.target_topic
    output_topic_prefix = args.output_topic_prefix

    # read source1 rosbag (get target topic)
    serialization_format = "cdr"
    storage_options = rosbag2_py.StorageOptions(uri=str(source1_rosbag_path), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format,
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    elements = target_topic.split("/")
    output_topic_name = "/".join(elements[:-1] + [output_topic_prefix + elements[-1]])
    target_topics = [target_topic]
    storage_filter = rosbag2_py.StorageFilter(topics=target_topics)
    reader.set_filter(storage_filter)

    data_list = []
    while reader.has_next():
        (topic, data, timestamp_rosbag) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        timestamp_header = int(msg.header.stamp.sec) + int(msg.header.stamp.nanosec) * 1e-9
        if topic == target_topic:
            data_list.append(
                {
                    "timestamp_header": timestamp_header,
                    "timestamp_rosbag": timestamp_rosbag,
                    "msg": msg,
                },
            )
        else:
            raise ValueError(f"Unknown topic: {topic}")
    df_data = pd.DataFrame(data_list)

    # prepare source2 rosbag (get all topics without target topic)
    storage_options = rosbag2_py.StorageOptions(uri=str(source2_rosbag_path), storage_id="sqlite3")
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    # prepare output rosbag
    storage_options = rosbag2_py.StorageOptions(uri=str(output_rosbag_path), storage_id="sqlite3")
    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_options, converter_options)

    # create_topic
    for topic in type_map:
        if topic == output_topic_name:
            continue
        topic_info = rosbag2_py.TopicMetadata(
            name=topic,
            type=type_map[topic],
            serialization_format=serialization_format,
        )
        writer.create_topic(topic_info)
    topic_info = rosbag2_py.TopicMetadata(
        name=output_topic_name,
        type="nav_msgs/msg/Odometry",
        serialization_format=serialization_format,
    )
    writer.create_topic(topic_info)

    first_timestamp_header = None
    last_timestamp_header = None
    while reader.has_next():
        (topic, data, timestamp_rosbag) = reader.read_next()
        if topic == output_topic_name:
            continue  # skip if already exists
        writer.write(topic, data, timestamp_rosbag)
        if first_timestamp_header is None:
            first_timestamp_header = timestamp_rosbag
        last_timestamp_header = timestamp_rosbag

    # write target topic to output rosbag
    add_count = 0
    for _, row in df_data.iterrows():
        timestamp_header = row["timestamp_header"] * 1e9
        timestamp_rosbag = row["timestamp_rosbag"]
        msg = row["msg"]
        if timestamp_header < first_timestamp_header or last_timestamp_header < timestamp_header:
            continue

        data = serialize_message(msg)
        writer.write(output_topic_name, data, timestamp_rosbag)
        add_count += 1

    print(f"Added {add_count} messages.")
