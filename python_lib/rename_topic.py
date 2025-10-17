"""A script to rename a topic in a rosbag."""

import argparse
from pathlib import Path

from rosbags.rosbag2 import Reader, Writer


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_bag", type=Path)
    parser.add_argument("topic_name_before", type=str)
    parser.add_argument("topic_name_after", type=str)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    input_bag = args.input_bag
    topic_name_before = args.topic_name_before
    topic_name_after = args.topic_name_after

    output_bag = input_bag.with_name(f"{input_bag.stem}_renamed{input_bag.suffix}")

    reader = Reader(input_bag)
    writer = Writer(output_bag)
    reader.open()
    writer.open()

    topic_name_to_connection = {}
    for connection in reader.connections:
        if connection.topic == topic_name_before:
            topic_name_to_connection[topic_name_after] = writer.add_connection(
                topic_name_after,
                connection.msgtype,
                offered_qos_profiles=connection.ext.offered_qos_profiles,
            )
        else:
            topic_name_to_connection[connection.topic] = writer.add_connection(
                connection.topic,
                connection.msgtype,
                offered_qos_profiles=connection.ext.offered_qos_profiles,
            )

    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == topic_name_before:
            writer.write(topic_name_to_connection[topic_name_after], timestamp, rawdata)
        else:
            writer.write(topic_name_to_connection[connection.topic], timestamp, rawdata)

    reader.close()
    writer.close()

    print(f"Saved to {output_bag}")
