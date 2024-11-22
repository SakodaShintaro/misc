"""A script to replace timestamp of a certain topic in a rosbag."""

import argparse
from pathlib import Path

from rosbags.rosbag2 import Reader, Writer
from rosbags.serde import deserialize_cdr, serialize_cdr


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Replace images in a rosbag with new images.")
    parser.add_argument("input_bag", type=Path, help="Input rosbag.")
    parser.add_argument("--topic_name", type=str, help="A topic to replace.")
    parser.add_argument("--add_time_millisec", type=float, default=0, help="Time to add millisec.")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    input_bag = args.input_bag
    topic_name = args.topic_name
    add_time_millisec = int(args.add_time_millisec)
    print(f"{add_time_millisec=}")

    output_bag = input_bag.with_name(f"{input_bag.stem}_replaced_timestamp")

    reader = Reader(str(input_bag))
    writer = Writer(str(output_bag))
    reader.open()
    writer.open()

    topic_name_to_connection = {}
    for connection in reader.connections:
        topic_name_to_connection[connection.topic] = writer.add_connection(
            connection.topic,
            connection.msgtype,
            offered_qos_profiles=connection.ext.offered_qos_profiles,
        )

    unit = int(1e9)

    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == topic_name:
            timestamp += add_time_millisec * int(1e6)
        writer.write(topic_name_to_connection[connection.topic], timestamp, rawdata)

    reader.close()
    writer.close()

    print(f"output_bag: {output_bag}")
