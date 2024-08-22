"""A script to add bias to imu."""

import argparse
from pathlib import Path

from rosbags.rosbag2 import Reader, Writer
from rosbags.serde import deserialize_cdr, serialize_cdr


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_bag", type=Path)
    parser.add_argument("topic_name", type=str)
    parser.add_argument("--bias_x", type=float, default=0.0)
    parser.add_argument("--bias_y", type=float, default=0.0)
    parser.add_argument("--bias_z", type=float, default=0.0)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    input_bag = args.input_bag
    topic_name = args.topic_name
    bias_x = args.bias_x
    bias_y = args.bias_y
    bias_z = args.bias_z

    output_bag = input_bag.with_name(f"{input_bag.stem}_bias{input_bag.suffix}")

    reader = Reader(input_bag)
    writer = Writer(output_bag)
    reader.open()
    writer.open()

    topic_name_to_connection = {}
    for connection in reader.connections:
        topic_name_to_connection[connection.topic] = writer.add_connection(
            connection.topic,
            connection.msgtype,
            offered_qos_profiles=connection.ext.offered_qos_profiles,
        )

    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == topic_name:
            msg = deserialize_cdr(rawdata, connection.msgtype)
            msg.angular_velocity.x += bias_x
            msg.angular_velocity.y += bias_y
            msg.angular_velocity.z += bias_z
            rawdata = serialize_cdr(msg, connection.msgtype)  # noqa: PLW2901
        writer.write(topic_name_to_connection[connection.topic], timestamp, rawdata)

    reader.close()
    writer.close()

    print(f"Saved to {output_bag}")
