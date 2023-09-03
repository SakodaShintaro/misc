""" A script to replace images in a rosbag with new images.
"""

import argparse
from rosbags.rosbag2 import Reader, Writer
from rosbags.serde import serialize_cdr, deserialize_cdr
from glob import glob
import cv2


def parse_args():
    parser = argparse.ArgumentParser(
        description='Replace images in a rosbag with new images.')
    parser.add_argument('input_bag', type=str, help='Input rosbag.')
    parser.add_argument('output_bag', type=str, help='Output rosbag.')
    parser.add_argument('image_topic', type=str,
                        help='Image topic to replace.')
    parser.add_argument('image_dir', type=str,
                        help='Directory containing replacement images.')
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    input_bag = args.input_bag
    output_bag = args.output_bag
    image_topic = args.image_topic
    image_dir = args.image_dir

    image_path_list = sorted(glob(f'{image_dir}/*.png'))

    reader = Reader(input_bag)
    writer = Writer(output_bag)
    reader.open()
    writer.open()

    topic_name_to_connection = {}
    for connection in reader.connections:
        print(connection.ext.offered_qos_profiles)
        topic_name_to_connection[connection.topic] = writer.add_connection(connection.topic,
                                                                           connection.msgtype,
                                                                           offered_qos_profiles=connection.ext.offered_qos_profiles)

    index = 0
    for connection, timestamp, rawdata in reader.messages():
        if connection.topic == image_topic:
            msg = deserialize_cdr(rawdata, connection.msgtype)
            image = cv2.imread(image_path_list[index])
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            msg.data = image.flatten()
            rawdata = serialize_cdr(msg, connection.msgtype)
            index += 1
            print(f"index = {index}")
        writer.write(
            topic_name_to_connection[connection.topic], timestamp, rawdata)

    reader.close()
    writer.close()
