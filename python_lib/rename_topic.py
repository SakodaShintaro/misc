"""Rename a topic in a ROS 2 bag."""

import argparse
import sys
from pathlib import Path
from shutil import rmtree

import rosbag2_py


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("input_bag", type=Path)
    parser.add_argument("--topic_names_before", nargs="+", required=True)
    parser.add_argument("--topic_names_after", nargs="+", required=True)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    input_bag = args.input_bag
    topic_names_before = args.topic_names_before
    topic_names_after = args.topic_names_after

    if len(topic_names_before) != len(topic_names_after):
        raise ValueError("The number of source topics must match the number of target topics.")  # noqa: EM101
    if len(set(topic_names_after)) != len(topic_names_after):
        raise ValueError("Target topic names must be unique.")  # noqa: EM101

    rename_map = dict(zip(topic_names_before, topic_names_after, strict=True))

    bag_dir = input_bag if input_bag.is_dir() else input_bag.parent

    output_bag = bag_dir.with_name(f"{bag_dir.name}_renamed")
    if output_bag.exists():
        rmtree(output_bag)

    storage_id = "sqlite3"
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage_id),
        converter_options,
    )
    topic_metadata_list = list(reader.get_all_topics_and_types())

    if all(before == after for before, after in rename_map.items()):
        print("Topic names are identical; nothing to rename.")
        sys.exit(0)

    original_names = {topic.name for topic in topic_metadata_list}
    missing_topics = [name for name in topic_names_before if name not in original_names]
    if missing_topics:
        raise RuntimeError(f"Topics not found in bag: {', '.join(missing_topics)}")

    conflict_topics = {
        new_name
        for new_name in topic_names_after
        if new_name in original_names and new_name not in rename_map
    }
    if conflict_topics:
        raise RuntimeError(
            "Target topics already exist in bag: " + ", ".join(sorted(conflict_topics)),
        )

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output_bag), storage_id=storage_id),
        converter_options,
    )

    topic_name_map: dict[str, str] = {}
    created_topics: set[str] = set()

    for topic in topic_metadata_list:
        new_name = rename_map.get(topic.name, topic.name)
        topic_name_map[topic.name] = new_name
        if new_name in created_topics:
            continue
        new_metadata = rosbag2_py.TopicMetadata(
            name=new_name,
            type=topic.type,
            serialization_format=topic.serialization_format,
            offered_qos_profiles=topic.offered_qos_profiles,
        )
        writer.create_topic(new_metadata)
        created_topics.add(new_name)

    message_count = 0
    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()
        writer.write(topic_name_map[topic_name], data, timestamp)
        message_count += 1

    print(f"Saved {message_count} messages to {output_bag}")
