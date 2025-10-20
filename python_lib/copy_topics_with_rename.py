"""Copy specified topics from a source rosbag to a destination rosbag with renamed topic names."""

import argparse
from pathlib import Path
from shutil import rmtree

import rosbag2_py


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("source_bag", type=Path)
    parser.add_argument("destination_bag", type=Path)
    parser.add_argument("save_path", type=Path)
    parser.add_argument("--source_topics", nargs="+", required=True)
    parser.add_argument("--renamed_topics", nargs="+", required=True)
    return parser.parse_args()


def main():
    args = parse_args()
    source_bag = args.source_bag
    destination_bag = args.destination_bag
    save_path = args.save_path
    source_topics = args.source_topics
    renamed_topics = args.renamed_topics

    # Validate arguments
    if len(source_topics) != len(renamed_topics):
        raise ValueError("The number of source topics must match the number of renamed topics.")
    if len(set(renamed_topics)) != len(renamed_topics):
        raise ValueError("Renamed topic names must be unique.")

    # Create mapping from source to renamed topics
    topic_rename_map = dict(zip(source_topics, renamed_topics, strict=True))

    # Check if save_path exists and remove if it does
    if save_path.exists():
        if save_path.is_dir():
            rmtree(save_path)
        else:
            save_path.unlink()

    # Setup storage options
    storage_id = "sqlite3"
    serialization_format = "cdr"
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format,
    )

    # Open source bag
    source_reader = rosbag2_py.SequentialReader()
    source_reader.open(
        rosbag2_py.StorageOptions(uri=str(source_bag), storage_id=storage_id),
        converter_options,
    )

    # Get all topic metadata from source
    source_topics_metadata = list(source_reader.get_all_topics_and_types())
    source_topic_map = {topic.name: topic for topic in source_topics_metadata}

    # Verify all source topics exist
    missing_topics = [topic for topic in source_topics if topic not in source_topic_map]
    if missing_topics:
        raise RuntimeError(f"Topics not found in source bag: {', '.join(missing_topics)}")

    # Open destination bag
    destination_reader = rosbag2_py.SequentialReader()
    destination_reader.open(
        rosbag2_py.StorageOptions(uri=str(destination_bag), storage_id=storage_id),
        converter_options,
    )

    # Get all topic metadata from destination
    destination_topics_metadata = list(destination_reader.get_all_topics_and_types())
    destination_topic_map = {topic.name: topic for topic in destination_topics_metadata}

    # Check for conflicts between renamed topic names and existing destination topics
    conflicts = [name for name in renamed_topics if name in destination_topic_map]
    if conflicts:
        raise RuntimeError(
            f"Renamed topic names already exist in destination bag: {', '.join(conflicts)}"
        )

    # Create output writer
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(save_path), storage_id=storage_id),
        converter_options,
    )

    # Create all topics from destination bag
    for topic_metadata in destination_topics_metadata:
        writer.create_topic(topic_metadata)

    # Create renamed topics from source bag
    for src_topic, dst_topic in topic_rename_map.items():
        src_metadata = source_topic_map[src_topic]
        new_metadata = rosbag2_py.TopicMetadata(
            name=dst_topic,
            type=src_metadata.type,
            serialization_format=src_metadata.serialization_format,
            offered_qos_profiles=src_metadata.offered_qos_profiles,
        )
        writer.create_topic(new_metadata)

    # Copy all messages from destination bag
    destination_message_count = 0
    while destination_reader.has_next():
        topic_name, data, timestamp = destination_reader.read_next()
        writer.write(topic_name, data, timestamp)
        destination_message_count += 1

    # Reset source reader and set filter
    source_reader.seek(0)
    storage_filter = rosbag2_py.StorageFilter(topics=source_topics)
    source_reader.set_filter(storage_filter)

    # Copy messages from source bag with renamed topics
    source_message_count = 0
    while source_reader.has_next():
        topic_name, data, timestamp = source_reader.read_next()
        if topic_name in topic_rename_map:
            writer.write(topic_rename_map[topic_name], data, timestamp)
            source_message_count += 1

    print(f"Copied {destination_message_count} messages from destination bag")
    print(f"Copied {source_message_count} messages from source bag with renamed topics")
    print("Topic mappings:")
    for src, dst in topic_rename_map.items():
        print(f"  {src} -> {dst}")


if __name__ == "__main__":
    main()
