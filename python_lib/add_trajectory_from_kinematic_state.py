"""Generate predicted Trajectory messages from localization kinematic state in a rosbag."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from shutil import rmtree

import rosbag2_py
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("source_rosbag_path", type=Path)
    parser.add_argument("--kinematic_topic", type=str, default="/localization/kinematic_state")
    parser.add_argument("--trajectory_topic", type=str, default="/actual_trajectory")
    parser.add_argument("--prediction_duration", type=float, default=8.0)
    parser.add_argument("--skip", type=int, default=5)
    parser.add_argument("--output_suffix", type=str, default="with_actual_trajectory")
    return parser.parse_args()


def float_to_duration(seconds: float) -> Duration:
    duration = Duration()
    sec = int(math.floor(seconds))
    nanosec = int(round((seconds - sec) * 1_000_000_000))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    duration.sec = sec
    duration.nanosec = nanosec
    return duration


def build_points(
    records: list[tuple[int, object]],
    start_index: int,
    skip: int,
    prediction_duration: float,
) -> list[TrajectoryPoint]:
    if prediction_duration <= 0.0:
        raise ValueError("prediction_duration must be a positive value.")
    if skip <= 0:
        raise ValueError("skip must be a positive integer.")

    start_time_ns = records[start_index][0]
    limit_ns = start_time_ns + int(prediction_duration * 1_000_000_000)

    points: list[TrajectoryPoint] = []
    for idx in range(start_index, len(records)):
        header_ns = records[idx][0]
        if header_ns > limit_ns:
            break
        if (idx - start_index) % skip != 0:
            continue
        msg = records[idx][1]
        trajectory_point = TrajectoryPoint()
        elapsed_sec = max(header_ns - start_time_ns, 0) / 1_000_000_000
        trajectory_point.time_from_start = float_to_duration(elapsed_sec)
        pose = msg.pose.pose
        trajectory_point.pose.position.x = pose.position.x
        trajectory_point.pose.position.y = pose.position.y
        trajectory_point.pose.position.z = pose.position.z
        trajectory_point.pose.orientation = pose.orientation
        twist = msg.twist.twist
        trajectory_point.longitudinal_velocity_mps = twist.linear.x
        trajectory_point.lateral_velocity_mps = twist.linear.y
        trajectory_point.acceleration_mps2 = 0.0
        trajectory_point.heading_rate_rps = twist.angular.z
        trajectory_point.front_wheel_angle_rad = 0.0
        trajectory_point.rear_wheel_angle_rad = 0.0
        points.append(trajectory_point)
    return points


def main() -> None:
    args = parse_args()
    source_rosbag_path: Path = args.source_rosbag_path
    kinematic_topic: str = args.kinematic_topic
    trajectory_topic: str = args.trajectory_topic
    prediction_duration: float = args.prediction_duration
    skip: int = args.skip
    output_suffix: str = args.output_suffix.strip()

    if not source_rosbag_path.exists():
        raise FileNotFoundError(f"Source rosbag not found: {source_rosbag_path}")
    if kinematic_topic == trajectory_topic:
        raise ValueError("trajectory_topic must be different from kinematic_topic.")
    if skip <= 0:
        raise ValueError("skip must be a positive integer.")

    if not output_suffix:
        raise ValueError("output_suffix must be a non-empty string.")
    output_rosbag_path = source_rosbag_path.parent / f"{source_rosbag_path.name}_{output_suffix}"
    if output_rosbag_path == source_rosbag_path:
        raise ValueError("Generated output path matches source path; use a different suffix.")

    if output_rosbag_path.exists():
        if output_rosbag_path.is_dir():
            rmtree(output_rosbag_path)
        else:
            output_rosbag_path.unlink()

    serialization_format = "cdr"
    storage_options = rosbag2_py.StorageOptions(uri=str(source_rosbag_path), storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format,
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topics_metadata = list(reader.get_all_topics_and_types())
    type_map = {topic.name: topic.type for topic in topics_metadata}

    if kinematic_topic not in type_map:
        raise RuntimeError(f"Topic '{kinematic_topic}' not found in source rosbag.")
    if trajectory_topic in type_map:
        raise RuntimeError(f"Topic '{trajectory_topic}' already exists in source rosbag.")

    kinematic_msg_type = get_message(type_map[kinematic_topic])

    kinematic_records: list[tuple[int, object]] = []
    while reader.has_next():
        topic_name, data, _ = reader.read_next()
        if topic_name != kinematic_topic:
            continue
        msg = deserialize_message(data, kinematic_msg_type)
        header_stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        kinematic_records.append((header_stamp_ns, msg))

    if not kinematic_records:
        raise RuntimeError(f"No messages found for topic '{kinematic_topic}'.")

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    writer_storage_options = rosbag2_py.StorageOptions(
        uri=str(output_rosbag_path),
        storage_id="sqlite3",
    )
    writer = rosbag2_py.SequentialWriter()
    writer.open(writer_storage_options, converter_options)

    for topic_metadata in topics_metadata:
        writer.create_topic(topic_metadata)
    trajectory_metadata = rosbag2_py.TopicMetadata(
        name=trajectory_topic,
        type="autoware_planning_msgs/msg/Trajectory",
        serialization_format=serialization_format,
        offered_qos_profiles="",
    )
    writer.create_topic(trajectory_metadata)

    trajectory_count = 0
    kinematic_index = 0

    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()
        writer.write(topic_name, data, timestamp)

        if topic_name != kinematic_topic:
            continue

        _, current_msg = kinematic_records[kinematic_index]
        trajectory_msg = Trajectory()
        trajectory_msg.header = current_msg.header
        trajectory_msg.points = build_points(
            kinematic_records,
            kinematic_index,
            skip,
            prediction_duration,
        )
        writer.write(trajectory_topic, serialize_message(trajectory_msg), timestamp)
        trajectory_count += 1
        kinematic_index += 1

    print(f"Added {trajectory_count} Trajectory messages to '{trajectory_topic}'.")
    print(f"Saved rosbag to: {output_rosbag_path}")


if __name__ == "__main__":
    main()
