"""rosbagから各トピックのFPSを求めるスクリプト."""

import argparse
import subprocess


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("rosbag_path", type=str)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    rosbag_path = args.rosbag_path
    rosbag_info = subprocess.run(
        f"ros2 bag info {rosbag_path}",
        shell=True,
        check=True,
        capture_output=True,
    ).stdout.decode()

    NUM_ELEMENTS = 2
    lines = rosbag_info.split("\n")
    duration = None
    for line in lines:
        elements = line.replace("Topic information: ", "").split("|")
        elements = [i.strip() for i in elements]
        topic_name = None
        topic_count = None
        for element in elements:
            kv = element.split(":")
            if len(kv) != NUM_ELEMENTS:
                continue
            key, value = kv
            if key == "Duration":
                duration = float(value.replace("s", ""))
                print(f"duration = {duration} sec")
            elif key == "Topic":
                topic_name = value
            elif key == "Count":
                topic_count = int(value)
        if topic_name is None:
            continue
        fps = topic_count / duration
        print(f"{topic_name:<50}\tcount = {topic_count}\tfps = {fps:.1f}")
