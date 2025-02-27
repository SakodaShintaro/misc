"""A script to extract pose from rosbag and save as tsv."""

import argparse
from pathlib import Path

from parse_functions import parse_rosbag


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("rosbag_path", type=Path)
    parser.add_argument("--save_dir", type=Path, default=None)
    parser.add_argument("--target_topics", type=str, required=True, nargs="+")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    rosbag_path = args.rosbag_path
    target_topics = args.target_topics
    save_dir = args.save_dir

    if save_dir is None:
        if rosbag_path.is_dir():  # if specified directory containing db3 files
            save_dir = rosbag_path.parent / "pose_tsv"
        else:  # if specified db3 file directly
            save_dir = rosbag_path.parent.parent / "pose_tsv"

    df_dict = parse_rosbag(str(rosbag_path), target_topics)

    for target_topic in target_topics:
        save_name = "__".join(target_topic.split("/")[1:])
        df = df_dict[target_topic]
        df.to_csv(
            f"{save_dir}/{save_name}.tsv",
            index=False,
            sep="\t",
            float_format="%.9f",
        )
