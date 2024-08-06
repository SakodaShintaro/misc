""" rosbagからPoseStampedのPoseを抽出するスクリプト
"""

import argparse
import matplotlib.pyplot as plt
import os
from parse_functions import parse_rosbag


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--rosbag_path", type=str, required=True)
    parser.add_argument("--target_topic", type=str, required=True)
    parser.add_argument("--output_dir", type=str, required=True)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    rosbag_path = args.rosbag_path
    target_topic = args.target_topic
    output_dir = args.output_dir

    os.makedirs(output_dir, exist_ok=True)

    df_dict = parse_rosbag(rosbag_path, [target_topic])

    df = df_dict[target_topic]
    save_name = "__".join(target_topic.split("/")[1:])

    # plot xy
    plt.plot(df["position.x"], df["position.y"])
    plt.axis("equal")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.savefig(
        f"{output_dir}/{save_name}_trajectory.png",
        bbox_inches="tight",
        pad_inches=0.05,
        dpi=300,
    )
    plt.close()

    # plot differential
    plt.plot(df["position.x"].diff(), label="x")
    plt.plot(df["position.y"].diff(), label="y")
    plt.plot(df["position.z"].diff(), label="z")
    plt.plot(df["timestamp"].diff() / 1e9, label="timestamp")
    plt.xlabel("Frame number")
    plt.ylabel("diff (x,y,z[m], timestamp[sec])")
    plt.legend()
    plt.savefig(
        f"{output_dir}/{save_name}_differential.png",
        bbox_inches="tight",
        pad_inches=0.05,
        dpi=300,
    )
    plt.close()

    df.to_csv(
        f"{output_dir}/{save_name}.tsv", index=False, sep="\t", float_format="%.9f"
    )
