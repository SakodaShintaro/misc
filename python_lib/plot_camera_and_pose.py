"""左にカメラ画像、右にPoseを表示するスクリプト"""

import argparse
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("images_dir", type=Path)
    parser.add_argument("camera_name", type=str)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    images_dir = args.images_dir
    camera_name = args.camera_name


    image_path_list = sorted((images_dir / camera_name).glob("*.png"))
    pose_df = pd.read_csv(images_dir / f"pose_{camera_name}.tsv", sep="\t", index_col=0)

    timestamp_list = [int(f.stem) for f in image_path_list]
    pose_df = pose_df[pose_df["timestamp"].isin(timestamp_list)]

    travel_distance = 0
    timestamp0 = timestamp_list[0]

    for i, image_path in enumerate(image_path_list):
        if i >= 1:
            prev_pose = pose_df.iloc[i - 1]
            curr_pose = pose_df.iloc[i]
            travel_distance += (
                (curr_pose["position.x"] - prev_pose["position.x"]) ** 2
                + (curr_pose["position.y"] - prev_pose["position.y"]) ** 2
            ) ** 0.5

        image = plt.imread(image_path)
        fig, ax = plt.subplots(1, 2, figsize=(12, 6))
        ax[0].imshow(image)
        ax[0].set_title(image_path.stem)

        timestamp = int(image_path.stem)
        curr_pose = pose_df.iloc[i]
        ax[1].plot(pose_df["position.x"], pose_df["position.y"], "b")
        ax[1].plot(curr_pose["position.x"], curr_pose["position.y"], "ro")
        ax[1].set_title(f"timestamp_diff: {(timestamp - timestamp0) / 1e9:.1f} sec, travel_distance: {travel_distance:.2f} m")
        ax[1].set_xlabel("x [m]")
        ax[1].set_ylabel("y [m]")
        ax[1].grid()
        ax[1].set_aspect("equal")

        save_path = images_dir / f"{camera_name}_and_pose_plot" / f"{timestamp}.png"
        save_path.parent.mkdir(exist_ok=True, parents=True)
        plt.savefig(save_path)
        print(f"Saved plot to {save_path}")

        plt.cla()
        plt.clf()
        plt.close()
