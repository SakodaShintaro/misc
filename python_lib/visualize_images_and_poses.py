import argparse
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
from tqdm import tqdm
from scipy.spatial.transform import Rotation


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("target_dir", type=Path)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    target_dir = args.target_dir

    images_dir = target_dir / "images"
    image_paths = sorted(images_dir.glob("*.png"))

    df = pd.read_csv(target_dir / "pose.tsv", sep="\t", index_col=0)

    assert len(image_paths) == len(df)

    save_dir = target_dir / "images_with_poses"
    save_dir.mkdir(exist_ok=True)

    travel_distance = 0

    bar = tqdm(total=len(image_paths))

    x_min = df["position.x"].min()
    x_max = df["position.x"].max()
    y_min = df["position.y"].min()
    y_max = df["position.y"].max()
    x_range = x_max - x_min
    y_range = y_max - y_min
    max_range = max(x_range, y_range)
    ego_length = max_range / 20
    default_orientation = [0, 0, ego_length]

    for image_path, (i, row) in zip(image_paths, df.iterrows()):
        if i > 0:
            travel_distance += (
                (row["position.x"] - df.loc[i - 1, "position.x"]) ** 2
                + (row["position.y"] - df.loc[i - 1, "position.y"]) ** 2
            ) ** 0.5

        fig, ax = plt.subplots(1, 2, figsize=(12, 6))
        ax[0].imshow(plt.imread(image_path))
        ax[0].axis("off")

        r = Rotation.from_quat(
            [
                row["orientation.x"],
                row["orientation.y"],
                row["orientation.z"],
                row["orientation.w"],
            ]
        )
        ego = r.apply(default_orientation)

        ax[1].plot(df["position.x"], df["position.y"])
        ax[1].scatter(row["position.x"], row["position.y"], color="red")
        ax[1].quiver(
            row["position.x"],
            row["position.y"],
            ego[0],
            ego[1],
            color="red",
            scale=1,
            scale_units="xy",
            angles="xy",
            width=0.01,
            headwidth=6,
        )
        ax[1].set_aspect("equal")
        ax[1].set_xlabel("x [m]")
        ax[1].set_ylabel("y [m]")
        ax[1].grid()
        ax[1].set_title(f"travel_distance = {travel_distance:.1f} m")
        plt.savefig(save_dir / f"{i:08d}.png")
        plt.close()
        bar.update(1)
