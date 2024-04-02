import argparse
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt


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

    """
        timestamp             x             y          z        qw        qx        qy        qz
    0  1.705818e+09  75765.198292  37856.350441  49.088126  0.292594 -0.382282 -0.603830  0.635325
    1  1.705818e+09  75762.156223  37853.649057  49.185281  0.296397 -0.344127 -0.620331  0.639464
    2  1.705818e+09  75758.096157  37848.363993  49.294780  0.227352 -0.273301 -0.658080  0.663738
    3  1.705818e+09  75754.647354  37841.355752  49.425510 -0.162863  0.201452  0.683421 -0.682516
    4  1.705818e+09  75750.404898  37831.761622  49.613135 -0.125832  0.176798  0.691506 -0.689006
    """

    assert len(image_paths) == len(df)

    save_dir = target_dir / "images_with_poses"
    save_dir.mkdir(exist_ok=True)

    travel_distance = 0

    for image_path, (i, row) in zip(image_paths, df.iterrows()):
        if i > 0:
            travel_distance += (
                (row["x"] - df.loc[i - 1, "x"]) ** 2
                + (row["y"] - df.loc[i - 1, "y"]) ** 2
            ) ** 0.5

        fig, ax = plt.subplots(1, 2, figsize=(12, 6))
        ax[0].imshow(plt.imread(image_path))
        ax[0].axis("off")

        ax[1].plot(df["x"], df["y"])
        ax[1].scatter(row["x"], row["y"], color="red")
        ax[1].set_aspect("equal")
        ax[1].set_xlabel("x [m]")
        ax[1].set_ylabel("y [m]")
        ax[1].grid()
        ax[1].set_title(f"travel_distance = {travel_distance:.1f} m")
        plt.savefig(save_dir / f"{i:08d}.png")
        plt.close()
