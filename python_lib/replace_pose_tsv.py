"""あるtsvのpose部分を別のtsvのpose部分に置き換えるスクリプト"""

import argparse
from pathlib import Path
import pandas as pd
from interpolate_pose import interpolate_pose


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("target_tsv", type=Path)
    parser.add_argument("source_tsv", type=Path)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    target_tsv = args.target_tsv
    source_tsv = args.source_tsv

    target_df = pd.read_csv(target_tsv, delimiter="\t")
    source_df = pd.read_csv(source_tsv, delimiter="\t")

    cond = (source_df.iloc[0]["timestamp"] <= target_df["timestamp"]) & (
        target_df["timestamp"] <= source_df.iloc[-1]["timestamp"]
    )

    print(f"{len(target_df)=}, {len(cond)=}")

    sliced_target_df = target_df[cond]

    interpolated_pose = interpolate_pose(source_df, sliced_target_df["timestamp"])

    print(interpolated_pose.head())

    keys = [
        "position.x",
        "position.y",
        "position.z",
        "orientation.x",
        "orientation.y",
        "orientation.z",
        "orientation.w",
    ]

    target_df.loc[cond, keys] = interpolated_pose[keys].values

    save_path = target_tsv.parent / f"replaced_{target_tsv.name}"
    target_df.to_csv(save_path, sep="\t", index=False)
