"""日々実行される結果からグラフを作成する."""

import argparse
import os
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("target_dir", type=Path)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    target_dir = args.target_dir

    result_dirs = [
        Path(target_dir) / f for f in os.listdir(target_dir) if (Path(target_dir) / f).is_dir()
    ]

    result_dirs.sort()

    suffix_error = (
        "localization__pose_estimator__pose_with_covariance_result/relative_pose_summary.tsv"
    )
    suffix_time = "localization_result/pose_estimator_exe_time_ms.tsv"

    date_list = []
    error_list = []
    time_list = []

    for result_dir in result_dirs:
        tsv_path_error = result_dir / suffix_error
        tsv_path_time = result_dir / suffix_time
        if not tsv_path_error.exists() or not tsv_path_time.exists():
            continue

        date = result_dir.name.split("_")
        date = date[0] + date[1]
        date = pd.to_datetime(date, format="%Y%m%d%H%M%S")
        date_list.append(date)

        df_error = pd.read_csv(tsv_path_error, sep="\t")
        error = df_error["error_mean"].to_numpy()[0]
        error_list.append(error)

        df_time = pd.read_csv(tsv_path_time, sep="\t")
        time = df_time["value"].mean() if "value" in df_time.columns else df_time["data"].mean()
        time_list.append(time)

    if len(date_list) == 0:
        print("no data")
        sys.exit(0)

    plt.figure(figsize=(6.4, 4.8 * 1.5))

    # plot error
    plt.subplot(2, 1, 1)
    percentile_90 = np.percentile(error_list, 90)
    plt.plot(date_list, error_list, marker=".", color="C0")
    plt.ylabel("error_mean")
    plt.ylim(0, percentile_90 * 2)
    plt.tick_params(axis="x", which="both", bottom=False, top=False, labelbottom=False)
    plt.grid()

    # plot time
    plt.subplot(2, 1, 2)
    percentile_90 = np.percentile(time_list, 90)
    plt.plot(date_list, time_list, marker=".", color="C1")
    plt.xlabel("date")
    plt.ylabel("exe_time_ms")
    plt.xticks(rotation=90)
    plt.ylim(0, percentile_90 * 2)
    plt.grid()

    plt.tight_layout()

    save_path = target_dir / "error_mean.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    print(f"save {save_path}")
