""" 日々実行される結果からグラフを作成する
"""

import argparse
import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('target_dir', type=str)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    target_dir = args.target_dir

    result_dirs = files_dir = [
        os.path.join(target_dir, f) for f in os.listdir(target_dir) if os.path.isdir(os.path.join(target_dir, f))
    ]

    result_dirs.sort()

    suffix = "localization__pose_twist_fusion_filter__pose_result/relative_pose_summary.tsv"

    date_list = []
    value_list = []

    for result_dir in result_dirs:
        tsv_path = os.path.join(result_dir, suffix)
        if not os.path.exists(tsv_path):
            continue
        df = pd.read_csv(tsv_path, sep="\t")
        date = os.path.basename(result_dir).split("_")
        date = date[0] + date[1]
        date = pd.to_datetime(date, format="%Y%m%d%H%M%S")
        value = df['error_mean'].values[0]
        date_list.append(date)
        value_list.append(value)

    percentile_90 = np.percentile(value_list, 90)
    plt.plot(date_list, value_list, marker=".")
    save_path = os.path.join(target_dir, "error_mean.png")
    plt.xlabel("date")
    plt.ylabel("error_mean")
    plt.xticks(rotation=90)
    plt.ylim(0, percentile_90 * 2)
    plt.savefig(save_path, bbox_inches='tight', pad_inches=0.05)
    print(f"save {save_path}")
