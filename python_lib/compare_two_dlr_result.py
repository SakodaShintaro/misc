"""2つのDriving Log Replayerの実行結果を比較するスクリプト."""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("dlr_result_dir1", type=Path)
    parser.add_argument("dlr_result_dir2", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--label1", type=str, default="result1")
    parser.add_argument("--label2", type=str, default="result2")
    parser.add_argument("--anonymous_print", action="store_true")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    dlr_result_dir1 = args.dlr_result_dir1
    dlr_result_dir2 = args.dlr_result_dir2
    output_dir = args.output_dir
    label1 = args.label1
    label2 = args.label2
    anonymous_print = args.anonymous_print

    dir_list1 = sorted(dlr_result_dir1.glob("*/"))
    dir_list2 = sorted(dlr_result_dir2.glob("*/"))

    # 共通するディレクトリ名を抽出
    set1 = {d.name for d in dir_list1}
    set2 = {d.name for d in dir_list2}
    common_dir_list = sorted(set1 & set2)
    # "LM_regression_"を含むディレクトリ名を抽出
    common_dir_list = [s for s in common_dir_list if "LM_regression_" in s]

    target_path = (
        "compare_trajectories/localization__kinematic_state_result/relative_pose_summary.tsv"
    )

    error_mean_list1 = []
    error_mean_list2 = []

    for dir_name in common_dir_list:
        tsv1 = dlr_result_dir1 / dir_name / target_path
        tsv2 = dlr_result_dir2 / dir_name / target_path
        df1 = pd.read_csv(tsv1, sep="\t")
        df2 = pd.read_csv(tsv2, sep="\t")
        error_mean1 = df1["error_mean"].mean()
        error_mean2 = df2["error_mean"].mean()
        print(f"{dir_name:50} {error_mean1:.3f} {error_mean2:.3f}")
        error_mean_list1.append(error_mean1)
        error_mean_list2.append(error_mean2)

    common_dir_list = [s.replace("LM_regression_", "") for s in common_dir_list]

    plt.plot(error_mean_list1, marker=".", label=label1)
    plt.plot(error_mean_list2, marker=".", label=label2)
    GUIDELINE = 0.5
    plt.plot([GUIDELINE] * len(common_dir_list), label=f"Guideline({GUIDELINE}m)", linestyle="--")
    plt.legend()
    plt.grid()
    if anonymous_print:
        plt.xticks(
            range(len(common_dir_list)),
            [f"location{i:2d}" for i in range(len(common_dir_list))],
            rotation=-90,
        )
    else:
        plt.xticks(range(len(common_dir_list)), common_dir_list, rotation=-90)
    plt.yscale("log")
    plt.ylabel("Error Mean [m]")
    save_path = output_dir / "compare_result.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    print(f"Save: {save_path}")
