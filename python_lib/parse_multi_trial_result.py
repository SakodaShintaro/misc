import argparse
from glob import glob
import pandas as pd
import matplotlib.pyplot as plt


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("output_dir", type=str)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    output_dir = args.output_dir

    THRESHOLD = 1.0

    result_dir_list = sorted(glob(f"{output_dir}/*/"))
    error_list = list()
    for result_dir in result_dir_list:
        ekf_result = f"{result_dir}/ekf_pose_result/relative_pose_summary.tsv"
        df = pd.read_csv(ekf_result, sep="\t")
        error_mean = df["error_mean"].values[0]
        error_list.append(error_mean)
        if error_mean > THRESHOLD:
            print(f"{result_dir}, error_mean = {error_mean:.2f}")

    ok = sum([(e <= THRESHOLD) for e in error_list])
    percent = ok / len(error_list) * 100
    print(f"{ok} / {len(error_list)} = {percent:5.1f}%")

    plt.hist(error_list, width=1)
    plt.xlabel("mean_error[m]")
    plt.ylabel("frequency")
    plt.savefig(f"{output_dir}/histogram.png",
                bbox_inches="tight", pad_inches=0.05)
