import pandas as pd
import matplotlib.pyplot as plt
import argparse
import os


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('log_file1')
    parser.add_argument('log_file2')
    return parser.parse_args()

if __name__ == "__main__":
    args = parse_args()
    log_file1 = args.log_file1
    log_file2 = args.log_file2

    df1 = pd.read_csv(log_file1, sep="\t", index_col=0)
    df2 = pd.read_csv(log_file2, sep="\t", index_col=0)

    print(df1.head())

    """
        score  diff_position  diff_rotation
    0  6.42650       0.038684       0.049227
    1  2.90563       0.521276       6.255484
    2  4.87581       0.362523       3.584214
    3  4.25184       0.523796       1.137508
    4  5.99468       0.185958       0.269413
    """

    save_dir = os.path.dirname(log_file2)

    # plot histogram score
    # ビンの幅を両者で共通化させる
    max_val = max(df1["score"].max(), df2["score"].max())
    min_val = min(df1["score"].min(), df2["score"].min())
    plt.hist(df1["score"], bins=50, range=(min_val, max_val),
            alpha=0.5, label="random")
    plt.hist(df2["score"], bins=50, range=(min_val, max_val),
            alpha=0.5, label="optuna")
    plt.xlabel("score")
    plt.ylabel("frequency")
    plt.legend()
    plt.savefig(f"{save_dir}/compare_histogram_score.png",
                bbox_inches="tight", pad_inches=0.05)
    print(f"Saved to {save_dir}/compare_histogram_score.png")
    plt.close()

    # plot histogram diff_position
    max_val = max(df1["diff_position"].max(), df2["diff_position"].max())
    min_val = min(df1["diff_position"].min(), df2["diff_position"].min())
    plt.hist(df1["diff_position"], bins=50, range=(min_val, max_val),
            alpha=0.5, label="random")
    plt.hist(df2["diff_position"], bins=50, range=(min_val, max_val),
            alpha=0.5, label="optuna")
    plt.xlabel("diff_position[m]")
    plt.ylabel("frequency")
    plt.legend()
    plt.savefig(f"{save_dir}/compare_histogram_diff_position.png",
                bbox_inches="tight", pad_inches=0.05)
    print(f"Saved to {save_dir}/compare_histogram_diff_position.png")
    plt.close()

    # plot histogram diff_rotation
    max_val = max(df1["diff_rotation"].max(), df2["diff_rotation"].max())
    min_val = min(df1["diff_rotation"].min(), df2["diff_rotation"].min())
    plt.hist(df1["diff_rotation"], bins=50, range=(min_val, max_val),
            alpha=0.5, label="random")
    plt.hist(df2["diff_rotation"], bins=50, range=(min_val, max_val),
            alpha=0.5, label="optuna")
    plt.xlabel("diff_rotation[deg]")
    plt.ylabel("frequency")
    plt.legend()
    plt.savefig(f"{save_dir}/compare_histogram_diff_rotation.png",
                bbox_inches="tight", pad_inches=0.05)
    print(f"Saved to {save_dir}/compare_histogram_diff_rotation.png")
    plt.close()
