import argparse
import matplotlib.pyplot as plt
import pandas as pd
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("pose_tsv", type=Path)
    parser.add_argument("ndt_diagnostics_tsv", type=Path)
    parser.add_argument("target_column_name", type=str)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    pose_tsv = args.pose_tsv
    ndt_diagnostics_tsv = args.ndt_diagnostics_tsv
    target_column_name = args.target_column_name

    df_pose = pd.read_csv(pose_tsv, delimiter="\t")
    df_diag = pd.read_csv(ndt_diagnostics_tsv, delimiter="\t")
    df_diag = df_diag[["timestamp_header", target_column_name]]

    df_pose = pd.merge(df_pose, df_diag, left_on="timestamp", right_on="timestamp_header", how="left")

    color = df_pose[target_column_name].values

    plt.figure(figsize=(6.4 * 2, 4.8))
    plt.subplot(1, 2, 1)
    plt.plot(df_pose["timestamp"], color)
    plt.xlabel("timestamp")
    plt.ylabel(target_column_name)
    plt.grid()

    plt.subplot(1, 2, 2)
    plt.scatter(df_pose["position.x"], df_pose["position.y"], c=color)
    plt.colorbar()
    plt.axis("equal")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.grid()

    save_dir = ndt_diagnostics_tsv.parent
    save_path = save_dir / f"pose_{target_column_name}.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    plt.close()
    print(f"saved to {save_path}")
