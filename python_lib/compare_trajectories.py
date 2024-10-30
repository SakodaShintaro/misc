"""A script to compare two trajectories."""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

from calc_relative_pose import calc_relative_pose
from interpolate_pose import interpolate_pose


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("prediction_tsv", type=Path)
    parser.add_argument("reference_tsv", type=Path)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    prediction_tsv = args.prediction_tsv
    reference_tsv = args.reference_tsv

    result_name = prediction_tsv.stem
    save_dir = prediction_tsv.parent / f"{result_name}_result"
    save_dir.mkdir(parents=True, exist_ok=True)

    df_prd = pd.read_csv(prediction_tsv, sep="\t")
    df_ref = pd.read_csv(reference_tsv, sep="\t")

    # plot
    plt.plot(df_prd["position.x"], df_prd["position.y"], label="prediction")
    plt.plot(df_ref["position.x"], df_ref["position.y"], label="reference")
    plt.legend()
    plt.axis("equal")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.savefig(
        f"{save_dir}/compare_trajectories.png",
        bbox_inches="tight",
        pad_inches=0.05,
        dpi=300,
    )
    plt.close()

    # sort by timestamp
    df_prd = df_prd.sort_values(by="timestamp")
    df_ref = df_ref.sort_values(by="timestamp")

    # interpolate
    timestamp = df_prd["timestamp"]
    ok_mask = (timestamp > df_ref["timestamp"].min()) * (timestamp < df_ref["timestamp"].max())
    df_prd = df_prd[ok_mask]
    timestamp = timestamp[ok_mask]
    df_ref = interpolate_pose(df_ref, timestamp)

    # reset index
    df_prd = df_prd.reset_index(drop=True)
    df_ref = df_ref.reset_index(drop=True)

    assert len(df_prd) == len(df_ref), f"len(df_pr)={len(df_prd)}, len(df_gt)={len(df_ref)}"

    # calc mean error
    diff_x = df_prd["position.x"].to_numpy() - df_ref["position.x"].to_numpy()
    diff_y = df_prd["position.y"].to_numpy() - df_ref["position.y"].to_numpy()
    diff_z = df_prd["position.z"].to_numpy() - df_ref["position.z"].to_numpy()
    diff_meter = (diff_x**2 + diff_y**2 + diff_z**2) ** 0.5

    # calc relative pose
    df_relative = calc_relative_pose(df_prd, df_ref)
    df_relative.to_csv(f"{save_dir}/relative_pose.tsv", sep="\t", index=False)

    x_diff_mean = df_relative["position.x"].abs().mean()
    y_diff_mean = df_relative["position.y"].abs().mean()
    z_diff_mean = df_relative["position.z"].abs().mean()
    angle_x_diff_mean = df_relative["angle.x"].abs().mean()
    angle_y_diff_mean = df_relative["angle.y"].abs().mean()
    angle_z_diff_mean = df_relative["angle.z"].abs().mean()
    error_norm = df_relative["position.norm"]
    df_summary = pd.DataFrame(
        {
            "x_diff_mean": [x_diff_mean],
            "y_diff_mean": [y_diff_mean],
            "z_diff_mean": [z_diff_mean],
            "error_mean": [error_norm.mean()],
            "roll_diff_mean": [angle_x_diff_mean],
            "pitch_diff_mean": [angle_y_diff_mean],
            "yaw_diff_mean": [angle_z_diff_mean],
        },
    )
    df_summary.to_csv(
        f"{save_dir}/relative_pose_summary.tsv",
        sep="\t",
        index=False,
        float_format="%.4f",
    )
    print(f"mean error: {error_norm.mean():.3f} m")

    # plot (relative position)
    plt.subplot(2, 1, 1)
    plt.plot(df_relative["position.x"], label="x")
    plt.plot(df_relative["position.y"], label="y")
    plt.plot(df_relative["position.z"], label="z")
    GUIDELINE_POSITION = 0.5
    plt.plot(
        [0, len(df_relative)],
        [GUIDELINE_POSITION, GUIDELINE_POSITION],
        linestyle="dashed",
        color="red",
        label=f"guideline = {GUIDELINE_POSITION}m",
    )
    plt.plot(
        [0, len(df_relative)],
        [-GUIDELINE_POSITION, -GUIDELINE_POSITION],
        linestyle="dashed",
        color="red",
    )
    bottom, top = plt.ylim()
    plt.ylim(bottom=min(bottom, -1), top=max(top, 1))
    plt.legend(loc="upper left", bbox_to_anchor=(1, 1))
    plt.xlabel("frame number")
    plt.ylabel("relative position [m]")
    plt.grid()

    # plot (relative angle)
    plt.subplot(2, 1, 2)
    plt.plot(df_relative["angle.x"], label="roll")
    plt.plot(df_relative["angle.y"], label="pitch")
    plt.plot(df_relative["angle.z"], label="yaw")
    GUIDELINE_ANGLE = 0.5
    plt.plot(
        [0, len(df_relative)],
        [GUIDELINE_ANGLE, GUIDELINE_ANGLE],
        linestyle="dashed",
        color="red",
        label=f"guideline = {GUIDELINE_ANGLE}deg",
    )
    plt.plot(
        [0, len(df_relative)],
        [-GUIDELINE_ANGLE, -GUIDELINE_ANGLE],
        linestyle="dashed",
        color="red",
    )
    bottom, top = plt.ylim()
    plt.ylim(bottom=min(bottom, -1), top=max(top, 1))
    plt.legend(loc="upper left", bbox_to_anchor=(1, 1))
    plt.xlabel("frame number")
    plt.ylabel("relative angle [degree]")
    plt.grid()

    plt.tight_layout()
    plt.savefig(
        f"{save_dir}/relative_pose.png",
        bbox_inches="tight",
        pad_inches=0.05,
        dpi=300,
    )
    print(f"saved to {save_dir}/relative_pose.png")
    plt.close()
