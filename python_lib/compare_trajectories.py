"""A script to compare two trajectories."""

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.font_manager import FontProperties
from scipy.spatial.transform import Rotation
from tqdm import tqdm

from calc_relative_pose import calc_relative_pose
from interpolate_pose import interpolate_pose


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("prediction_tsv", type=Path)
    parser.add_argument("ground_truth_tsv", type=Path)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    prediction_tsv = args.prediction_tsv
    ground_truth_tsv = args.ground_truth_tsv

    result_name = prediction_tsv.stem
    save_dir = prediction_tsv.parent / f"{result_name}_result"
    save_dir.mkdir(parents=True, exist_ok=True)

    df_pr = pd.read_csv(prediction_tsv, sep="\t")
    df_gt = pd.read_csv(ground_truth_tsv, sep="\t")

    # plot
    plt.plot(df_pr["position.x"], df_pr["position.y"], label="prediction")
    plt.plot(df_gt["position.x"], df_gt["position.y"], label="ground truth")
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
    df_pr = df_pr.sort_values(by="timestamp")
    df_gt = df_gt.sort_values(by="timestamp")

    # interpolate
    timestamp = df_pr["timestamp"]
    ok_mask = (timestamp > df_gt["timestamp"].min()) * (timestamp < df_gt["timestamp"].max())
    df_pr = df_pr[ok_mask]
    timestamp = timestamp[ok_mask]
    df_gt = interpolate_pose(df_gt, timestamp)

    # インデックスをリセット
    df_pr = df_pr.reset_index(drop=True)
    df_gt = df_gt.reset_index(drop=True)

    assert len(df_pr) == len(df_gt), f"len(df_pr)={len(df_pr)}, len(df_gt)={len(df_gt)}"

    # calc mean error
    diff_x = df_pr["position.x"].to_numpy() - df_gt["position.x"].to_numpy()
    diff_y = df_pr["position.y"].to_numpy() - df_gt["position.y"].to_numpy()
    diff_z = df_pr["position.z"].to_numpy() - df_gt["position.z"].to_numpy()
    diff_meter = (diff_x**2 + diff_y**2 + diff_z**2) ** 0.5

    # calc relative pose
    df_relative = calc_relative_pose(df_pr, df_gt)
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

    # plot (relative_pose of each frame)
    if not Path(f"{save_dir}/../image_timestamps.tsv").exists():
        sys.exit(0)
    df_image_timestamp = pd.read_csv(f"{save_dir}/../image_timestamps.tsv", sep="\t")
    df_image_timestamp["timestamp"] *= 1e-9
    (save_dir / "relative_pose_plot").mkdir(exist_ok=True)
    (save_dir / "combined_plot").mkdir(exist_ok=True)
    df_index = 0

    # 等幅化
    font = FontProperties(family="monospace")

    for i in tqdm(range(len(df_image_timestamp))):
        target_time = df_image_timestamp.iloc[i]["timestamp"]
        while df_index < len(df_relative):
            curr_time = df_relative.iloc[df_index]["timestamp"]
            if curr_time > target_time:
                break
            df_index += 1
        # GTから見たpredictionの相対位置を矢印で描画
        x = df_relative.iloc[df_index]["position.x"]
        y = df_relative.iloc[df_index]["position.y"]
        z = df_relative.iloc[df_index]["position.z"]
        angle_x = df_relative.iloc[df_index]["angle.x"]
        angle_y = df_relative.iloc[df_index]["angle.y"]
        angle_z = df_relative.iloc[df_index]["angle.z"]
        direction = np.array([5, 0, 0])
        plt.quiver(
            0,
            0,
            direction[0],
            direction[1],
            angles="xy",
            scale_units="xy",
            scale=1,
            color="blue",
            label="ground truth",
        )
        r = Rotation.from_euler("xyz", [angle_x, angle_y, angle_z], degrees=True)
        direction = r.apply(direction)
        plt.quiver(
            x,
            y,
            direction[0],
            direction[1],
            angles="xy",
            scale_units="xy",
            scale=1,
            color="red",
            label="prediction",
        )
        plt.axis("equal")
        plt.xlim(-3, 7)
        plt.ylim(-5, 5)
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        plt.legend()
        plt.text(-2, 3.0, f"error_x = {x * 100:+.1f} cm", fontsize=10, fontproperties=font)
        plt.text(-2, 2.5, f"error_y = {y * 100:+.1f} cm", fontsize=10, fontproperties=font)
        plt.text(-2, 2.0, f"error_z = {z * 100:+.1f} cm", fontsize=10, fontproperties=font)
        plt.text(-2, 1.5, f"roll    = {angle_x:+.2f} deg", fontsize=10, fontproperties=font)
        plt.text(-2, 1.0, f"pitch   = {angle_y:+.2f} deg", fontsize=10, fontproperties=font)
        plt.text(-2, 0.5, f"yaw     = {angle_z:+.2f} deg", fontsize=10, fontproperties=font)
        plt.savefig(
            f"{save_dir}/relative_pose_plot/{i:08d}.png",
            bbox_inches="tight",
            pad_inches=0.05,
        )
        plt.close()

        # グラフ上で今のdf_indexの位置に縦点線を入れる
        fig = plt.figure(figsize=(5, 5))

        # 上段：相対位置
        plt.subplot(2, 1, 1)
        plt.plot(df_relative["position.x"], label="x")
        plt.plot(df_relative["position.y"], label="y")
        plt.plot(df_relative["position.z"], label="z")
        plt.plot([df_index, df_index], [-1, 1], color="black", linestyle="dashed")
        plt.ylim((-1, 1))
        plt.legend()
        plt.xlabel("frame number")
        plt.ylabel("relative position [m]")

        # 下段：相対角度
        plt.subplot(2, 1, 2)
        plt.plot(df_relative["angle.x"], label="roll")
        plt.plot(df_relative["angle.y"], label="pitch")
        plt.plot(df_relative["angle.z"], label="yaw")
        plt.plot([df_index, df_index], [-1, 1], color="black", linestyle="dashed")
        plt.ylim((-1, 1))
        plt.legend()
        plt.xlabel("frame number")
        plt.ylabel("relative angle [degree]")

        plt.tight_layout()
        plt.savefig(
            f"{save_dir}/combined_plot/{i:08d}.png",
            bbox_inches="tight",
            pad_inches=0.05,
            dpi=150,
        )
        plt.close()
