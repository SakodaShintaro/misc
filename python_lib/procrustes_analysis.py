"""GLIMの出力結果と、姿勢のtsvファイルを比較して、プロクラステス解析をした上で相対姿勢を求めるスクリプト"""

import argparse
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation
from interpolate_pose import interpolate_pose
from calc_relative_pose import calc_relative_pose


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("traj_lidar_txt", type=Path)
    parser.add_argument("pose_tsv", type=Path)
    return parser.parse_args()


def procrustes(df_from: pd.DataFrame, df_to: pd.DataFrame) -> pd.DataFrame:
    """
    Orthogonal procrustes analysis
    """
    assert len(df_from) == len(df_to)
    xyz_from = df_from[["position.x", "position.y", "position.z"]].values
    xyz_to = df_to[["position.x", "position.y", "position.z"]].values

    mean_from = xyz_from.mean(axis=0)
    mean_to = xyz_to.mean(axis=0)

    centered_xyz_from = xyz_from - mean_from
    centered_xyz_to = xyz_to - mean_to

    norm_from = np.linalg.norm(centered_xyz_from, axis=1).max()
    norm_to = np.linalg.norm(centered_xyz_to, axis=1).max()

    print(f"{norm_from=}, {norm_to=}")

    normalized_xyz_from = centered_xyz_from / norm_from
    normalized_xyz_to = centered_xyz_to / norm_to

    # SVDを計算
    u, _, vt = np.linalg.svd(normalized_xyz_to.T @ normalized_xyz_from)

    # 回転行列の計算 (3x3)
    R = u @ vt

    # 回転行列を適用して df_from を変換
    transformed_xyz_from = (centered_xyz_from @ R.T) * norm_to / norm_from + mean_to
    r = Rotation.from_quat(
        df_from[
            ["orientation.x", "orientation.y", "orientation.z", "orientation.w"]
        ].values
    )
    r = r * Rotation.from_matrix(R)
    transformed_quat_from = r.as_quat()

    transformed_df_from = df_from.copy()
    transformed_df_from[["position.x", "position.y", "position.z"]] = (
        transformed_xyz_from
    )
    transformed_df_from[
        ["orientation.x", "orientation.y", "orientation.z", "orientation.w"]
    ] = transformed_quat_from

    return transformed_df_from


if __name__ == "__main__":
    args = parse_args()
    traj_lidar_txt = args.traj_lidar_txt
    pose_tsv = args.pose_tsv

    df_glim = pd.read_csv(traj_lidar_txt, sep=" ", header=None)
    df_glim.columns = [
        "timestamp",
        "position.x",
        "position.y",
        "position.z",
        "orientation.x",
        "orientation.y",
        "orientation.z",
        "orientation.w",
    ]
    df_glim["timestamp"] = df_glim["timestamp"] * 1e9
    df_glim["timestamp"] = df_glim["timestamp"].astype(int)
    df_glim = df_glim.sort_values("timestamp")

    df_pose = pd.read_csv(pose_tsv, sep="\t")

    cond = (df_pose.iloc[0]["timestamp"] <= df_glim["timestamp"]) & (
        df_glim["timestamp"] <= df_pose.iloc[-1]["timestamp"]
    )
    df_glim = df_glim[cond]

    df_pose = interpolate_pose(df_pose, df_glim["timestamp"])

    df_glim["position.x"] += df_pose["position.x"].mean() - df_glim["position.x"].mean()
    df_glim["position.y"] += df_pose["position.y"].mean() - df_glim["position.y"].mean()
    df_glim["position.z"] += df_pose["position.z"].mean() - df_glim["position.z"].mean()

    assert len(df_glim) == len(df_pose), f"{len(df_glim)=}, {len(df_pose)=}"

    # プロクラステス解析
    df_glim_transformed = procrustes(df_glim, df_pose)

    plt.plot(
        df_glim_transformed["position.x"],
        df_glim_transformed["position.y"],
        label="glim",
    )
    plt.plot(df_pose["position.x"], df_pose["position.y"], label="pose")
    plt.axis("equal")
    plt.legend()
    save_dir = pose_tsv.parent / "compare_to_glim"
    save_dir.mkdir(exist_ok=True)
    save_path = save_dir / f"{pose_tsv.stem}_trajectory.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    plt.close()
    print(f"Saved to {save_path}")

    df_relative_pose = calc_relative_pose(df_pose, df_glim_transformed)
    df_relative_pose.to_csv(
        save_dir / "relative_pose_from_glim.tsv", sep="\t", index=False
    )

    mean_error = df_relative_pose["position.norm"].mean()
    print(f"mean_error={mean_error:.3f}")

    plt.subplot(2, 1, 1)
    plt.plot(df_relative_pose["timestamp"], df_relative_pose["position.x"], label="x")
    plt.plot(df_relative_pose["timestamp"], df_relative_pose["position.y"], label="y")
    plt.plot(df_relative_pose["timestamp"], df_relative_pose["position.z"], label="z")
    plt.grid()
    plt.legend()
    plt.title(f"mean_error = {mean_error:.3f} [m]")
    plt.ylabel("diff [m]")

    plt.subplot(2, 1, 2)
    plt.plot(df_relative_pose["timestamp"], df_relative_pose["angle.x"], label="x")
    plt.plot(df_relative_pose["timestamp"], df_relative_pose["angle.y"], label="y")
    plt.plot(df_relative_pose["timestamp"], df_relative_pose["angle.z"], label="z")
    plt.grid()
    plt.legend()
    plt.xlabel("timestamp [ns]")
    plt.ylabel("diff [deg]")

    save_path = save_dir / f"{pose_tsv.stem}_relative_pose.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    plt.close()
    print(f"Saved to {save_path}")

    save_path = save_dir / "glim_transformed.tsv"
    df_glim_transformed.to_csv(save_path, sep="\t", index=False)
    print(f"Saved to {save_path}")
