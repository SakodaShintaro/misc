"""A script to plot a point cloud map in a pcd file."""

import argparse
import pathlib

import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
import pandas as pd


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("map_dir", type=pathlib.Path)
    parser.add_argument("--save_path", type=pathlib.Path, default=None)
    parser.add_argument("--pose_tsv", type=pathlib.Path, default=None)
    parser.add_argument("--by_open3d", action="store_true")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    map_dir = args.map_dir
    save_path = args.save_path
    pose_tsv = args.pose_tsv
    by_open3d = args.by_open3d

    pcd_path = args.map_dir / "pointcloud_map.pcd"
    assert pcd_path.exists()

    if save_path is None:
        save_path = map_dir / "pointcloud_map.png"

    pcd = None
    if pcd_path.is_file():
        pcd = o3d.io.read_point_cloud(str(pcd_path))
    else:
        # search for pcd files in the directory
        for p in pcd_path.iterdir():
            if p.suffix == ".pcd":
                if pcd is None:
                    pcd = o3d.io.read_point_cloud(str(p))
                else:
                    pcd += o3d.io.read_point_cloud(str(p))

    # if visualize by open3d
    if by_open3d:
        min_bound = pcd.get_min_bound()
        max_bound = pcd.get_max_bound()
        print(min_bound, max_bound)
        o3d.visualization.draw_geometries([pcd])

    # if visualize by matplotlib
    # downsample
    pcd = pcd.voxel_down_sample(voxel_size=4.0)
    points = np.asarray(pcd.points)
    plt.scatter(points[:, 0], points[:, 1], s=0.5, linewidths=0)

    if pose_tsv is not None:
        df = pd.read_csv(pose_tsv, delimiter="\t")
        plt.plot(df["position.x"], df["position.y"], color="orange")

    plt.axis("equal")
    plt.grid()
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05, dpi=300)
    print(f"Saved to {save_path.resolve()}")
