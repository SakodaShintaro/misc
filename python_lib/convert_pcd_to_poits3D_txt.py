import argparse
from pathlib import Path
import pandas as pd
import numpy as np
import open3d as o3d


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("pcd_path", type=Path)
    parser.add_argument("target_dir", type=Path)
    parser.add_argument("--center_point", nargs="+", type=float, default=[0.0, 0.0, 0.0])
    parser.add_argument("--crop_width", type=float, default=float("inf"))
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    pcd_path = args.pcd_path
    target_dir = args.target_dir
    center_point = args.center_point
    crop_width = args.crop_width

    if pcd_path.is_dir():
        pcd_combined = o3d.geometry.PointCloud()
        for file in pcd_path.glob("*.pcd"):
            pcd = o3d.io.read_point_cloud(str(file))
            pcd_combined += pcd
        pcd = pcd_combined
    else:
        pcd = o3d.io.read_point_cloud(str(pcd_path))

    print(center_point)
    print(f"{len(pcd.points)=}")
    points = np.asarray(pcd.points)
    mask = np.all(np.abs(points - center_point) <= crop_width / 2, axis=1)
    pcd = pcd.select_by_index(np.where(mask)[0])
    print(f"{len(pcd.points)=}")

    save_dir = target_dir / "colmap" / "sparse" / "0"
    save_dir.mkdir(exist_ok=True, parents=True)

    f = open(save_dir / "points3D.txt", "w")
    f.write("# 3D point list with one line of data per point:\n")
    f.write(
        "#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)\n"
    )

    for i, point in enumerate(np.asarray(pcd.points)):
        f.write(f"{i} {' '.join(map(str, point))} 0 0 0 0 0 0\n")
