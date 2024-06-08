import argparse
from pathlib import Path
import pandas as pd
import numpy as np
import open3d as o3d


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("pcd_path", type=Path)
    parser.add_argument("target_dir", type=Path)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    pcd_path = args.pcd_path
    target_dir = args.target_dir

    if pcd_path.is_dir():
        pcd_combined = o3d.geometry.PointCloud()
        for file in pcd_path.glob("*.pcd"):
            pcd = o3d.io.read_point_cloud(str(file))
            pcd_combined += pcd
        pcd = pcd_combined
    else:
        pcd = o3d.io.read_point_cloud(str(pcd_path))

    save_dir = target_dir / "colmap" / "sparse" / "0"
    save_dir.mkdir(exist_ok=True, parents=True)

    f = open(save_dir / "points3D.txt", "w")
    f.write("# 3D point list with one line of data per point:\n")
    f.write(
        "#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)\n"
    )

    for i, point in enumerate(np.asarray(pcd.points)):
        f.write(f"{i} {' '.join(map(str, point))} 0 0 0 0 0 0\n")
