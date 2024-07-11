import argparse
import open3d as o3d
import pathlib
import matplotlib.pyplot as plt
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("map_dir", type=pathlib.Path)
    parser.add_argument("--save_path", type=pathlib.Path)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    pcd_path = args.map_dir / "pointcloud_map.pcd"
    assert pcd_path.exists()
    save_path = args.save_path

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
    # min_bound = pcd.get_min_bound()
    # max_bound = pcd.get_max_bound()
    # print(min_bound, max_bound)
    # o3d.visualization.draw_geometries([pcd])

    # if visualize by matplotlib
    # downsample
    pcd = pcd.voxel_down_sample(voxel_size=4.0)
    points = np.asarray(pcd.points)
    plt.scatter(points[:, 0], points[:, 1], s=0.01)
    plt.axis("equal")
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05, dpi=300)
    print(f"Saved to {save_path.resolve()}")
