import argparse
import open3d as o3d
import pathlib


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('pcd_path', type=pathlib.Path)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    pcd_path = args.pcd_path
    pcd = o3d.io.read_point_cloud(str(pcd_path))
    min_bound = pcd.get_min_bound()
    max_bound = pcd.get_max_bound()
    print(min_bound, max_bound)
    o3d.visualization.draw_geometries([pcd])
