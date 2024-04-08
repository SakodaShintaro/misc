import argparse
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("target_dir", type=Path)
    return parser.parse_args()


def calc_min_interval(value_list):
    set_value = set(value_list)
    tmp_list = list(set_value)
    tmp_list.sort()
    min_interval = min(
        [tmp_list[i] - tmp_list[i - 1] for i in range(1, len(tmp_list))]
    )
    return min_interval


if __name__ == "__main__":
    args = parse_args()
    target_dir = args.target_dir

    pcd_list = list(target_dir.glob("*.pcd"))
    pcd_list.sort()
    filename_list = []
    x_list = []
    y_list = []
    for pcd_path in pcd_list:
        filename_list.append(pcd_path.name)
        x, y = pcd_path.stem.split("_")[-2:]
        x = int(x)
        y = int(y)
        x_list.append(x)
        y_list.append(y)

    min_interval_x = calc_min_interval(x_list)
    min_interval_y = calc_min_interval(y_list)

    f = open(target_dir.parent / "pointcloud_map_metadata.yaml", "w")
    f.write(f"x_resolution: {min_interval_x}\n")
    f.write(f"y_resolution: {min_interval_y}\n")
    for filename, x, y in zip(filename_list, x_list, y_list):
        f.write(f"{filename}: [{x}, {y}]\n")
