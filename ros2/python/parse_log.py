""" A script to parse the autoware's log file.
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import os


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('log_file')
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    log_file = args.log_file

    GT_POSITION = np.array([81377.34375, 49916.89453125, 41.205413818359375])
    GT_QUATERNION = np.array(
        [0.00017802136426325887, -0.007339125499129295, 0.3006163239479065, 0.9537169337272644])
    GT_ROTATION = R.from_quat(GT_QUATERNION)

    f = open(log_file, 'r')
    lines = f.readlines()
    f.close()
    tp_list = []
    nvtl_list = []
    position_list = []
    rotation_list = []
    diff_position_list = []
    diff_rotation_list = []
    for line in lines:
        line = line.strip()
        elements = line.split(' ')
        if len(elements) < 5:
            continue
        if elements[4] != "[localization.pose_estimator.ndt_scan_matcher]:":
            continue
        contents = elements[5].split(',')
        if len(contents) < 10:
            continue
        try:
            index = int(contents[0])
        except:
            continue
        print(contents)
        position = np.array([float(contents[1]), float(contents[2]), float(contents[3])])
        quaternion = np.array([float(contents[4]), float(contents[5]), float(contents[6]), float(contents[7])])
        rotation = R.from_quat(quaternion)
        diff_position = np.linalg.norm(position - GT_POSITION)
        diff_rotation = (rotation * GT_ROTATION.inv()).magnitude()
        position_list.append(position)
        rotation_list.append(rotation)
        diff_position_list.append(diff_position)
        diff_rotation_list.append(diff_rotation)
        tp = float(contents[11])
        nvtl = float(contents[12])
        tp_list.append(tp)
        nvtl_list.append(nvtl)

    # 全リストをdiff_positionでソート
    key = tp_list
    position_list = [x for _, x in sorted(zip(key, position_list))]
    rotation_list = [x for _, x in sorted(zip(key, rotation_list))]
    nvtl_list = [x for _, x in sorted(zip(key, nvtl_list))]
    diff_rotation_list = [x for _, x in sorted(zip(key, diff_rotation_list))]
    diff_position_list = [x for _, x in sorted(zip(key, diff_position_list))]
    tp_list = [x for _, x in sorted(zip(key, tp_list))]

    filename = os.path.basename(log_file)[0:-4]

    plt.scatter(tp_list, nvtl_list)
    plt.xlabel("TP")
    plt.ylabel("NVTL")
    save_dir = os.path.dirname(log_file)
    save_path = f"{save_dir}/{filename}_tp_nvtl.png"
    plt.savefig(save_path, bbox_inches='tight', pad_inches=0.05)
    print(f"Saved to {save_path}")
    plt.close()

    position_list = np.array(position_list)
    rotation_list = np.array(rotation_list)
    diff_position_list = np.array(diff_position_list)
    diff_rotation_list = np.array(diff_rotation_list)
    plt.scatter(tp_list, diff_position_list)
    plt.plot([6.0, 6.0], [0.0, 5.0], color='red', linestyle='dashed')
    plt.xlabel("TP")
    plt.ylabel("Diff Position to GT[m]")
    save_path = f"{save_dir}/{filename}_tp_diff_position.png"
    plt.savefig(save_path, bbox_inches='tight', pad_inches=0.05)
    print(f"Saved to {save_path}")
    plt.close()

    plt.scatter(tp_list, diff_rotation_list)
    plt.plot([6.0, 6.0], [0.0, 5.0], color='red', linestyle='dashed')
    plt.xlabel("TP")
    plt.ylabel("Diff Rotation to GT[rad]")
    save_path = f"{save_dir}/{filename}_tp_diff_rotation.png"
    plt.savefig(save_path, bbox_inches='tight', pad_inches=0.05)
    print(f"Saved to {save_path}")
    plt.close()

    plt.scatter(nvtl_list, diff_position_list)
    plt.plot([3.0, 3.0], [0.0, 5.0], color='red', linestyle='dashed')
    plt.xlabel("NVTL")
    plt.ylabel("Diff Position to GT[m]")
    save_path = f"{save_dir}/{filename}_nvtl_diff_position.png"
    plt.savefig(save_path, bbox_inches='tight', pad_inches=0.05)
    print(f"Saved to {save_path}")
    plt.close()

    plt.scatter(nvtl_list, diff_rotation_list)
    plt.plot([3.0, 3.0], [0.0, 5.0], color='red', linestyle='dashed')
    plt.xlabel("NVTL")
    plt.ylabel("Diff Rotation to GT[rad]")
    save_path = f"{save_dir}/{filename}_nvtl_diff_rotation.png"
    plt.savefig(save_path, bbox_inches='tight', pad_inches=0.05)
    print(f"Saved to {save_path}")
    plt.close()

    # plot as arrow
    cmap = plt.get_cmap("plasma")
    scale = 0.2
    width = 0.05
    head_width = 0.2
    for i in range(len(position_list)):
        position = position_list[i]
        rotation = rotation_list[i]
        tp = tp_list[i]
        # tpでグラデーションをかけて色付け
        color = cmap(tp / 6.0)
        euler = rotation.as_euler('ZYX')
        plt.arrow(position[0], position[1], np.cos(euler[0]) * scale, np.sin(euler[0]) * scale, color=color, width=width, head_width=head_width)
    # color bar
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(vmin=0, vmax=6))
    plt.colorbar(sm)
    # plot gt
    gt_euler = GT_ROTATION.as_euler('ZYX')
    plt.arrow(GT_POSITION[0], GT_POSITION[1], np.cos(gt_euler[0]) * scale, np.sin(gt_euler[0]) * scale, color='red', width=width, head_width=head_width)
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.axis('equal')
    save_path = f"{save_dir}/{filename}_tp-arrow.png"
    plt.savefig(save_path, bbox_inches='tight', pad_inches=0.05)
    print(f"Saved to {save_path}")
    plt.close()

    # plot as arrow
    cmap = plt.get_cmap("plasma")
    scale = 0.2
    width = 0.05
    head_width = 0.2
    for i in range(len(position_list)):
        position = position_list[i]
        rotation = rotation_list[i]
        nvtl = nvtl_list[i]
        # tpでグラデーションをかけて色付け
        color = cmap(nvtl / 3.0)
        euler = rotation.as_euler('ZYX')
        plt.arrow(position[0], position[1], np.cos(euler[0]) * scale, np.sin(euler[0]) * scale, color=color, width=width, head_width=head_width)
    # color bar
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(vmin=0, vmax=3))
    plt.colorbar(sm)
    # plot gt
    gt_euler = GT_ROTATION.as_euler('ZYX')
    plt.arrow(GT_POSITION[0], GT_POSITION[1], np.cos(gt_euler[0]) * scale, np.sin(gt_euler[0]) * scale, color='red', width=width, head_width=head_width)
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.axis('equal')
    save_path = f"{save_dir}/{filename}_nvtl-arrow.png"
    plt.savefig(save_path, bbox_inches='tight', pad_inches=0.05)
    print(f"Saved to {save_path}")
    plt.close()
