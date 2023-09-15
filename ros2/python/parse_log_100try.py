""" A script to parse the autoware's log file.
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
import os
import pandas as pd


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
    score_list = []
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
        if contents[0] == "best_score":
            print(contents)
            score = float(contents[1])
            score_list.append(score)
        elif contents[0] == "align_using_monte_carlo_output":
            position = np.array([float(contents[1]), float(contents[2]), float(contents[3])])
            quaternion = np.array([float(contents[4]), float(contents[5]), float(contents[6]), float(contents[7])])
            rotation = R.from_quat(quaternion)
            position_list.append(position)
            rotation_list.append(rotation)
            diff_position = np.linalg.norm(position - GT_POSITION)
            diff_rotation = (rotation * GT_ROTATION.inv()).magnitude() * 180 / np.pi
            diff_position_list.append(diff_position)
            diff_rotation_list.append(diff_rotation)

    print(len(score_list), len(position_list), len(rotation_list))
    filename = os.path.basename(log_file)[0:-4]
    save_dir = os.path.dirname(log_file)

    # save_as df
    df = pd.DataFrame({
        "score": score_list,
        "diff_position": diff_position_list,
        "diff_rotation": diff_rotation_list
    })
    df.to_csv(f"{save_dir}/{filename}.tsv", sep="\t")

    # plot histogram score
    plt.hist(score_list)
    plt.xlabel("score")
    plt.ylabel("frequency")
    save_path = f"{save_dir}/{filename}_histogram.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    print(f"Saved to {save_path}")
    plt.close()

    # plot histogram diff_position
    plt.hist(diff_position_list)
    plt.xlabel("diff_position[m]")
    plt.ylabel("frequency")
    save_path = f"{save_dir}/{filename}_histogram_diff_position.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    print(f"Saved to {save_path}")
    plt.close()

    # plot histogram diff_rotation
    plt.hist(diff_rotation_list)
    plt.xlabel("diff_rotation[deg]")
    plt.ylabel("frequency")
    save_path = f"{save_dir}/{filename}_histogram_diff_rotation.png"
    plt.savefig(save_path, bbox_inches="tight", pad_inches=0.05)
    print(f"Saved to {save_path}")
    plt.close()
