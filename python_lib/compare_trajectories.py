""" A script to compare two trajectories.
"""

import argparse
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from scipy.spatial.transform import Rotation
import os
from calc_relative_pose import calc_relative_pose
import numpy as np
from tqdm import tqdm
from interpolate_pose import interpolate_pose


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('prediction_tsv', type=str)
    parser.add_argument('ground_truth_tsv', type=str)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    prediction_tsv = args.prediction_tsv
    ground_truth_tsv = args.ground_truth_tsv

    result_name = os.path.basename(prediction_tsv)[:-4]
    save_dir = f"{os.path.dirname(prediction_tsv)}/{result_name}_result/"
    os.makedirs(save_dir, exist_ok=True)

    df_pr = pd.read_csv(prediction_tsv, sep='\t')
    df_gt = pd.read_csv(ground_truth_tsv, sep='\t')

    """
    print(df_pr.head())
       timestamp             x             y          z        qw        qx        qy        qz
    0  30.039234  81377.359375  49916.898438  41.228001  0.953786  0.000628 -0.006766  0.300410
    1  30.059261  81377.359375  49916.898438  41.228001  0.953786  0.000628 -0.006766  0.300410
    2  30.079253  81377.359375  49916.898438  41.228001  0.953786  0.000628 -0.006766  0.300410
    3  30.099268  81377.359375  49916.898438  41.228001  0.953786  0.000628 -0.006766  0.300410
    4  30.119261  81377.359375  49916.901949  41.235358  0.953769  0.000613 -0.007307  0.300452
    """

    # plot
    plt.plot(df_pr['x'], df_pr['y'], label='prediction')
    plt.plot(df_gt['x'], df_gt['y'], label='ground truth')
    plt.legend()
    plt.axis('equal')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    plt.savefig(f'{save_dir}/compare_trajectories.png',
                bbox_inches='tight', pad_inches=0.05, dpi=300)
    plt.close()

    # sort by timestamp
    df_pr = df_pr.sort_values(by='timestamp')
    df_gt = df_gt.sort_values(by='timestamp')

    # interpolate
    timestamp = df_pr['timestamp'].values
    ok_mask = (timestamp > df_gt['timestamp'].min()) * \
              (timestamp < df_gt['timestamp'].max())
    df_pr = df_pr[ok_mask]
    timestamp = timestamp[ok_mask]
    df_gt = interpolate_pose(df_gt, timestamp)

    # インデックスをリセット
    df_pr = df_pr.reset_index(drop=True)
    df_gt = df_gt.reset_index(drop=True)

    assert len(df_pr) == len(df_gt), \
        f"len(df_pr)={len(df_pr)}, len(df_gt)={len(df_gt)}"

    # calc differential
    plt.plot(df_pr['x'].diff(), label='x')
    plt.plot(df_pr['y'].diff(), label='y')
    plt.plot(df_pr['z'].diff(), label='z')
    plt.plot(df_pr['timestamp'].diff(), label='timestamp')
    plt.xlabel("Frame number")
    plt.ylabel("diff (x,y,z[m], timestamp[sec])")
    plt.legend()
    plt.savefig(f'{save_dir}/prediction_differential.png',
                bbox_inches='tight', pad_inches=0.05, dpi=300)
    plt.close()

    # calc mean error
    diff_x = df_pr['x'].values - df_gt['x'].values
    diff_y = df_pr['y'].values - df_gt['y'].values
    diff_z = df_pr['z'].values - df_gt['z'].values
    diff_meter = (diff_x**2 + diff_y**2 + diff_z**2)**0.5

    # calc relative pose
    df_relative = calc_relative_pose(df_pr, df_gt)
    df_relative.to_csv(f'{save_dir}/relative_pose.tsv', sep='\t', index=False)

    x_diff_mean = df_relative['x'].abs().mean()
    y_diff_mean = df_relative['y'].abs().mean()
    z_diff_mean = df_relative['z'].abs().mean()
    angle_x_diff_mean = df_relative["angle_x"].abs().mean()
    angle_y_diff_mean = df_relative["angle_y"].abs().mean()
    angle_z_diff_mean = df_relative["angle_z"].abs().mean()
    error = (df_relative['x']**2 +
             df_relative['y']**2 +
             df_relative['z']**2)**0.5
    df_summary = pd.DataFrame({
        'x_diff_mean': [x_diff_mean],
        'y_diff_mean': [y_diff_mean],
        'z_diff_mean': [z_diff_mean],
        'error_mean': [error.mean()],
        'roll_diff_mean': [angle_x_diff_mean],
        'pitch_diff_mean': [angle_y_diff_mean],
        'yaw_diff_mean': [angle_z_diff_mean],
    })
    df_summary.to_csv(
        f'{save_dir}/relative_pose_summary.tsv', sep='\t', index=False, float_format='%.4f')
    print(f'mean error: {error.mean():.3f} m')

    # plot (relative position)
    plt.plot(df_relative['x'], label='x')
    plt.plot(df_relative['y'], label='y')
    plt.plot(df_relative['z'], label='z')
    bottom, top = plt.ylim()
    plt.ylim(bottom=min(bottom, -1), top=max(top, 1))
    plt.legend()
    plt.xlabel('frame number')
    plt.ylabel('relative position [m]')
    plt.savefig(f'{save_dir}/relative_position.png',
                bbox_inches='tight', pad_inches=0.05, dpi=300)
    plt.close()

    # plot (relative angle)
    plt.plot(df_relative["angle_x"], label='roll')
    plt.plot(df_relative["angle_y"], label='pitch')
    plt.plot(df_relative["angle_z"], label='yaw')
    bottom, top = plt.ylim()
    plt.ylim(bottom=min(bottom, -1), top=max(top, 1))
    plt.legend()
    plt.xlabel('frame number')
    plt.ylabel('relative angle [degree]')
    plt.savefig(f'{save_dir}/relative_angle.png',
                bbox_inches='tight', pad_inches=0.05, dpi=300)
    plt.close()

    # plot (relative_pose of each frame)
    if not os.path.exists(f'{save_dir}/../image_timestamps.tsv'):
        exit(0)
    df_image_timestamp = pd.read_csv(
        f'{save_dir}/../image_timestamps.tsv', sep='\t')
    df_image_timestamp["timestamp"] *= 1e-9
    os.makedirs(f'{save_dir}/relative_pose_plot', exist_ok=True)
    os.makedirs(f'{save_dir}/combined_plot', exist_ok=True)
    df_index = 0

    # 等幅化
    font = FontProperties(family='monospace')

    for i in tqdm(range(len(df_image_timestamp))):
        target_time = df_image_timestamp.iloc[i]['timestamp']
        while df_index < len(df_relative):
            curr_time = df_relative.iloc[df_index]['timestamp']
            if curr_time > target_time:
                break
            df_index += 1
        # GTから見たpredictionの相対位置を矢印で描画
        x = df_relative.iloc[df_index]['x']
        y = df_relative.iloc[df_index]['y']
        z = df_relative.iloc[df_index]['z']
        angle_x = df_relative.iloc[df_index]['angle_x']
        angle_y = df_relative.iloc[df_index]['angle_y']
        angle_z = df_relative.iloc[df_index]['angle_z']
        direction = np.array([5, 0, 0])
        plt.quiver(0, 0, direction[0], direction[1], angles='xy',
                   scale_units='xy', scale=1, color='blue', label='ground truth')
        r = Rotation.from_euler(
            'xyz', [angle_x, angle_y, angle_z], degrees=True)
        direction = r.apply(direction)
        plt.quiver(x, y, direction[0], direction[1], angles='xy',
                   scale_units='xy', scale=1, color='red', label='prediction')
        plt.axis('equal')
        plt.xlim(-3, 7)
        plt.ylim(-5, 5)
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.legend()
        plt.text(-2, 3.0, f'error_x = {x * 100:+.1f} cm',
                 fontsize=10, fontproperties=font)
        plt.text(-2, 2.5, f'error_y = {y * 100:+.1f} cm',
                 fontsize=10, fontproperties=font)
        plt.text(-2, 2.0, f'error_z = {z * 100:+.1f} cm',
                 fontsize=10, fontproperties=font)
        plt.text(-2, 1.5, f'roll    = {angle_x:+.2f} deg',
                 fontsize=10, fontproperties=font)
        plt.text(-2, 1.0, f'pitch   = {angle_y:+.2f} deg',
                 fontsize=10, fontproperties=font)
        plt.text(-2, 0.5, f'yaw     = {angle_z:+.2f} deg',
                 fontsize=10, fontproperties=font)
        plt.savefig(f'{save_dir}/relative_pose_plot/{i:08d}.png',
                    bbox_inches='tight', pad_inches=0.05)
        plt.close()

        # グラフ上で今のdf_indexの位置に縦点線を入れる
        fig = plt.figure(figsize=(5, 5))

        # 上段: 相対位置
        plt.subplot(2, 1, 1)
        plt.plot(df_relative['x'], label='x')
        plt.plot(df_relative['y'], label='y')
        plt.plot(df_relative['z'], label='z')
        plt.plot([df_index, df_index], [-1, 1],
                 color='black', linestyle='dashed')
        plt.ylim((-1, 1))
        plt.legend()
        plt.xlabel('frame number')
        plt.ylabel('relative position [m]')

        # 下段: 相対角度
        plt.subplot(2, 1, 2)
        plt.plot(df_relative["angle_x"], label='roll')
        plt.plot(df_relative["angle_y"], label='pitch')
        plt.plot(df_relative["angle_z"], label='yaw')
        plt.plot([df_index, df_index], [-1, 1],
                 color='black', linestyle='dashed')
        plt.ylim((-1, 1))
        plt.legend()
        plt.xlabel('frame number')
        plt.ylabel('relative angle [degree]')

        plt.tight_layout()
        plt.savefig(f'{save_dir}/combined_plot/{i:08d}.png',
                    bbox_inches='tight', pad_inches=0.05, dpi=150)
        plt.close()
