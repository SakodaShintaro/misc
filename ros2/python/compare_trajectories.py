""" A script to compare two trajectories.
"""

import argparse
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from scipy.spatial.transform import Rotation, Slerp
import os
from calc_relative_pose import calc_relative_pose
import numpy as np
from tqdm import tqdm


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('prediction_tsv', type=str)
    parser.add_argument('ground_truth_tsv', type=str)
    return parser.parse_args()


def interpolate_pose_in_time(df: pd.DataFrame, target_timestamp_list: pd.Series) -> pd.DataFrame:
    """ dfというデータフレームを、target_timestamp_listが示すタイムスタンプに合わせて補間する
    制約)
    * dfとtarget_timestamp_listはタイムスタンプでソートされていること
    * dfはtarget_timestamp_listを覆うような前後に広い区間のタイムスタンプを持つこと
    出力)
    * dfと同じカラムを持ち、長さがtarget_timestamp_listと同じであるデータフレーム
    """
    POSITIONS_KEY = ['x', 'y', 'z']
    ORIENTATIONS_KEY = ['qw', 'qx', 'qy', 'qz']
    result_df = pd.DataFrame(columns=df.columns)
    target_index = 0
    df_index = 0
    while df_index < len(df) - 1 and target_index < len(target_timestamp_list):
        curr_time = df.iloc[df_index]['timestamp']
        next_time = df.iloc[df_index + 1]['timestamp']
        target_time = target_timestamp_list[target_index]

        # target_timeを挟み込むようなdf_indexを探す
        if not (curr_time <= target_time <= next_time):
            df_index += 1
            continue

        curr_weight = (next_time - target_time) / (next_time - curr_time)
        next_weight = 1.0 - curr_weight

        curr_position = df.iloc[df_index][POSITIONS_KEY]
        next_position = df.iloc[df_index + 1][POSITIONS_KEY]
        target_position = curr_position * curr_weight + next_position * next_weight

        curr_orientation = df.iloc[df_index][ORIENTATIONS_KEY]
        next_orientation = df.iloc[df_index + 1][ORIENTATIONS_KEY]
        curr_r = Rotation.from_quat(curr_orientation)
        next_r = Rotation.from_quat(next_orientation)
        slerp = Slerp([curr_time, next_time],
                      Rotation.concatenate([curr_r, next_r]))
        target_orientation = slerp([target_time]).as_quat()[0]

        target_row = df.iloc[df_index].copy()
        target_row['timestamp'] = target_timestamp_list[target_index]
        target_row[POSITIONS_KEY] = target_position
        target_row[ORIENTATIONS_KEY] = target_orientation
        result_df = result_df.append(target_row)
        target_index += 1
    return result_df


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
        sec    nanosec            x             y          z        qx        qy        qz        qw
    0   46  250007580  81377.34375  49916.894531  41.205414  0.000178 -0.007339  0.300616  0.953717
    1   46  266674250  81377.34375  49916.894531  41.205414  0.000178 -0.007339  0.300616  0.953717
    2   46  283340919  81377.34375  49916.894531  41.205414  0.000178 -0.007339  0.300616  0.953717
    3   46  300007589  81377.34375  49916.894531  41.205414  0.000178 -0.007339  0.300616  0.953717
    4   46  316674258  81377.34375  49916.894531  41.205414  0.000178 -0.007339  0.300615  0.953717
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

    # calc timestamp
    df_pr['timestamp'] = df_pr['sec'] + df_pr['nanosec'] * 1e-9
    df_gt['timestamp'] = df_gt['sec'] + df_gt['nanosec'] * 1e-9
    df_pr = df_pr.drop(columns=['sec', 'nanosec'])
    df_gt = df_gt.drop(columns=['sec', 'nanosec'])

    # sort by timestamp
    df_pr = df_pr.sort_values(by='timestamp')
    df_gt = df_gt.sort_values(by='timestamp')

    # interpolate
    timestamp = df_pr['timestamp'].values
    ok_mask = (timestamp > df_gt['timestamp'].min()) * \
              (timestamp < df_gt['timestamp'].max())
    df_pr = df_pr[ok_mask]
    timestamp = timestamp[ok_mask]
    df_gt = interpolate_pose_in_time(df_gt, timestamp)

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
    df_summary = pd.DataFrame()
    df_summary = df_summary.append({
        'x_diff_mean': x_diff_mean,
        'y_diff_mean': y_diff_mean,
        'z_diff_mean': z_diff_mean,
        'error_mean': error.mean(),
        'roll_diff_mean': angle_x_diff_mean,
        'pitch_diff_mean': angle_y_diff_mean,
        'yaw_diff_mean': angle_z_diff_mean,
    }, ignore_index=True)
    df_summary.to_csv(
        f'{save_dir}/relative_pose_summary.tsv', sep='\t', index=False)
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
        x = df_relative.iloc[i]['x']
        y = df_relative.iloc[i]['y']
        z = df_relative.iloc[i]['z']
        angle_x = df_relative.iloc[i]['angle_x']
        angle_y = df_relative.iloc[i]['angle_y']
        angle_z = df_relative.iloc[i]['angle_z']
        direction = np.array([5, 0, 0])
        plt.quiver(0, 0, direction[0], direction[1], angles='xy',
                   scale_units='xy', scale=1, color='blue', label='ground truth')
        r = Rotation.from_euler('xyz', [angle_x, angle_y, angle_z], degrees=True)
        direction = r.apply(direction)
        plt.quiver(x, y, direction[0], direction[1], angles='xy',
                   scale_units='xy', scale=1, color='red', label='prediction')
        plt.axis('equal')
        plt.xlim(-3, 7)
        plt.ylim(-5, 5)
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.legend()
        plt.text(-2, 3.0, f'error_x = {x * 100:+.1f} cm', fontsize=10, fontproperties=font)
        plt.text(-2, 2.5, f'error_y = {y * 100:+.1f} cm', fontsize=10, fontproperties=font)
        plt.text(-2, 2.0, f'error_z = {z * 100:+.1f} cm', fontsize=10, fontproperties=font)
        plt.text(-2, 1.5, f'roll    = {angle_x:+.2f} deg', fontsize=10, fontproperties=font)
        plt.text(-2, 1.0, f'pitch   = {angle_y:+.2f} deg', fontsize=10, fontproperties=font)
        plt.text(-2, 0.5, f'yaw     = {angle_z:+.2f} deg', fontsize=10, fontproperties=font)
        plt.savefig(f'{save_dir}/relative_pose_plot/{i:08d}.png',
                    bbox_inches='tight', pad_inches=0.05)
        plt.close()
