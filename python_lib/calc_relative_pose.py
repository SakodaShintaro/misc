""" A script to compare two pose_lists.
"""
from scipy.spatial.transform import Rotation
import pandas as pd


def calc_relative_pose(df_pred: pd.DataFrame, df_true: pd.DataFrame) -> pd.DataFrame:
    """ df_trueから見たdf_predの相対位置・相対姿勢を計算する
    制約)
    * df_predとdf_trueは同じカラムを持つ
    * df_predとdf_trueは同じフレーム数を持つ
    """
    POSITIONS_KEY = ["position.x", "position.y", "position.z"]
    ORIENTATIONS_KEY = [
        "orientation.x",
        "orientation.y",
        "orientation.z",
        "orientation.w",
    ]
    assert len(df_pred) == len(df_true)
    assert (
        sorted(df_pred.columns) == sorted(df_true.columns)
    ), f"df_pred.columns: {sorted(df_pred.columns)}, df_true.columns: {sorted(df_true.columns)}"

    df_relative = df_pred.copy()
    df_relative[POSITIONS_KEY] = df_pred[POSITIONS_KEY].values - df_true[POSITIONS_KEY].values
    rotation_pred = Rotation.from_quat(df_pred[ORIENTATIONS_KEY].values)
    rotation_true = Rotation.from_quat(df_true[ORIENTATIONS_KEY].values)
    df_relative[ORIENTATIONS_KEY] = (rotation_pred * rotation_true.inv()).as_quat()

    # 各フレームの相対位置をrotation_true.inv()で回転させる
    # これにより、df_trueの姿勢を基準とした相対位置になる
    df_relative[POSITIONS_KEY] = (rotation_true.inv().apply(df_relative[POSITIONS_KEY].values))

    # roll, pitch, yawに変換したものも追加する
    r = Rotation.from_quat(
        df_relative[
            ["orientation.x", "orientation.y", "orientation.z", "orientation.w"]
        ]
    )
    euler = r.as_euler('xyz', degrees=True)
    df_relative['angle_x'] = euler[:, 0]
    df_relative['angle_y'] = euler[:, 1]
    df_relative['angle_z'] = euler[:, 2]

    return df_relative
