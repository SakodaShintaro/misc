"""A script to compare two pose_lists."""

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation


def calc_relative_pose(df_pred: pd.DataFrame, df_true: pd.DataFrame) -> pd.DataFrame:
    """df_trueから見たdf_predの相対位置・相対姿勢を計算する.

    制約)
    * df_predとdf_trueは必要なカラムを持つ
    * df_predとdf_trueは同じフレーム数を持つ
    """
    positions_key = ["position.x", "position.y", "position.z"]
    orientations_key = [
        "orientation.x",
        "orientation.y",
        "orientation.z",
        "orientation.w",
    ]
    assert len(df_pred) == len(df_true)

    df_relative = df_pred.copy()
    df_relative[positions_key] = (
        df_pred[positions_key].to_numpy() - df_true[positions_key].to_numpy()
    )
    rotation_pred = Rotation.from_quat(df_pred[orientations_key].values)
    rotation_true = Rotation.from_quat(df_true[orientations_key].values)
    df_relative[orientations_key] = (rotation_pred * rotation_true.inv()).as_quat()

    # 各フレームの相対位置をrotation_true.inv()で回転させる
    # これにより、df_trueの姿勢を基準とした相対位置になる
    df_relative[positions_key] = rotation_true.inv().apply(
        df_relative[positions_key].values,
    )

    # ノルムの追加
    df_relative["position.norm"] = np.linalg.norm(
        df_relative[positions_key].values,
        axis=1,
    )

    # roll, pitch, yawに変換したものも追加する
    r = Rotation.from_quat(
        df_relative[["orientation.x", "orientation.y", "orientation.z", "orientation.w"]],
    )
    euler = r.as_euler("xyz", degrees=True)
    df_relative["angle.x"] = euler[:, 0]
    df_relative["angle.y"] = euler[:, 1]
    df_relative["angle.z"] = euler[:, 2]
    df_relative["angle.norm"] = np.linalg.norm(r.as_rotvec(), axis=1)

    # 列の順番を整理
    return df_relative[
        [
            "timestamp",
            *positions_key,
            "position.norm",
            *orientations_key,
            "angle.x",
            "angle.y",
            "angle.z",
            "angle.norm",
        ]
    ]
