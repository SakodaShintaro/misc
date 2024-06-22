"""自車の写り込みマスクを作成するスクリプト
基本的に、自車が写っているところは時系列的にずっと同じものが写るはず
つまりピクセルレベルで見たときに分散が小さい
"""

import argparse
from pathlib import Path
import cv2
import numpy as np
from copy import deepcopy
from tqdm import tqdm


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("target_dir", type=Path)
    parser.add_argument("--binary_threshold", type=int, default=17)
    parser.add_argument("--skip_num", type=int, default=5)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    target_dir = args.target_dir
    binary_threshold = args.binary_threshold
    skip_num = args.skip_num

    image_path_list = sorted(list(target_dir.glob("*.png")))
    image_path_list = image_path_list[::skip_num]

    # 平均を求める
    ave_images = None
    progress = tqdm(total=len(image_path_list))
    for i, image_path in enumerate(image_path_list):
        image = cv2.imread(str(image_path))
        if i == 0:
            ave_images = deepcopy(image)
            ave_images = ave_images.astype(np.float64)
        else:
            ave_images += image
        progress.update(1)
    ave_images /= len(image_path_list)

    # 分散を求める
    var_images = np.zeros_like(ave_images)
    progress = tqdm(total=len(image_path_list))
    for i, image_path in enumerate(image_path_list):
        image = cv2.imread(str(image_path))
        diff = image - ave_images
        diff_2 = np.power(diff, 2)
        var_images += diff_2
        progress.update(1)
    var_images /= len(image_path_list)
    print(var_images.shape)
    print(np.min(var_images), np.max(var_images))

    # 標準偏差にする
    stddev_images = np.sqrt(var_images)

    # RGBについて平均を取る
    stddev_images = np.mean(stddev_images, 2)

    # 2値化
    _, mask = cv2.threshold(stddev_images, binary_threshold, 255, cv2.THRESH_BINARY)

    # 除去する部分が255となるように反転
    mask = 255 - mask

    # モルフォロジー変換
    def make_kernel(size: int):
        return np.ones((size, size), np.uint8)
    # ノイズ除去(Opening)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, make_kernel(5))
    # 穴埋め(Closing)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, make_kernel(100))
    # 膨張処理
    mask = cv2.dilate(mask, make_kernel(5), iterations=1)

    # 残す部分が255となるように反転
    mask = 255 - mask

    save_path = (
        target_dir.parent / f"{target_dir.stem}_mask_thresh{binary_threshold}.png"
    )
    cv2.imwrite(str(save_path), mask)
    print(f"Saved to {save_path}")
