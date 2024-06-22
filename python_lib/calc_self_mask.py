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
    parser.add_argument("--binary_threshold", type=int, default=14)
    parser.add_argument("--kernel_size", type=int, default=40)
    parser.add_argument("--skip_num", type=int, default=1)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    target_dir = args.target_dir
    binary_threshold = args.binary_threshold
    kernel_size = args.kernel_size
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

    # 除去する部分が1となるように反転
    mask = 255 - mask

    # 膨張処理
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=1)

    save_path = (
        target_dir.parent
        / f"{target_dir.stem}_mask_thresh{binary_threshold}_kernel{kernel_size}.png"
    )
    cv2.imwrite(str(save_path), mask)
    print(f"Saved to {save_path}")
