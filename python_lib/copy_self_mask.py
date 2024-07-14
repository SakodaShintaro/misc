"""自車の写り込みマスク1枚から元画像と同じファイル数としてコピーするスクリプト
"""

import argparse
from pathlib import Path
import cv2


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("mask_image", type=Path)
    parser.add_argument("target_dir", type=Path)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    mask_image = args.mask_image
    target_dir = args.target_dir

    image_path_list = sorted(list(target_dir.glob("*.png")))

    mask = cv2.imread(str(mask_image), cv2.IMREAD_GRAYSCALE)

    save_dir = target_dir.parent.parent / "masks" / target_dir.name
    save_dir.mkdir(exist_ok=True, parents=True)

    for image_path in image_path_list:
        save_path = save_dir / image_path.name
        cv2.imwrite(str(save_path), mask)
