"""画像を左右に連結するスクリプト."""

import argparse
from pathlib import Path

import cv2
import numpy as np
from tqdm import tqdm


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("dir1", type=Path)
    parser.add_argument("dir2", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--text1", type=str, default="")
    parser.add_argument("--text2", type=str, default="")
    parser.add_argument("--ext", type=str, default="png")
    return parser.parse_args()


def put_text(
    image: np.ndarray,
    text: str,
    outline_color: tuple[int, int, int] = (0, 0, 0),
) -> np.ndarray:
    color = (128, 255, 128)
    face = cv2.FONT_HERSHEY_SIMPLEX
    text_pixel = image.shape[0] // 24
    scale = cv2.getFontScaleFromHeight(face, text_pixel)
    x = 0
    y = text_pixel
    cv2.putText(image, text, (x, y), face, scale, outline_color, 4, cv2.LINE_AA)
    cv2.putText(image, text, (x, y), face, scale, color, 1, cv2.LINE_AA)
    return image


def pad_images(image1: np.ndarray, image2: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    # 画像の高さが異なる場合、高さを揃える
    if image1.shape[0] != image2.shape[0]:
        # 差を計算
        diff = abs(image1.shape[0] - image2.shape[0])
        # 短い方の画像にパディングを追加
        if image1.shape[0] < image2.shape[0]:
            padding = np.zeros((diff, image1.shape[1], image1.shape[2]), dtype=image1.dtype)
            image1 = np.vstack((image1, padding))
        else:
            padding = np.zeros((diff, image2.shape[1], image2.shape[2]), dtype=image2.dtype)
            image2 = np.vstack((image2, padding))

    return image1, image2


def main() -> None:
    args = parse_args()
    dir1 = args.dir1
    dir2 = args.dir2
    output_dir = args.output_dir
    ext = args.ext

    output_dir.mkdir(parents=True, exist_ok=True)

    files1 = sorted(dir1.glob(f"*.{ext}"))
    files2 = sorted(dir2.glob(f"*.{ext}"))
    assert len(files1) == len(files2), f"{len(files1)} != {len(files2)}"
    progress = tqdm(total=len(files1))

    for file1, file2 in zip(files1, files2):
        progress.update(1)
        image1 = cv2.imread(str(dir1 / file1))
        image2 = cv2.imread(str(dir2 / file2))

        # 画像の高さを揃える
        image1, image2 = pad_images(image1, image2)

        # 文字を入れる
        put_text(image1, f"{args.text1}")
        put_text(image2, f"{args.text2}")

        # 連結
        new_image = cv2.hconcat([image1, image2])

        # 保存
        output_path = output_dir / file1.name
        cv2.imwrite(output_path, new_image)


if __name__ == "__main__":
    main()
