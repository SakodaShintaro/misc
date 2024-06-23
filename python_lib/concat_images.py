import cv2
import os
import argparse
from glob import glob
import numpy as np
from tqdm import tqdm


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('dir1', type=str)
    parser.add_argument('dir2', type=str)
    parser.add_argument('output_dir', type=str)
    parser.add_argument('--text1', type=str, default="")
    parser.add_argument('--text2', type=str, default="")
    parser.add_argument("--ext", type=str, default="png")
    return parser.parse_args()


def put_text(image, text, outline_color=(0, 0, 0)):
    color = (128, 255, 128)
    face = cv2.FONT_HERSHEY_SIMPLEX
    text_pixel = image.shape[0] // 24
    scale = cv2.getFontScaleFromHeight(face, text_pixel)
    x = 0
    y = text_pixel
    cv2.putText(image, text, (x, y), face, scale, outline_color, 5, cv2.LINE_AA)
    cv2.putText(image, text, (x, y), face, scale, color, 2, cv2.LINE_AA)
    return image


def pad_images(image1, image2):
    # 画像の高さが異なる場合、高さを揃える
    if image1.shape[0] != image2.shape[0]:
        # 差を計算
        diff = abs(image1.shape[0] - image2.shape[0])
        # 短い方の画像にパディングを追加
        if image1.shape[0] < image2.shape[0]:
            padding = np.zeros(
                (diff, image1.shape[1], image1.shape[2]), dtype=image1.dtype)
            image1 = np.vstack((image1, padding))
        else:
            padding = np.zeros(
                (diff, image2.shape[1], image2.shape[2]), dtype=image2.dtype)
            image2 = np.vstack((image2, padding))

    return image1, image2


def main():
    args = parse_args()
    dir1 = args.dir1
    dir2 = args.dir2
    output_dir = args.output_dir
    ext = args.ext

    os.makedirs(output_dir, exist_ok=True)

    files1 = sorted(glob(f"{dir1}/*.{ext}"))
    files2 = sorted(glob(f"{dir2}/*.{ext}"))
    assert len(files1) == len(files1)
    progress = tqdm(total=len(files1))

    for file1, file2 in zip(files1, files2):
        progress.update(1)
        image1 = cv2.imread(os.path.join(dir1, file1))
        image2 = cv2.imread(os.path.join(dir2, file2))

        # 画像の高さを揃える
        image1, image2 = pad_images(image1, image2)

        # 文字を入れる
        filename1 = os.path.basename(file1)
        filename2 = os.path.basename(file2)
        put_text(image1, f"{args.text1}_{filename1}")
        put_text(image2, f"{args.text2}_{filename2}")

        # 連結
        new_image = cv2.hconcat([image1, image2])

        # 保存
        output_path = f"{output_dir}/{os.path.basename(file1)}"
        cv2.imwrite(output_path, new_image)


if __name__ == "__main__":
    main()
