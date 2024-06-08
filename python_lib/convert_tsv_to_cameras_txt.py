""" extract_images_from_rosbag.pyを実行してimagesディレクトリにtsvが展開されている状態から
【Before】
.
└── images
    ├── camera0
    ├── camera1
    ├── camera2
    ├── camera3
    ├── camera4
    ├── camera5
    ├── camera6
    ├── camera_info_camera0.tsv
    ├── camera_info_camera1.tsv
    ├── camera_info_camera2.tsv
    ├── camera_info_camera3.tsv
    ├── camera_info_camera4.tsv
    ├── camera_info_camera5.tsv
    ├── camera_info_camera6.tsv
    ├── timestamps_camera0.tsv
    ├── timestamps_camera1.tsv
    ├── timestamps_camera2.tsv
    ├── timestamps_camera3.tsv
    ├── timestamps_camera4.tsv
    ├── timestamps_camera5.tsv
    └── timestamps_camera6.tsv
【After】

"""

import argparse
from pathlib import Path
import pandas as pd
import numpy as np


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("target_dir", type=Path)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    target_dir = args.target_dir

    images_dir = target_dir / "images"
    camera_info_tsv_list = sorted(list(images_dir.glob("camera_info_*.tsv")))
    pose_tsv_list = sorted(list(images_dir.glob("pose_*.tsv")))

    save_dir = target_dir / "colmap" / "sparse" / "0"
    save_dir.mkdir(exist_ok=True, parents=True)

    f_cameras = open(save_dir / "cameras.txt", "w")
    f_images = open(save_dir / "images.txt", "w")

    # cameras.txt のヘッダーを書き込み
    f_cameras.write("# Camera list with one line of data per camera:\n")
    f_cameras.write("#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n")
    """
    実例
    1 OPENCV 1920 1280 2059.612011552946 2059.612011552946 952.4121898799498 634.5872082485005 0.03545287376426267 -0.33830085391165776 1.9229596190070855e-05 0.0007138551068011635
    2 OPENCV 1920 1280 2046.6346113297673 2046.6346113297673 975.056043638034 640.9087071643551 0.030496309445083532 -0.3101716556652957 0.002555391421798901 0.0008861465365293735
    3 OPENCV 1920 1280 2053.547383468203 2053.547383468203 944.359788133735 630.6490975690649 0.03332389289822763 -0.3018914499228165 8.591920960768094e-06 -0.0001887783629063975
    4 OPENCV 1920 886 2050.2525380004945 2050.2525380004945 970.3085586719832 248.13553724409144 0.03346595072144602 -0.33633543917951975 0.00022426185630236573 0.002455344038539979
    5 OPENCV 1920 886 2053.6156009850306 2053.6156009850306 970.5469368931234 235.61579209679962 0.026366806720632876 -0.28801395149313963 -2.2055500561490645e-05 -0.0008252566325695711
    """

    # images.txt のヘッダーを書き込み
    f_images.write("# Image list with two lines of data per image:\n")
    f_images.write("#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n")
    f_images.write("#   POINTS2D[] as (X, Y, POINT3D_ID)\n")
    """
    実例
    1 0.49035187959760007 0.6143544699974626 -0.4828056148692478 0.3860341404849641 210.90171913664233 6.242715219682963 -24.556937744799793 1 FRONT/1550083467346370.jpg

    2 0.49284278800061276 0.6163122340875992 -0.4798188326127646 0.38345678283198853 210.64868301581419 6.456540576056748 -27.275727234103062 1 FRONT/1550083467446163.jpg

    3 0.4945383461942329 0.6183584613456604 -0.47737590841634137 0.3810208386550179 210.4105965062657 6.739718593375017 -29.687256769654653 1 FRONT/1550083467545990.jpg

    """

    for i, camera_info_tsv in enumerate(camera_info_tsv_list):
        df = pd.read_csv(camera_info_tsv, sep="\t")
        row = df.iloc[0]
        frame_id = row["frame_id"]
        width = row["width"]
        height = row["height"]
        D_str = row["D"]
        K_str = row["K"]

        D = np.array(eval(D_str.replace("array('d', ", "").replace(")", "")))
        print(D)

        K = np.array(eval(K_str.replace(" ", ","))).reshape((3, 3))
        print(K)

        f_cameras.write(
            f"{i} OPENCV {width} {height} {K[0][0]} {K[1][1]} {K[0][2]} {K[1][2]} {D[0]} {D[1]} {D[2]} {D[3]} {D[4]}\n"
        )

    images_dir_list = sorted(list(images_dir.glob("camera*/")))
    image_id = 0
    for i, images_dir in enumerate(images_dir_list):
        if not images_dir.is_dir():
            continue
        df = pd.read_csv(pose_tsv_list[i], sep="\t")
        image_list = list(images_dir.glob("*.png"))
        values = df[
            [
                "orientation.w",
                "orientation.x",
                "orientation.y",
                "orientation.z",
                "position.x",
                "position.y",
                "position.z",
            ]
        ].values
        assert len(image_list) == len(values)
        qw = values[:, 0]
        qx = values[:, 1]
        qy = values[:, 2]
        qz = values[:, 3]
        x = values[:, 4]
        y = values[:, 5]
        z = values[:, 6]

        for j, image_path in enumerate(image_list):
            image_name = image_path.name
            f_images.write(
                f"{image_id:08d} {qw[j]} {qx[j]} {qy[j]} {qz[j]} {x[j]} {y[j]} {z[j]} {i} {images_dir.name}/{image_name}\n"
            )
            f_images.write(f"\n")
            image_id += 1
