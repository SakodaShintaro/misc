""" rosbagからGaussian Splattingで学習するための形式に変換する
* 各カメラのモデル・パラメータ
* 各カメラの画像
* LiDARの点群を仮でマッピングしたもの

./train_data_dir
├── colmap
│   └── sparse
│       └── 0
│           ├── cameras.txt
│           ├── images.txt
│           └── points3D.txt
├── images
│   ├── FRONT
│   ├── FRONT_LEFT
│   ├── FRONT_RIGHT
│   ├── SIDE_LEFT
│   └── SIDE_RIGHT
└── transform.json
"""

import argparse
import numpy as np
import cv2
from tf2_ros import Buffer
from scipy.spatial.transform import Rotation
from interpolate_pose import interpolate_pose
from parse_functions import parse_rosbag
from builtin_interfaces.msg import Time
from tqdm import tqdm
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("path_to_rosbag", type=Path)
    parser.add_argument("output_dir", type=Path)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    path_to_rosbag = args.path_to_rosbag
    output_dir = args.output_dir

    image_topic_list = ["/sensing/camera/traffic_light/image_raw"]
    camera_info_topic_list = ["/sensing/camera/traffic_light/camera_info"]
    assert len(image_topic_list) == len(camera_info_topic_list)

    lidar_topic_list = ["/sensing/lidar/top/pointcloud_raw_ex"]

    pose_topic_name = "/awsim/ground_truth/localization/kinematic_state"

    target_topics = image_topic_list + camera_info_topic_list + [
        pose_topic_name,
        "/tf_static",
    ]

    df_dict = parse_rosbag(str(path_to_rosbag), target_topics)

    # save rosbag info
    output_dir.mkdir(exist_ok=True)
    with open(f"{output_dir}/rosbag_info.txt", "w") as f:
        f.write(f"{path_to_rosbag}\n")

    # poseを準備
    df_pose = df_dict[pose_topic_name]

    # tf_bufferを準備
    df_tf_static = df_dict["/tf_static"]
    tf_buffer = Buffer()
    for _, row in df_tf_static.iterrows():
        for transform_stamped in row["transforms"]:
            tf_buffer.set_transform_static(transform_stamped, "default_authority")

    # 保存ディレクトリを準備
    colmap_dir = output_dir / "colmap/sparse/0"
    colmap_dir.mkdir(exist_ok=True)
    f_cameras = open(f"{str(colmap_dir/"cameras.txt")}", "w")
    f_images = open(f"{str(colmap_dir/"images.txt")}", "w")
    f_points3d = open(f"{str(colmap_dir/"points3D.txt")}", "w")
    images_dir = output_dir / "images"
    images_dir.mkdir(exist_ok=True)

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

    # transform.jsonのための情報
    transform_dict = {
        "sensor_params": {
            "camera_order": []
            # 加えて各カメラごとに
            # camera_name: {
            #     "type": "camera",
            #     "camera_model": "OPENCV",
            #     "camera_intrinsic": (3x3の配列),
            #     "camera_D": (長さ5の配列),
            #     "extrinsic": (4x4の配列 これがtf_staticに相当するbase_linkから見た固定の変換),
            #     "width": width(int),
            #     "height": height(int),
            # },
            # が入る
        },
        "frames": []
        # 一つのフレームは、例として
        # {
        #     "file_path": "images/FRONT/1550083467346370.jpg",
        #     "fl_x": 2059.612011552946,
        #     "fl_y": 2059.612011552946,
        #     "cx": 952.4121898799498,
        #     "cy": 634.5872082485005,
        #     "w": 1920,
        #     "h": 1280,
        #     "camera_model": "OPENCV",
        #     "camera": "FRONT",
        #     "timestamp": 1550083467.34637,
        #     "k1": 0.03545287376426267,
        #     "k2": -0.33830085391165776,
        #     "k3": 0.0,
        #     "k4": 0.0,
        #     "p1": 1.9229596190070855e-05,
        #     "p2": 0.0007138551068011635,
        #     "transform_matrix": [
        #         [
        #             -0.9718127080208842,
        #             0.05290754485165619,
        #             -0.22974083709015683,
        #             10546.449448489166
        #         ],
        #         [
        #             0.2357527612615248,
        #             0.2146424425182994,
        #             -0.9478128810199538,
        #             -1255.2664542820412
        #         ],
        #         [
        #             -0.0008343180943107114,
        #             -0.9752586393196159,
        #             -0.2210653531102714,
        #             -24.18073582749034
        #         ],
        #         [
        #             0.0,
        #             0.0,
        #             0.0,
        #             1.0
        #         ]
        #     ]
        # },
    }

    # 画像の保存
    for camera_id, (image_topic, camera_info_topic) in enumerate(zip(image_topic_list, camera_info_topic_list)):
        camera_name = "_".join(image_topic.split("/")[2:])

        # save camera_info
        df_camera_info = df_dict[camera_info_topic]
        camera_info = df_camera_info.iloc[0]
        D = np.array(camera_info["D"], dtype=np.float32)
        K = np.array(camera_info["K"], dtype=np.float32).reshape((3, 3))
        f_cameras.write(
            f"{camera_id} OPENCV {camera_info['width']} {camera_info['height']} {K[0, 0]} {K[1, 1]} {K[0, 2]} {K[1, 2]} {D[0]} {D[1]} {D[2]} {D[3]}\n"
        )

        # get pose
        image_timestamp_list = df_image["timestamp"].values
        image_list = df_image["image"].values
        min_pose_t = df_pose["timestamp"].min()
        max_pose_t = df_pose["timestamp"].max()
        ok_image_timestamp = (
            (min_pose_t < image_timestamp_list) * (image_timestamp_list < max_pose_t)
        )
        image_timestamp_list = image_timestamp_list[ok_image_timestamp]
        image_list = image_list[ok_image_timestamp]
        df_pose_curr = interpolate_pose(df_pose, image_timestamp_list)

        # save images
        df_image = df_dict[image_topic]
        curr_image_dir = images_dir / camera_name
        curr_image_dir.mkdir(exist_ok=True)
        bar = tqdm(total=len(image_list))
        for i, image in enumerate(image_list):
            save_path = curr_image_dir / f"{i:08d}.png"
            cv2.imwrite(save_path, image)
            quaternion = df_pose_curr.iloc[i][["qw", "qx", "qy", "qz"]].values
            translation = df_pose_curr.iloc[i][["x", "y", "z"]].values
            f_images.write(
                f"{i} {quaternion[0]} {quaternion[1]} {quaternion[2]} {quaternion[3]} {translation[0]} {translation[1]} {translation[2]} {camera_id} {camera_name / save_path.filename}\n"
            )
            R_b2m: np.ndarray = Rotation.from_quat(
            df_pose[
                ["orientation.x", "orientation.y", "orientation.z", "orientation.w"]
            ].values).as_matrix()
            t_b2m: np.ndarray = df_pose[["position.x", "position.y", "position.z"]].values

            transform_dict["frames"].append(
                {
                    "file_path": str(save_path),
                    "fl_x": K[0, 0],
                    "fl_y": K[1, 1],
                    "cx": K[0, 2],
                    "cy": K[1, 2],
                    "w": camera_info["width"],
                    "h": camera_info["height"],
                    "camera_model": "OPENCV",
                    "camera": camera_name,
                    "timestamp": image_timestamp_list[i],
                    "k1": D[0],
                    "k2": D[1],
                    "k3": D[2],
                    "k4": D[3],
                    "p1": D[4],
                    "p2": 0.0,
                    "transform_matrix": transform.transform_to_matrix(),
                }
            )
            bar.update(1)

        # static
        camera_frame = df_image["frame_id"].values[0]
        print(f"{camera_frame=}")
        transform = tf_buffer.lookup_transform(
            target_frame="base_link", source_frame=camera_frame, time=Time()
        )
