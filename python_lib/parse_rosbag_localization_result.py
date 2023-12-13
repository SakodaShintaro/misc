import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import argparse
import matplotlib.pyplot as plt
import os
import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('rosbag_path', type=str)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    rosbag_path = args.rosbag_path

    serialization_format = 'cdr'
    storage_options = rosbag2_py.StorageOptions(
        uri=rosbag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {
        topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    target_topics = [
        '/localization/pose_estimator/pose',
        '/localization/pose_estimator/initial_to_result_relative_pose',
        '/localization/pose_estimator/transform_probability',
        '/localization/pose_estimator/nearest_voxel_transformation_likelihood',
        '/localization/pose_estimator/iteration_num',
        '/localization/pose_estimator/exe_time_ms',
        '/localization/pose_estimator/ndt_marker',
    ]

    storage_filter = rosbag2_py.StorageFilter(topics=target_topics)
    reader.set_filter(storage_filter)

    time_nvtl_list = []
    value_nvtl_list = []
    time_itr_list = []
    value_itr_list = []
    time_exe_list = []
    value_exe_list = []
    time_diff_position_list = []
    value_diff_position_list = []
    time_pose_array_list = []
    value_pose_array_list = []
    value_quat_array_list = []
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        if topic == '/localization/pose_estimator/nearest_voxel_transformation_likelihood':
            t_from_msg = msg.stamp.sec * 1e9 + msg.stamp.nanosec
            t_from_msg /= 1e9
            time_nvtl_list.append(t_from_msg)
            value_nvtl_list.append(msg.data)
        elif topic == '/localization/pose_estimator/iteration_num':
            t_from_msg = msg.stamp.sec * 1e9 + msg.stamp.nanosec
            t_from_msg /= 1e9
            time_itr_list.append(t_from_msg)
            value_itr_list.append(msg.data)
        elif topic == '/localization/pose_estimator/exe_time_ms':
            t_from_msg = msg.stamp.sec * 1e9 + msg.stamp.nanosec
            t_from_msg /= 1e9
            time_exe_list.append(t_from_msg)
            value_exe_list.append(msg.data)
        elif topic == '/localization/pose_estimator/initial_to_result_relative_pose':
            t_from_msg = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec
            t_from_msg /= 1e9
            time_diff_position_list.append(t_from_msg)
            value_diff_position_list.append(msg.pose.position.x**2 +
                                            msg.pose.position.y**2 +
                                            msg.pose.position.z**2)
        elif topic == '/localization/pose_estimator/ndt_marker':
            curr_pose_array = []
            curr_quat_array = []
            for marker_msg in msg.markers:
                t_from_msg = marker_msg.header.stamp.sec * 1e9 + marker_msg.header.stamp.nanosec
                t_from_msg /= 1e9
                pose = marker_msg.pose
                if pose.position.x == 0 and pose.position.y == 0 and pose.position.z == 0:
                    break
                curr_pose_array.append(
                    [pose.position.x, pose.position.y, pose.position.z])
                curr_quat_array.append(
                    Rotation.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]))
            time_pose_array_list.append(t_from_msg)
            value_pose_array_list.append(curr_pose_array)
            value_quat_array_list.append(curr_quat_array)

    # 時刻を0からの相対時刻に変換
    time_nvtl_list = [t - time_nvtl_list[0] for t in time_nvtl_list]
    time_itr_list = [t - time_itr_list[0] for t in time_itr_list]
    time_exe_list = [t - time_exe_list[0] for t in time_exe_list]
    time_diff_position_list = [
        t - time_diff_position_list[0] for t in time_diff_position_list]

    save_dir = os.path.dirname(rosbag_path) if os.path.isfile(
        rosbag_path) else rosbag_path
    save_name = "localization_result"

    # plotの縦サイズを大きくする
    plt.rcParams['figure.figsize'] = 9, 12

    # plot
    plt.subplot(4, 1, 1)
    plt.plot(time_itr_list, value_itr_list)
    plt.xlabel('time [s]')
    plt.ylabel('iteration_num')
    plt.ylim(bottom=0)
    plt.grid()

    plt.subplot(4, 1, 2)
    plt.plot(time_exe_list, value_exe_list)
    plt.xlabel('time [s]')
    plt.ylabel('exe_time [ms]')
    plt.ylim(bottom=0)
    plt.grid()

    plt.subplot(4, 1, 3)
    plt.plot(time_nvtl_list, value_nvtl_list)
    plt.xlabel('time [s]')
    plt.ylabel('NVTL')
    plt.ylim(bottom=0)
    plt.grid()

    plt.subplot(4, 1, 4)
    plt.plot(time_diff_position_list, value_diff_position_list)
    plt.xlabel('time [s]')
    plt.ylabel('diff_position [m]')
    plt.ylim(bottom=0)
    plt.grid()

    plt.tight_layout()
    plt.savefig(f'{save_dir}/{save_name}.png')
    plt.close()
    print(f'saved {save_dir}/{save_name}.png')

    # pose_arrayを気合で可視化
    plt.rcParams['figure.figsize'] = 9, 9
    pose_array_result_dir = f'{save_dir}/{save_name}_pose_array'
    yaw_array_result_dir = f'{save_dir}/{save_name}_yaw_array'
    os.makedirs(pose_array_result_dir, exist_ok=True)
    os.makedirs(yaw_array_result_dir, exist_ok=True)
    for i, (time, pose_array) in enumerate(zip(time_pose_array_list, value_pose_array_list)):
        if len(pose_array) < 30:
            continue
        pose_array = np.array(pose_array)
        pose_array -= pose_array[0]
        plt.figure()
        # indexで色付け(0 ~ 29), 0から1, 1から2,... へと矢印を描画
        for j in range(len(pose_array) - 1):
            diff_x = pose_array[j + 1][0] - pose_array[j][0]
            diff_y = pose_array[j + 1][1] - pose_array[j][1]
            plt.arrow(pose_array[j][0], pose_array[j][1],
                      diff_x, diff_y, color=plt.cm.jet(j / (len(pose_array) - 1)), length_includes_head=True)
        plt.title(f"iteration: {len(pose_array) - 1}")
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.axis('equal')
        lim = 0.2
        plt.xlim(-lim, lim)
        plt.ylim(-lim, lim)
        plt.grid()
        plt.savefig(f'{pose_array_result_dir}/{i:08d}.png')
        plt.close()

        # quaternionを気合で可視化
        plt.figure()
        r_list = []
        p_list = []
        y_list = []
        first_r = value_quat_array_list[i][0]
        for j in range(len(pose_array)):
            curr_r = value_quat_array_list[i][j]
            curr_r = curr_r * first_r.inv()
            r, p, y = curr_r.as_euler('xyz')
            r_list.append(r * 180 / np.pi)
            p_list.append(p * 180 / np.pi)
            y_list.append(y * 180 / np.pi)
        r_list = np.array(r_list)
        r_list -= r_list[0]
        p_list = np.array(p_list)
        p_list -= p_list[0]
        y_list = np.array(y_list)
        y_list -= y_list[0]
        plt.plot(r_list, label="roll")
        plt.plot(p_list, label="pitch")
        plt.plot(y_list, label="yaw")
        plt.xlabel('iteration')
        plt.ylabel('yaw [deg]')
        plt.ylim(-1, 1)
        plt.grid()
        plt.legend()
        plt.savefig(f'{yaw_array_result_dir}/{i:08d}_yaw.png')
        plt.close()
