import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import argparse
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy.spatial.transform import Rotation


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('rosbag_path', type=str)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    rosbag_path = args.rosbag_path
    target_topic = "/localization/pose_estimator/initial_to_result_relative_pose"

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

    storage_filter = rosbag2_py.StorageFilter(topics=[target_topic])
    reader.set_filter(storage_filter)

    time_list = list()
    position_x_list = list()
    position_y_list = list()
    position_z_list = list()
    angle_x_list = list()
    angle_y_list = list()
    angle_z_list = list()
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        time_list.append(t)
        position_x_list.append(msg.pose.position.x)
        position_y_list.append(msg.pose.position.y)
        position_z_list.append(msg.pose.position.z)
        quat = msg.pose.orientation
        r = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])
        angle_x, angle_y, angle_z = r.as_euler('xyz', degrees=True)
        angle_x_list.append(angle_x)
        angle_y_list.append(angle_y)
        angle_z_list.append(angle_z)

    time_list = np.array(time_list, dtype=np.float64)
    time_list /= 1e9
    time_list -= time_list[0]

    save_name = '__'.join(target_topic.split('/')[1:])

    plt.figure(figsize=(10, 6))

    plt.subplot(2, 1, 1)
    plt.plot(time_list, position_x_list, label='x')
    plt.plot(time_list, position_y_list, label='y')
    plt.plot(time_list, position_z_list, label='z')
    POSITION_THRESHOLD = 0.15
    plt.plot(time_list, +np.ones_like(time_list) * POSITION_THRESHOLD,
             '--', color="red", label=f'threshold +{POSITION_THRESHOLD}m')
    plt.plot(time_list, -np.ones_like(time_list) * POSITION_THRESHOLD,
             '--', color="red", label=f'threshold -{POSITION_THRESHOLD}m')
    plt.ylabel('relative position [m]')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0)

    plt.subplot(2, 1, 2)
    plt.plot(time_list, angle_x_list, label='roll')
    plt.plot(time_list, angle_y_list, label='pitch')
    plt.plot(time_list, angle_z_list, label='yaw')
    ANGLE_THRESHOLD = 0.5
    plt.plot(time_list, +np.ones_like(time_list) * ANGLE_THRESHOLD,
             '--', color="red", label=f'threshold +{ANGLE_THRESHOLD}deg')
    plt.plot(time_list, -np.ones_like(time_list) * ANGLE_THRESHOLD,
             '--', color="red", label=f'threshold -{ANGLE_THRESHOLD}deg')
    plt.xlabel('time [s]')
    plt.ylabel('relative angle [deg]')
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0)

    plt.tight_layout()

    save_dir = os.path.dirname(rosbag_path)
    save_path = f"{save_dir}/{save_name}.png"
    plt.savefig(save_path, bbox_inches='tight', pad_inches=0.05)
    plt.close()
    print(f"Saved to {save_path}")
