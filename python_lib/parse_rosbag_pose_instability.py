import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import argparse
import matplotlib.pyplot as plt


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

    storage_filter = rosbag2_py.StorageFilter(topics=["/diagnostics"])
    reader.set_filter(storage_filter)

    t0 = None

    diff_position_x_list = []
    diff_position_y_list = []
    diff_position_z_list = []
    diff_angle_x_list = []
    diff_angle_y_list = []
    diff_angle_z_list = []

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        t /= 1e9
        if t0 is None:
            t0 = t
        else:
            t -= t0
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        skip = (len(msg.status) == 0)
        for status in msg.status:
            skip = skip or ("localization: pose_" not in status.name)
        if skip:
            continue

        assert len(msg.status) == 1, f"{msg}, {skip}"
        message = msg.status[0].message
        key_value_map = {kv.key: kv.value for kv in msg.status[0].values}
        diff_position_x_list.append(float(key_value_map["diff_position_x:value"]))
        diff_position_y_list.append(float(key_value_map["diff_position_y:value"]))
        diff_position_z_list.append(float(key_value_map["diff_position_z:value"]))
        diff_angle_x_list.append(float(key_value_map["diff_angle_x:value"]))
        diff_angle_y_list.append(float(key_value_map["diff_angle_y:value"]))
        diff_angle_z_list.append(float(key_value_map["diff_angle_z:value"]))

    # 2行に分けて表示する

    # 1行目:position
    plt.subplot(2, 1, 1)
    plt.plot(diff_position_x_list, label="diff_position_x")
    plt.plot(diff_position_y_list, label="diff_position_y")
    plt.plot(diff_position_z_list, label="diff_position_z")
    THRESHOLD = 0.1
    plt.plot([+THRESHOLD] * len(diff_position_x_list), label=f"threshold(+{THRESHOLD})", linestyle="dashed", color="red")
    plt.plot([-THRESHOLD] * len(diff_position_x_list), label=f"threshold(-{THRESHOLD})", linestyle="dashed", color="red")
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0)
    plt.ylabel("diff_position [m]")

    # 2行目:angle
    plt.subplot(2, 1, 2)
    plt.plot(diff_angle_x_list, label="diff_angle_x")
    plt.plot(diff_angle_y_list, label="diff_angle_y")
    plt.plot(diff_angle_z_list, label="diff_angle_z")
    THRESHOLD = 0.01
    plt.plot([+THRESHOLD] * len(diff_angle_x_list), label=f"threshold(+{THRESHOLD})", linestyle="dashed", color="red")
    plt.plot([-THRESHOLD] * len(diff_angle_x_list), label=f"threshold(-{THRESHOLD})", linestyle="dashed", color="red")
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0)
    plt.ylabel("diff_position [rad]")

    plt.xlabel("frame number")
    save_path = f"{rosbag_path}/diff_pose.png"
    plt.savefig(save_path, bbox_inches='tight', pad_inches=0.05)
    plt.close()
    print(f"Saved {save_path}")
