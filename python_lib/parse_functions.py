""" The library to parse the data from rosbag file.
"""

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from cv_bridge import CvBridge
from collections import defaultdict
import pandas as pd


def parse_rosbag(rosbag_path: str, target_topic_list):
    serialization_format = "cdr"
    storage_options = rosbag2_py.StorageOptions(uri=rosbag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format,
    )

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {
        topic_types[i].name: topic_types[i].type for i in range(len(topic_types))
    }

    storage_filter = rosbag2_py.StorageFilter(topics=target_topic_list)
    reader.set_filter(storage_filter)

    topic_name_to_data = defaultdict(list)
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        if topic in target_topic_list:
            topic_name_to_data[topic].append(parse_msg(msg, msg_type))
    for key in target_topic_list:
        topic_name_to_data[key] = pd.DataFrame(topic_name_to_data[key])
        print(f"{key}: {len(topic_name_to_data[key])} msgs")
    return topic_name_to_data


def parse_stamp(stamp):
    return stamp.sec * int(1e9) + stamp.nanosec


def parse_msg(msg, msg_type):
    class_name = msg_type.__class__.__name__.replace("Metaclass_", "")
    if class_name == "Float32Stamped":
        return parse_Float32Stamped(msg)
    elif class_name == "Int32Stamped":
        return parse_Float32Stamped(msg)
    elif class_name == "PoseStamped":
        return parse_PoseStamped(msg)
    elif class_name == "PoseWithCovarianceStamped":
        return parse_PoseWithCovarianceStamped(msg)
    elif class_name == "TwistWithCovarianceStamped":
        return parse_TwistWithCovarianceStamped(msg)
    elif class_name == "Odometry":
        return parse_Odometry(msg)
    elif class_name == "MarkerArray":
        return parse_MarkerArray(msg)
    elif class_name == "Image":
        return parse_Image(msg)
    elif class_name == "CompressedImage":
        return parse_CompressedImage(msg)
    elif class_name == "CameraInfo":
        return parse_CameraInfo(msg)
    elif class_name == "TFMessage":
        return parse_TFMessage(msg)
    elif class_name == "Imu":
        return parse_Imu(msg)
    elif class_name == "VelocityReport":
        return parse_VelocityReport(msg)
    else:
        print(f"Error: {class_name} is not supported.")
        exit(0)


def parse_PoseStamped(msg):
    return {
        "timestamp": parse_stamp(msg.header.stamp),
        "position.x": msg.pose.position.x,
        "position.y": msg.pose.position.y,
        "position.z": msg.pose.position.z,
        "orientation.x": msg.pose.orientation.x,
        "orientation.y": msg.pose.orientation.y,
        "orientation.z": msg.pose.orientation.z,
        "orientation.w": msg.pose.orientation.w,
    }


def parse_PoseWithCovarianceStamped(msg):
    return {
        "timestamp": parse_stamp(msg.header.stamp),
        "position.x": msg.pose.pose.position.x,
        "position.y": msg.pose.pose.position.y,
        "position.z": msg.pose.pose.position.z,
        "orientation.x": msg.pose.pose.orientation.x,
        "orientation.y": msg.pose.pose.orientation.y,
        "orientation.z": msg.pose.pose.orientation.z,
        "orientation.w": msg.pose.pose.orientation.w,
        "covariance_position.xx": msg.pose.covariance[0],
        "covariance_position.xy": msg.pose.covariance[1],
        "covariance_position.xz": msg.pose.covariance[2],
        "covariance_position.yx": msg.pose.covariance[6],
        "covariance_position.yy": msg.pose.covariance[7],
        "covariance_position.yz": msg.pose.covariance[8],
        "covariance_position.zx": msg.pose.covariance[12],
        "covariance_position.zy": msg.pose.covariance[13],
        "covariance_position.zz": msg.pose.covariance[14],
        "covariance_angle.x": msg.pose.covariance[21],
        "covariance_angle.y": msg.pose.covariance[28],
        "covariance_angle.z": msg.pose.covariance[35],
    }


def parse_TwistWithCovarianceStamped(msg):
    return {
        "timestamp": parse_stamp(msg.header.stamp),
        "linear_velocity.x": msg.twist.twist.linear.x,
        "linear_velocity.y": msg.twist.twist.linear.y,
        "linear_velocity.z": msg.twist.twist.linear.z,
        "angular_velocity.x": msg.twist.twist.angular.x,
        "angular_velocity.y": msg.twist.twist.angular.y,
        "angular_velocity.z": msg.twist.twist.angular.z,
        "covariance_position.xx": msg.twist.covariance[0],
        "covariance_position.xy": msg.twist.covariance[1],
        "covariance_position.xz": msg.twist.covariance[2],
        "covariance_position.yx": msg.twist.covariance[6],
        "covariance_position.yy": msg.twist.covariance[7],
        "covariance_position.yz": msg.twist.covariance[8],
        "covariance_position.zx": msg.twist.covariance[12],
        "covariance_position.zy": msg.twist.covariance[13],
        "covariance_position.zz": msg.twist.covariance[14],
        "covariance_angle.x": msg.twist.covariance[21],
        "covariance_angle.y": msg.twist.covariance[28],
        "covariance_angle.z": msg.twist.covariance[35],
    }


def parse_Odometry(msg):
    return {
        "timestamp": parse_stamp(msg.header.stamp),
        "position.x": msg.pose.pose.position.x,
        "position.y": msg.pose.pose.position.y,
        "position.z": msg.pose.pose.position.z,
        "orientation.x": msg.pose.pose.orientation.x,
        "orientation.y": msg.pose.pose.orientation.y,
        "orientation.z": msg.pose.pose.orientation.z,
        "orientation.w": msg.pose.pose.orientation.w,
        "covariance_position.xx": msg.pose.covariance[0],
        "covariance_position.xy": msg.pose.covariance[1],
        "covariance_position.xz": msg.pose.covariance[2],
        "covariance_position.yx": msg.pose.covariance[6],
        "covariance_position.yy": msg.pose.covariance[7],
        "covariance_position.yz": msg.pose.covariance[8],
        "covariance_position.zx": msg.pose.covariance[12],
        "covariance_position.zy": msg.pose.covariance[13],
        "covariance_position.zz": msg.pose.covariance[14],
        "covariance_angle.x": msg.pose.covariance[21],
        "covariance_angle.y": msg.pose.covariance[28],
        "covariance_angle.z": msg.pose.covariance[35],
        "linear_velocity.x": msg.twist.twist.linear.x,
        "linear_velocity.y": msg.twist.twist.linear.y,
        "linear_velocity.z": msg.twist.twist.linear.z,
        "angular_velocity.x": msg.twist.twist.angular.x,
        "angular_velocity.y": msg.twist.twist.angular.y,
        "angular_velocity.z": msg.twist.twist.angular.z,
    }


def parse_Float32Stamped(msg):
    return {
        "timestamp": parse_stamp(msg.stamp),
        "data": msg.data,
    }


def parse_Int32Stamped(msg):
    return {
        "timestamp": parse_stamp(msg.stamp),
        "data": msg.data,
    }


def parse_MarkerArray(msg):
    result_dict = {}
    result_dict["timestamp"] = parse_stamp(msg.markers[0].header.stamp)
    result_dict["marker"] = list()
    for marker_msg in msg.markers:
        one_marker = {}
        one_marker["timestamp"] = parse_stamp(marker_msg.header.stamp)
        one_marker["position.x"] = marker_msg.pose.position.x
        one_marker["position.y"] = marker_msg.pose.position.y
        one_marker["position.z"] = marker_msg.pose.position.z
        one_marker["orientation.x"] = marker_msg.pose.orientation.x
        one_marker["orientation.y"] = marker_msg.pose.orientation.y
        one_marker["orientation.z"] = marker_msg.pose.orientation.z
        one_marker["orientation.w"] = marker_msg.pose.orientation.w
        result_dict["marker"].append(one_marker)
    return result_dict


def parse_Image(msg):
    image = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")
    return {
        "timestamp": parse_stamp(msg.header.stamp),
        "frame_id": msg.header.frame_id,
        "image": image,
    }


def parse_CompressedImage(msg):
    image = CvBridge().compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
    return {
        "timestamp": parse_stamp(msg.header.stamp),
        "frame_id": msg.header.frame_id,
        "image": image,
    }


def parse_CameraInfo(msg):
    return {
        "timestamp": parse_stamp(msg.header.stamp),
        "frame_id": msg.header.frame_id,
        "width": msg.width,
        "height": msg.height,
        "D": msg.d,
        "K": msg.k,
        "R": msg.r,
        "P": msg.p,
    }


def parse_TFMessage(msg):
    return {
        "transforms": msg.transforms,
    }


def parse_Imu(msg):
    return {
        "timestamp": parse_stamp(msg.header.stamp),
        "frame_id": msg.header.frame_id,
        "angular_velocity.x": msg.angular_velocity.x,
        "angular_velocity.y": msg.angular_velocity.y,
        "angular_velocity.z": msg.angular_velocity.z,
        "linear_acceleration.x": msg.linear_acceleration.x,
        "linear_acceleration.y": msg.linear_acceleration.y,
        "linear_acceleration.z": msg.linear_acceleration.z,
        "orientation.x": msg.orientation.x,
        "orientation.y": msg.orientation.y,
        "orientation.z": msg.orientation.z,
        "orientation.w": msg.orientation.w,
    }


def parse_VelocityReport(msg):
    return {
        "timestamp": parse_stamp(msg.header.stamp),
        "longitudinal_velocity": msg.longitudinal_velocity,
        "lateral_velocity": msg.lateral_velocity,
        "heading_rate": msg.heading_rate,
    }
