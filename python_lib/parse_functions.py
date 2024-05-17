""" The library to parse the data from rosbag file.
"""


def parse_stamp(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


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
    elif class_name == "Odometry":
        return parse_Odometry(msg)
    elif class_name == "MarkerArray":
        return parse_MarkerArray(msg)
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
