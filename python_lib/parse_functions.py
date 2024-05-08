""" The library to parse the data from rosbag file.
"""


def parse_stamp(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


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


def parse_Float32Stamped(msg):
    return {
        "timestamp": parse_stamp(msg.stamp),
        "data": msg.data,
    }
