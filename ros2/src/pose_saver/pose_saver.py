#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv
import os


class PoseSaver(Node):
    def __init__(self):
        super().__init__('pose_saver')
        self.localization_subscription = self.create_subscription(
            PoseStamped,
            '/localization/pose_twist_fusion_filter/pose',
            self.localization_callback,
            10)

        save_directory = self.declare_parameter("save_directory", ".").value
        os.makedirs(save_directory, exist_ok=True)
        localization_path = os.path.join(save_directory, 'localization.tsv')
        self.localization_file = open(localization_path, 'w', newline='')
        self.localization_writer = csv.writer(
            self.localization_file, delimiter='\t')
        self.localization_writer.writerow(
            ['sec', 'nanosec', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])

    def localization_callback(self, msg):
        self.get_logger().info('Received localization pose')
        row = [msg.header.stamp.sec, msg.header.stamp.nanosec, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
               msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        self.localization_writer.writerow(row)

    def __del__(self):
        self.localization_file.close()


def main(args=None):
    rclpy.init(args=args)
    pose_saver = PoseSaver()
    rclpy.spin(pose_saver)
    pose_saver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
