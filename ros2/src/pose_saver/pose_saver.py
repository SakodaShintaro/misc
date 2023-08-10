#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv


class PoseSaver(Node):
    def __init__(self):
        super().__init__('pose_saver')
        self.ground_truth_subscription = self.create_subscription(
            PoseStamped,
            '/awsim/ground_truth/vehicle/pose',
            self.ground_truth_callback,
            10)
        self.localization_subscription = self.create_subscription(
            PoseStamped,
            '/localization/pose_twist_fusion_filter/pose',
            self.localization_callback,
            10)

        self.ground_truth_file = open('ground_truth.tsv', 'w', newline='')
        self.localization_file = open('localization.tsv', 'w', newline='')
        self.ground_truth_writer = csv.writer(
            self.ground_truth_file, delimiter='\t')
        self.localization_writer = csv.writer(
            self.localization_file, delimiter='\t')
        self.ground_truth_writer.writerow(
            ['sec', 'nanosec', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        self.localization_writer.writerow(
            ['sec', 'nanosec', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])

    def ground_truth_callback(self, msg):
        self.get_logger().info('Received ground truth pose')
        row = [msg.header.stamp.sec, msg.header.stamp.nanosec, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
               msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        self.ground_truth_writer.writerow(row)

    def localization_callback(self, msg):
        self.get_logger().info('Received localization pose')
        row = [msg.header.stamp.sec, msg.header.stamp.nanosec, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
               msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        self.localization_writer.writerow(row)

    def __del__(self):
        self.ground_truth_file.close()
        self.localization_file.close()


def main(args=None):
    rclpy.init(args=args)
    pose_saver = PoseSaver()
    rclpy.spin(pose_saver)
    pose_saver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
