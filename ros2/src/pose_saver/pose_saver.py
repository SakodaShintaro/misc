#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv
import os


class PoseSaver(Node):
    def __init__(self):
        super().__init__('pose_saver')
        save_directory = self.declare_parameter("save_directory", ".").value
        os.makedirs(save_directory, exist_ok=True)

        # ekf
        self.sub_pose_ekf = self.create_subscription(
            PoseStamped,
            '/localization/pose_twist_fusion_filter/pose',
            self.ekf_pose_callback,
            1000)
        ekf_pose_path = os.path.join(save_directory, 'ekf_pose.tsv')
        self.ekf_pose_file = open(ekf_pose_path, 'w', newline='')
        self.ekf_pose_writer = csv.writer(self.ekf_pose_file, delimiter='\t')
        self.ekf_pose_writer.writerow(
            ['sec', 'nanosec', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])

        # ndt
        self.sub_pose_ndt = self.create_subscription(
            PoseStamped,
            '/localization/pose_estimator/pose',
            self.ndt_pose_callback,
            1000)
        ndt_pose_path = os.path.join(save_directory, 'ndt_pose.tsv')
        self.ndt_pose_file = open(ndt_pose_path, 'w', newline='')
        self.ndt_pose_writer = csv.writer(self.ndt_pose_file, delimiter='\t')
        self.ndt_pose_writer.writerow(
            ['sec', 'nanosec', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])

    def ekf_pose_callback(self, msg):
        self.get_logger().info('Received ekf pose')
        row = [msg.header.stamp.sec, msg.header.stamp.nanosec, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
               msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        self.ekf_pose_writer.writerow(row)

    def ndt_pose_callback(self, msg):
        self.get_logger().info('Received ndt pose')
        row = [msg.header.stamp.sec, msg.header.stamp.nanosec, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
               msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        self.ndt_pose_writer.writerow(row)

    def __del__(self):
        self.ekf_pose_file.close()
        self.ndt_pose_file.close()


def main(args=None):
    rclpy.init(args=args)
    pose_saver = PoseSaver()
    rclpy.spin(pose_saver)
    pose_saver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
