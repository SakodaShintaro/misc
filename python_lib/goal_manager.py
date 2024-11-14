"""AWSIM + Autowareを動かしているときに無限周回させるためのスクリプト."""

import argparse
import sys
import time
from pathlib import Path

import rclpy
import yaml
from autoware_adapi_v1_msgs.msg import RouteState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.task import Future

# ruff: noqa: ANN101


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--goals_yaml", type=Path, required=True)
    parser.add_argument("--loop_num", type=int, default=1, help="0 means infinite loop")
    return parser.parse_args()


class MissionPlanningNode(Node):
    """Node to publish goals and call service to change operation mode."""

    def __init__(self, goals_yaml: Path, loop_num: int) -> None:
        super().__init__("mission_planning_node")

        with goals_yaml.open() as f:
            self.goal_list = yaml.safe_load(f)
        for goal in self.goal_list:
            print(goal)
        # self.publisher_ = self.create_publisher(PoseStamped, "/rviz/routing/rough_goal", 10)
        self.publisher_ = self.create_publisher(PoseStamped, "/planning/mission_planning/goal", 10)
        self.subscription = self.create_subscription(
            RouteState,
            "/api/routing/state",
            self.goal_callback,
            10,
        )
        self.client_ = self.create_client(
            ChangeOperationMode,
            "/api/operation_mode/change_to_autonomous",
        )
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")
        # Publish initially when the node starts
        self.next_goal_index = 0
        self.curr_loop_num = 0
        self.target_loop_num = loop_num
        self.publish_goal()
        time.sleep(3)
        self.call_engage_service()

    def goal_callback(self, msg: RouteState) -> None:
        """Handle the RouteState message."""
        if msg.state == 3:
            self.next_goal_index += 1
            self.next_goal_index %= len(self.goal_list)

            # check if the loop is finished
            if self.next_goal_index == 0:
                self.curr_loop_num += 1
                self.get_logger().info(f"Loop number: {self.curr_loop_num}")
            if self.target_loop_num > 1 and self.curr_loop_num == self.target_loop_num:
                self.get_logger().info("Finished the target loop")
                sys.exit(0)

            self.publish_goal()
            time.sleep(3)
            self.call_engage_service()

    def publish_goal(self) -> None:
        """Publish the goal."""
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        curr_goal = self.goal_list[self.next_goal_index]
        goal.pose.position.x = curr_goal["position_x"]
        goal.pose.position.y = curr_goal["position_y"]
        goal.pose.position.z = curr_goal["position_z"]
        goal.pose.orientation.x = curr_goal["orientation_x"]
        goal.pose.orientation.y = curr_goal["orientation_y"]
        goal.pose.orientation.z = curr_goal["orientation_z"]
        goal.pose.orientation.w = curr_goal["orientation_w"]
        self.publisher_.publish(goal)
        self.get_logger().info(f"Publishing: {goal}")

    def call_engage_service(self) -> None:
        """Call the service to change the operation mode."""
        req = ChangeOperationMode.Request()
        future = self.client_.call_async(req)
        future.add_done_callback(self.service_callback)

    def service_callback(self, future: Future) -> None:
        """Handle the response from the service call."""
        try:
            response = future.result()
            self.get_logger().info(f"Service call succeeded {response=}")
            if response.status.success:
                self.get_logger().info("Engaged the autonomous mode")
            else:
                self.get_logger().error("Failed to engage the autonomous mode")
                time.sleep(1)
                self.call_engage_service()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Service call failed: {e}")
            time.sleep(1)
            self.call_engage_service()


if __name__ == "__main__":
    args = parse_args()
    goals_yaml = args.goals_yaml
    loop_num = args.loop_num

    rclpy.init()
    mission_planning_node = MissionPlanningNode(goals_yaml, loop_num)
    try:
        rclpy.spin(mission_planning_node)
    except KeyboardInterrupt:
        pass
    finally:
        mission_planning_node.destroy_node()
        rclpy.shutdown()
