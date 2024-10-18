"""AWSIM + Autowareを動かしているときに無限周回させるためのスクリプト."""

import time

import rclpy
from autoware_adapi_v1_msgs.msg import RouteState
from autoware_adapi_v1_msgs.srv import ChangeOperationMode
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.task import Future

# ruff: noqa: ANN101


class MissionPlanningNode(Node):
    """Node to publish goals and call service to change operation mode."""

    def __init__(self) -> None:
        super().__init__("mission_planning_node")
        self.goal_list = [
            [
                81536.5703,
                50199.3632,
                34.2321,
                +0.0099193127825856,
                -0.0045161009766161,
                -0.0861385315656662,
                -0.9962236285209656,
            ],
            [
                81726.4218,
                50069.6367,
                41.0830,
                -0.0042670401744544,
                +0.0036174126435071,
                -0.6379052996635437,
                +0.7700945734977722,
            ],
        ]
        self.publisher_ = self.create_publisher(PoseStamped, "/rviz/routing/rough_goal", 10)
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
        self.publish_goal()
        time.sleep(3)
        self.call_service()

    def goal_callback(self, msg: RouteState) -> None:
        """Handle the RouteState message."""
        if msg.state == 3:
            self.next_goal_index += 1
            self.next_goal_index %= len(self.goal_list)
            self.publish_goal()
            time.sleep(3)
            self.call_service()

    def publish_goal(self) -> None:
        """Publish the goal."""
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        (
            goal.pose.position.x,
            goal.pose.position.y,
            goal.pose.position.z,
            goal.pose.orientation.x,
            goal.pose.orientation.y,
            goal.pose.orientation.z,
            goal.pose.orientation.w,
        ) = self.goal_list[self.next_goal_index]
        self.publisher_.publish(goal)
        self.get_logger().info(f"Publishing: {goal}")

    def call_service(self) -> None:
        """Call the service to change the operation mode."""
        req = ChangeOperationMode.Request()
        future = self.client_.call_async(req)
        future.add_done_callback(self.service_callback)

    def service_callback(self, future: Future) -> None:
        """Handle the response from the service call."""
        try:
            response = future.result()
            self.get_logger().info(f"Service call succeeded {response=}")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Service call failed: {e}")


if __name__ == "__main__":
    rclpy.init()
    mission_planning_node = MissionPlanningNode()
    try:
        rclpy.spin(mission_planning_node)
    except KeyboardInterrupt:
        pass
    finally:
        mission_planning_node.destroy_node()
        rclpy.shutdown()
