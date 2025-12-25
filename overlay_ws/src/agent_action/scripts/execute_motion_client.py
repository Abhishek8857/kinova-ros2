#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from agent_action_interface.action import ExecuteMotion


class MotionClient(Node):

    def __init__(self):
        super().__init__("motion_action_client")

        # Action Client
        self._client = ActionClient(self, ExecuteMotion, "execute_motion")

        # Subscription
        self.subscription = self.create_subscription(
            Float64MultiArray,
            "published_coordinates",
            self.agent_callback,
            10,
        )

        self._goal_in_flight = False
        self.get_logger().info(
            "Motion action client initialised and listening to 'published_coordinates'..."
        )

    def agent_callback(self, msg: Float64MultiArray):
        # Avoid sending multiple goals at once
        if self._goal_in_flight:
            self.get_logger().warn("Goal already in flight. Ignoring new coordinates.")
            return

        data = list(msg.data)
        self.get_logger().info(f"Received topic goal: {data}")

        goal_msg = ExecuteMotion.Goal()
        goal_msg.data = data

        self._client.wait_for_server()
        self._goal_in_flight = True

        send_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            self._goal_in_flight = False
            return

        self.get_logger().info("Goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"Feedback: state={fb.state}, progress={fb.progress}"
        )

    def result_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(
                f"Result: success={result.success}, "
                f"code={result.error_code}, "
                f"description={result.error_description}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to get result: {e}")
        finally:
            self._goal_in_flight = False


def main(args=None):
    rclpy.init(args=args)
    node = MotionClient()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
