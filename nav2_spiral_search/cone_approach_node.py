#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

from nav2_simple_commander.robot_navigator import BasicNavigator

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs.tf2_geometry_msgs as tf2_gm

from rclpy.duration import Duration
from rclpy.time import Time
from nav2_simple_commander.robot_navigator import TaskResult



class ConeApproachNode(Node):
    def __init__(self):
        super().__init__('cone_approach_node')

        # -----------------------------
        # Nav2 interface
        # -----------------------------
        self.navigator = BasicNavigator()
        self.get_logger().info("Waiting for Nav2 to become active...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active.")

        # -----------------------------
        # TF
        # -----------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -----------------------------
        # State
        # -----------------------------
        self.cone_detected = False
        self.goal_sent = False
        self.latest_cone_pose = None

        # -----------------------------
        # Subscribers
        # -----------------------------
        self.create_subscription(
            Bool,
            "/cone_detected",
            self.cone_detected_callback,
            10
        )

        self.create_subscription(
            PoseStamped,
            "/cone_pose",
            self.cone_pose_callback,
            10
        )

        self.result_timer = None

        self.get_logger().info("ConeApproachNode READY. Waiting for cone.")

    # =====================================
    # Callbacks
    # =====================================
    def cone_detected_callback(self, msg: Bool):
        if msg.data:
            self.cone_detected = True
            self.try_send_goal()

    def cone_pose_callback(self, msg: PoseStamped):
        self.latest_cone_pose = msg
        self.try_send_goal()

    # =====================================
    # Core logic
    # =====================================
    def try_send_goal(self):
        if not self.cone_detected:
            return

        if self.goal_sent:
            return

        if self.latest_cone_pose is None:
            return

        # -----------------------------
        # Reject stale cone poses
        # -----------------------------
        msg_time = rclpy.time.Time.from_msg(
            self.latest_cone_pose.header.stamp
        )

        age = (self.get_clock().now() - msg_time).nanoseconds * 1e-9

        if age > 0.5:
            self.get_logger().warn("Cone pose too old, waiting for fresh data...")
            return


        # -----------------------------
        # TF availability check
        # -----------------------------
        target_frame = self.latest_cone_pose.header.frame_id

        if not self.tf_buffer.can_transform(
            "map",
            target_frame,
            Time(),
            timeout=Duration(seconds=1.0)
        ):
            self.get_logger().warn("Waiting for TF transform to map...")
            return

        # -----------------------------
        # Transform cone pose to map
        # -----------------------------
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                target_frame,
                Time()
            )

            cone_in_map = tf2_gm.do_transform_pose_stamped(
                self.latest_cone_pose,
                transform
            )

        except Exception as e:
            self.get_logger().warn(f"TF transform failed: {e}")
            return

        # -----------------------------
        # Send Nav2 goal
        # -----------------------------
        self.get_logger().warn("Cone detected → sending Nav2 goal.")
        self.navigator.goToPose(cone_in_map)
        self.goal_sent = True

        self.result_timer = self.create_timer(
            0.5,
            self.check_navigation_result
        )

    # =====================================
    # Result handling
    # =====================================
    def check_navigation_result(self):
        if not self.navigator.isTaskComplete():
            return

        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Successfully reached the cone.")
        else:
            self.get_logger().warn(f"Failed to reach cone. Result: {result}")

        self.result_timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = ConeApproachNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()