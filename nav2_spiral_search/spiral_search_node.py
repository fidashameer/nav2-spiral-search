#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_simple_commander.robot_navigator import BasicNavigator
from tf2_ros import Buffer, TransformListener


class SpiralSearchNode(Node):
    def __init__(self):
        super().__init__("spiral_search_node")
        self.get_logger().info("Spiral Search Node (Waypoint Follower) starting")

        # -----------------------------
        # Nav2
        # -----------------------------
        self.navigator = BasicNavigator()
        self.get_logger().info("Waiting for Nav2...")
        self.navigator.waitUntilNav2Active()
        self.get_logger().info("Nav2 active")

        # -----------------------------
        # TF
        # -----------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -----------------------------
        # State
        # -----------------------------
        self.cone_detected = False

        # -----------------------------
        # Spiral parameters
        # -----------------------------
        self.start_radius = 0.3
        self.radius_step = 0.25
        self.angle_step = math.pi / 8
        self.max_radius = 4.0

        # -----------------------------
        # Visualization
        # -----------------------------
        self.path_pub = self.create_publisher(Path, "/spiral_path", 10)
        self.spiral_path = Path()
        self.spiral_path.header.frame_id = "map"

        # -----------------------------
        # Lock spiral center
        # -----------------------------
        self.center_x, self.center_y = self.get_stable_initial_pose()
        self.get_logger().info(
            f"Spiral center locked at x={self.center_x:.2f}, y={self.center_y:.2f}"
        )

        # -----------------------------
        # Generate spiral
        # -----------------------------
        self.spiral_waypoints = self.generate_spiral_waypoints()
        self.get_logger().info(
            f"Generated {len(self.spiral_waypoints)} spiral waypoints"
        )

        # Publish spiral path
        self.spiral_path.poses = self.spiral_waypoints
        self.spiral_path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.spiral_path)

        # -----------------------------
        # Start waypoint follower (ONCE)
        # -----------------------------
        self.navigator.followWaypoints(self.spiral_waypoints)
        self.get_logger().info("Spiral waypoint following started")

        # -----------------------------
        # Subscribers
        # -----------------------------
        self.create_subscription(Bool, "/cone_detected", self.cone_callback, 10)

        # -----------------------------
        # Result monitor
        # -----------------------------
        self.result_timer = self.create_timer(0.5, self.check_result)

    # ==================================================
    def get_stable_initial_pose(self, samples=5, tolerance=0.05):
        poses = []
        while rclpy.ok():
            try:
                tf = self.tf_buffer.lookup_transform(
                    "map", "base_link", rclpy.time.Time()
                )
                x = tf.transform.translation.x
                y = tf.transform.translation.y
                poses.append((x, y))

                if len(poses) >= samples:
                    xs, ys = zip(*poses)
                    mx = sum(xs) / len(xs)
                    my = sum(ys) / len(ys)
                    if all(abs(px - mx) < tolerance and abs(py - my) < tolerance for px, py in poses):
                        return mx, my
                    poses.clear()

            except Exception:
                self.get_logger().info("Waiting for stable localization...")

            rclpy.spin_once(self, timeout_sec=0.5)

    # ==================================================
    def generate_spiral_waypoints(self):
        waypoints = []
        radius = self.start_radius
        angle = 0.0

        while radius <= self.max_radius:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = self.center_x + radius * math.cos(angle)
            pose.pose.position.y = self.center_y + radius * math.sin(angle)
            pose.pose.orientation.w = 1.0

            waypoints.append(pose)
            angle += self.angle_step
            radius += self.radius_step

        return waypoints

    # ==================================================
    def cone_callback(self, msg: Bool):
        if msg.data and not self.cone_detected:
            self.cone_detected = True
            self.get_logger().warn("Cone detected → cancelling spiral")
            self.navigator.cancelTask()

    def check_result(self):
        if not self.navigator.isTaskComplete():
            return

        result = self.navigator.getResult()
        if result == self.navigator.Result.SUCCEEDED:
            self.get_logger().info("Spiral search completed successfully")
        else:
            self.get_logger().warn(f"Spiral stopped. Result: {result}")

        self.result_timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = SpiralSearchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# T1:
# source /opt/ros/humble/setup.bash
# export TURTLEBOT3_MODEL=waffle
# ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

#T2:
# source /opt/ros/humble/setup.bash
# export TURTLEBOT3_MODEL=waffle
# ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True

# T3:# cd ~/ROS2/project_cartier/cartier_space
# source install/setup.bash
# ros2 run cartier_behaviours spiral_search_node.py

# T4
# ros2 topic pub /cone_detected std_msgs/Bool "{data: true}" --once

# Simulating cone for stop spiral: ros2 topic pub /cone_detected std_msgs/Bool "{data: true}" --once
# Resetting spiral without restarting node: ros2 topic pub /spiral_reset std_msgs/Bool "{data: true}" --once