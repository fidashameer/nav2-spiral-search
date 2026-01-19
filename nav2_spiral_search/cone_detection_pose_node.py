#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import Bool
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

from cv_bridge import CvBridge
from ultralytics import YOLO

import cv2
import numpy as np


class ConeDetectionPoseNode(Node):
    def __init__(self):
        super().__init__('cone_detection_pose_node')

        # --------------------------
        # Parameters
        # --------------------------
        self.FRAME_ID = "camera_color_optical_frame"
        self.NAV_OFFSET = 0.5  # stop before cone

        weight_path = os.path.join(
            get_package_share_directory('nav2_spiral_search'),
            "nav2_spiral_search",
            "weights.pt"
        )

        self.model = YOLO(weight_path)
        self.bridge = CvBridge()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # --------------------------
        # Subscribers
        # --------------------------
        self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.image_callback,
            qos
        )

        self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self.camera_info_callback,
            qos
        )

        self.create_subscription(
            Image,
            "/camera/camera/depth/image_rect_raw",
            self.depth_callback,
            qos
        )

        # --------------------------
        # Publishers
        # --------------------------
        self.cone_detected_pub = self.create_publisher(Bool, "/cone_detected", 10)
        self.cone_pose_pub = self.create_publisher(PoseStamped, "/cone_pose", 10)
        self.marker_pub = self.create_publisher(Marker, "/cone_marker", 10)

        # --------------------------
        # Camera state
        # --------------------------
        self.fx = self.fy = self.cx = self.cy = None
        self.latest_depth = None
        self.depth_encoding = None

        self.get_logger().info("ConeDetectionPoseNode READY.")

    # ==========================
    # Callbacks
    # ==========================
    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )
        self.depth_encoding = msg.encoding

    def image_callback(self, msg):
        if self.fx is None or self.latest_depth is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        results = self.model(
            frame,
            verbose=False,
            classes=[0],     # cone class
            max_det=1,
            conf=0.5
        )[0]

        if len(results.boxes) == 0:
            self.publish_no_detection()
            return

        box = results.boxes[0]
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        cx = int((x1 + x2) / 2)
        cy = int((y1 + y2) / 2)

        h, w = self.latest_depth.shape[:2]
        if not (0 <= cx < w and 0 <= cy < h):
            self.publish_no_detection()
            return

        depth = self.latest_depth[cy, cx]
        if not np.isfinite(depth) or depth <= 0:
            self.publish_no_detection()
            return

        Z = float(depth) / 1000.0 if self.depth_encoding == "16UC1" else float(depth)
        X = (cx - self.cx) * Z / self.fx
        Y = (cy - self.cy) * Z / self.fy
        Z_goal = max(0.0, Z - self.NAV_OFFSET)

        self.publish_detection(X, Y, Z_goal)

    # ==========================
    # Publishing helpers
    # ==========================
    def publish_no_detection(self):
        self.cone_detected_pub.publish(Bool(data=False))

        marker = Marker()
        marker.action = Marker.DELETEALL
        marker.header.frame_id = self.FRAME_ID
        self.marker_pub.publish(marker)

    def publish_detection(self, X, Y, Z):
        self.cone_detected_pub.publish(Bool(data=True))

        pose = PoseStamped()
        pose.header.frame_id = self.FRAME_ID
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = X
        pose.pose.position.y = Y
        pose.pose.position.z = Z
        pose.pose.orientation.w = 1.0

        self.cone_pose_pub.publish(pose)

        marker = Marker()
        marker.header = pose.header
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = 0.15
        marker.color.r = 1.0
        marker.color.a = 1.0
        marker.pose.position.x = X
        marker.pose.position.y = Y
        marker.pose.position.z = Z + self.NAV_OFFSET

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = ConeDetectionPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()