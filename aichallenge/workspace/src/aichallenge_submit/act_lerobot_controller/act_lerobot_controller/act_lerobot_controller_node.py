#!/usr/bin/env python3
import time

import cv2
import numpy as np
import rclpy
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import VelocityReport
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from act_lerobot_controller.act_lerobot_controller_core import ActLeRobotCore


class ActLeRobotNode(Node):
    def __init__(self):
        super().__init__("act_lerobot_node")

        self.declare_parameter("log_interval_sec", 5.0)
        self.declare_parameter("model.policy_path", "")
        self.declare_parameter("model.dataset_repo_id", "")
        self.declare_parameter("model.dataset_root", "")
        self.declare_parameter("model.device", "cuda")
        self.declare_parameter("model.image_height", 200)
        self.declare_parameter("model.image_width", 320)
        self.declare_parameter("model.crop_top_ratio", 0.375)
        self.declare_parameter("model.crop_bottom_ratio", 0.0)
        self.declare_parameter("state_mode", "none")
        self.declare_parameter("control_mode", "ai")
        self.declare_parameter("acceleration", 0.6)
        self.declare_parameter("debug", False)

        self.debug = self.get_parameter("debug").value
        self.log_interval = self.get_parameter("log_interval_sec").value
        self.latest_odom = None
        self.latest_velocity = None

        self.core = ActLeRobotCore(
            policy_path=self.get_parameter("model.policy_path").value,
            dataset_repo_id=self.get_parameter("model.dataset_repo_id").value,
            dataset_root=self.get_parameter("model.dataset_root").value,
            device=self.get_parameter("model.device").value,
            image_height=self.get_parameter("model.image_height").value,
            image_width=self.get_parameter("model.image_width").value,
            crop_top_ratio=self.get_parameter("model.crop_top_ratio").value,
            crop_bottom_ratio=self.get_parameter("model.crop_bottom_ratio").value,
            state_mode=self.get_parameter("state_mode").value,
            control_mode=self.get_parameter("control_mode").value,
            acceleration=self.get_parameter("acceleration").value,
        )

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, "/image_raw", self.image_callback, qos)
        self.create_subscription(Odometry, "/localization/kinematic_state", self.odom_callback, 1)
        self.create_subscription(VelocityReport, "/vehicle/status/velocity_status", self.velocity_callback, 1)
        self.pub_control = self.create_publisher(AckermannControlCommand, "/control/command/control_cmd", 1)

        self.inference_times = []
        self.last_log_time = self.get_clock().now()
        self.get_logger().info("ActLeRobotNode is ready.")

    def odom_callback(self, msg):
        self.latest_odom = msg

    def velocity_callback(self, msg):
        self.latest_velocity = msg

    def image_callback(self, msg):
        start_time = time.monotonic()
        image = self._image_msg_to_numpy(msg)
        if image is None:
            return

        try:
            accel, steer = self.core.process(image, self._current_state())
        except Exception as exc:
            self.get_logger().error(f"ACT inference failed: {exc}", throttle_duration_sec=5.0)
            return

        cmd = AckermannControlCommand()
        cmd.stamp = self.get_clock().now().to_msg()
        cmd.longitudinal.acceleration = float(accel)
        cmd.lateral.steering_tire_angle = float(steer)
        self.pub_control.publish(cmd)

        if self.debug:
            self.inference_times.append((time.monotonic() - start_time) * 1000.0)
            self._log_performance_metrics()

    def _current_state(self):
        if self.core.state_mode == "odometry" and self.latest_odom is not None:
            twist = self.latest_odom.twist.twist
            return np.array([twist.linear.x, twist.linear.y, twist.angular.z], dtype=np.float32)
        if self.core.state_mode == "vehicle_status" and self.latest_velocity is not None:
            msg = self.latest_velocity
            return np.array([msg.longitudinal_velocity, msg.heading_rate], dtype=np.float32)
        return None

    def _image_msg_to_numpy(self, msg):
        try:
            encoding = msg.encoding.lower()
            if encoding == "bgr8":
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            if encoding == "rgb8":
                return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3).copy()
            if encoding == "bgra8":
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
                return cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
            if encoding == "rgba8":
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
                return cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
            self.get_logger().warn(f"Unsupported image encoding: {msg.encoding}", throttle_duration_sec=5.0)
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}", throttle_duration_sec=5.0)
        return None

    def _log_performance_metrics(self):
        now = self.get_clock().now()
        if (now - self.last_log_time).nanoseconds / 1e9 <= self.log_interval:
            return
        if self.inference_times:
            avg_time = np.mean(self.inference_times)
            fps = 1000.0 / avg_time if avg_time > 0 else 0.0
            self.get_logger().info(f"DEBUG: Avg Inference: {avg_time:.2f}ms ({fps:.2f}Hz)")
            self.inference_times.clear()
        self.last_log_time = now


def main(args=None):
    rclpy.init(args=args)
    node = ActLeRobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
