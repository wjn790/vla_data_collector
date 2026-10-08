from __future__ import annotations

import os
import threading
import time
from typing import Any

import numpy as np


class RosRobotAdapter:
    def __init__(self, config: dict[str, Any], *, read_only: bool = False) -> None:
        os.environ["ROS_DOMAIN_ID"] = str(config["domain_id"])
        os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState

        self.rclpy = rclpy
        self.Twist = Twist
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node("svt_long_task_supervisor")
        self.config = config
        self.read_only = bool(read_only)
        self.joint_names = list(config["joint_names"])
        self.lock = threading.Lock()
        self.joint_position: np.ndarray | None = None
        self.joint_effort: np.ndarray | None = None
        self.joint_received_at = 0.0
        self.base_feedback = (0.0, 0.0, 0.0)
        self.base_received_at = 0.0
        self.stop_event = threading.Event()

        self.node.create_subscription(
            JointState,
            config["joint_state_topic"],
            self._joint_callback,
            qos_profile_sensor_data,
        )
        self.node.create_subscription(
            Twist,
            config["base_feedback_topic"],
            self._base_callback,
            10,
        )
        self.spin_thread = threading.Thread(target=self._spin, name="svt-supervisor-ros", daemon=True)
        self.spin_thread.start()
        time.sleep(0.25)

        if self.read_only:
            # Shadow mode subscribes to feedback only. It deliberately creates no
            # command publishers so it cannot emit a control message on shutdown.
            return

        command_topics = (
            config["left_arm_command_topic"],
            config["right_arm_command_topic"],
            config["base_command_topic"],
        )
        conflicts = {
            topic: [info.node_name for info in self.node.get_publishers_info_by_topic(topic)]
            for topic in command_topics
            if self.node.get_publishers_info_by_topic(topic)
        }
        if conflicts:
            self.close()
            raise RuntimeError(
                f"command topics already have publishers; disable teleoperation first: {conflicts}"
            )

        from std_msgs.msg import Float64MultiArray

        self.Float64MultiArray = Float64MultiArray
        self.left_publisher = self.node.create_publisher(
            Float64MultiArray, config["left_arm_command_topic"], 10
        )
        self.right_publisher = self.node.create_publisher(
            Float64MultiArray, config["right_arm_command_topic"], 10
        )
        self.base_publisher = self.node.create_publisher(Twist, config["base_command_topic"], 10)

    def assert_command_ownership(self) -> None:
        own_name = self.node.get_name()
        conflicts = {}
        for topic in (
            self.config["left_arm_command_topic"],
            self.config["right_arm_command_topic"],
            self.config["base_command_topic"],
        ):
            others = [
                info.node_name
                for info in self.node.get_publishers_info_by_topic(topic)
                if info.node_name != own_name
            ]
            if others:
                conflicts[topic] = others
        if conflicts:
            raise RuntimeError(f"another process acquired a command topic: {conflicts}")

    def _spin(self) -> None:
        while not self.stop_event.is_set() and self.rclpy.ok():
            self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def _joint_callback(self, message: Any) -> None:
        positions = {str(name): index for index, name in enumerate(message.name)}
        if any(name not in positions for name in self.joint_names):
            return
        position = np.asarray([message.position[positions[name]] for name in self.joint_names])
        effort = np.asarray(
            [
                message.effort[positions[name]] if positions[name] < len(message.effort) else 0.0
                for name in self.joint_names
            ]
        )
        if position.shape != (14,) or not np.isfinite(position).all():
            return
        with self.lock:
            self.joint_position = position
            self.joint_effort = effort
            self.joint_received_at = time.monotonic()

    def _base_callback(self, message: Any) -> None:
        with self.lock:
            self.base_feedback = (
                float(message.linear.x),
                float(message.linear.y),
                float(message.angular.z),
            )
            self.base_received_at = time.monotonic()

    def wait_ready(self, timeout_sec: float = 10.0) -> None:
        require_base_feedback = bool(self.config.get("require_base_feedback", True))
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            with self.lock:
                ready = self.joint_position is not None and (
                    not require_base_feedback or self.base_received_at > 0
                )
            if ready:
                return
            time.sleep(0.05)
        required = "joint/base" if require_base_feedback else "joint"
        raise RuntimeError(f"ROS {required} feedback did not become ready")

    def base_feedback_diagnostic(self) -> dict[str, Any]:
        """Return base feedback health without making it an execution prerequisite."""
        with self.lock:
            feedback = tuple(self.base_feedback)
            received_at = self.base_received_at
        age = float("inf") if received_at <= 0 else time.monotonic() - received_at
        return {
            "available": received_at > 0,
            "age_sec": age,
            "vx": feedback[0],
            "vy": feedback[1],
            "wz": feedback[2],
        }

    def arm_state(self, max_age_sec: float = 0.2) -> tuple[np.ndarray, np.ndarray]:
        with self.lock:
            position = None if self.joint_position is None else self.joint_position.copy()
            effort = None if self.joint_effort is None else self.joint_effort.copy()
            age = time.monotonic() - self.joint_received_at
        if position is None or effort is None or age > max_age_sec:
            raise RuntimeError(f"arm feedback is unavailable or stale ({age:.3f}s)")
        return position, effort

    def base_state(self, max_age_sec: float = 0.3) -> tuple[float, float, float]:
        with self.lock:
            feedback = tuple(self.base_feedback)
            age = time.monotonic() - self.base_received_at
        if age > max_age_sec:
            raise RuntimeError(f"base feedback is stale ({age:.3f}s)")
        return feedback

    def publish_arms(self, command: np.ndarray) -> None:
        if self.read_only:
            raise RuntimeError("read-only ROS adapter cannot publish arm commands")
        values = np.asarray(command, dtype=float)
        if values.shape != (14,) or not np.isfinite(values).all():
            raise ValueError("arm command must contain 14 finite values")
        left = self.Float64MultiArray()
        right = self.Float64MultiArray()
        left.data = values[:7].tolist()
        right.data = values[7:].tolist()
        self.left_publisher.publish(left)
        self.right_publisher.publish(right)

    def publish_base(self, vx: float, vy: float, wz: float) -> None:
        if self.read_only:
            raise RuntimeError("read-only ROS adapter cannot publish base commands")
        message = self.Twist()
        message.linear.x = float(vx)
        message.linear.y = float(vy)
        message.angular.z = float(wz)
        self.base_publisher.publish(message)

    def publish_base_zero(self) -> None:
        self.publish_base(0.0, 0.0, 0.0)

    def close(self) -> None:
        if not self.read_only and hasattr(self, "base_publisher"):
            for _ in range(5):
                self.publish_base_zero()
                time.sleep(0.02)
        if hasattr(self, "stop_event"):
            self.stop_event.set()
        if hasattr(self, "spin_thread"):
            self.spin_thread.join(timeout=1.0)
        if hasattr(self, "node"):
            self.node.destroy_node()
        if hasattr(self, "rclpy") and self.rclpy.ok():
            self.rclpy.shutdown()
