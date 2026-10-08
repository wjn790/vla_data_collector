#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Any

import yaml


def emit(stream: str, value: dict[str, Any]) -> None:
    value["stream"] = stream
    value["received_ns"] = time.time_ns()
    print(json.dumps(value, separators=(",", ":")), flush=True)


def message_stamp_ns(message: Any) -> int | None:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value or None


def joint_state_dict(message: Any) -> dict[str, Any]:
    return {
        "timestamp_ns": message_stamp_ns(message),
        "name": list(message.name),
        "position": list(message.position),
        "velocity": list(message.velocity),
        "effort": list(message.effort),
    }


def twist_dict(message: Any) -> dict[str, Any]:
    return {
        "timestamp_ns": None,
        "value": [
            float(message.linear.x),
            float(message.linear.y),
            float(message.angular.z),
        ],
        "raw_twist": {
            "linear": [float(message.linear.x), float(message.linear.y), float(message.linear.z)],
            "angular": [float(message.angular.x), float(message.angular.y), float(message.angular.z)],
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Read one SVT ROS domain and emit JSONL snapshots.")
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--section", choices=("arms", "base"), required=True)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    section = config["ros"][args.section]

    # RMW reads these variables during rclpy initialization.
    os.environ["ROS_DOMAIN_ID"] = str(section["domain_id"])
    os.environ["ROS_LOCALHOST_ONLY"] = str(section["localhost_only"])

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray

    rclpy.init(args=None)
    node = rclpy.create_node(f"lingbot_collector_{args.section}_{os.getpid()}")

    if args.section == "arms":
        node.create_subscription(
            JointState,
            section["joint_state_topic"],
            lambda msg: emit("arm_joint_state", joint_state_dict(msg)),
            qos_profile_sensor_data,
        )
        node.create_subscription(
            Float64MultiArray,
            section["left_command_topic"],
            lambda msg: emit("arm_command_left", {"timestamp_ns": None, "value": list(msg.data)}),
            10,
        )
        node.create_subscription(
            Float64MultiArray,
            section["right_command_topic"],
            lambda msg: emit("arm_command_right", {"timestamp_ns": None, "value": list(msg.data)}),
            10,
        )
    else:
        node.create_subscription(
            Twist,
            section["command_topic"],
            lambda msg: emit("base_command", twist_dict(msg)),
            10,
        )
        node.create_subscription(
            Twist,
            section["feedback_topic"],
            lambda msg: emit("base_feedback", twist_dict(msg)),
            10,
        )
        node.create_subscription(
            JointState,
            section["joint_state_topic"],
            lambda msg: emit("base_joint_state", joint_state_dict(msg)),
            qos_profile_sensor_data,
        )

    emit("agent_status", {"status": "ready", "section": args.section})
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
