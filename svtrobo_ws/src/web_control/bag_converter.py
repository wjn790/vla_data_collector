#!/usr/bin/env python3
"""Convert ROS2 bag (.db3) to JSONL files (one file per topic).

Usage:
    python3 bag_converter.py <bag_dir> <output_dir> [--delete-db]
"""

import argparse
import array
import json
import sqlite3
import sys
from pathlib import Path

import yaml
from rclpy.serialization import deserialize_message
from control_msgs.msg import DynamicJointState
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Joy, Imu, MagneticField, Temperature
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Int32MultiArray, String
from chassis_control.msg import ChassisDiagnostics

MSG_TYPE_MAP = {
    'geometry_msgs/msg/Twist': Twist,
    'sensor_msgs/msg/JointState': JointState,
    'sensor_msgs/msg/Joy': Joy,
    'sensor_msgs/msg/Imu': Imu,
    'sensor_msgs/msg/MagneticField': MagneticField,
    'sensor_msgs/msg/Temperature': Temperature,
    'control_msgs/msg/DynamicJointState': DynamicJointState,
    'std_msgs/msg/Int32MultiArray': Int32MultiArray,
    'std_msgs/msg/Float32MultiArray': Float32MultiArray,
    'std_msgs/msg/Float64MultiArray': Float64MultiArray,
    'std_msgs/msg/String': String,
    'chassis_control/msg/ChassisDiagnostics': ChassisDiagnostics,
}


def _value_to_json(value):
    """Recursively convert a ROS2 message field to JSON-serializable form."""
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float, str)):
        return value
    if isinstance(value, (list, tuple)):
        return [_value_to_json(v) for v in value]
    if isinstance(value, array.array):
        return value.tolist()
    if hasattr(value, 'tolist'):
        # numpy.ndarray and similar array types
        return value.tolist()
    if hasattr(value, 'get_fields_and_field_types'):
        return msg_to_dict(value)
    if hasattr(value, 'sec') and hasattr(value, 'nanosec'):
        return value.sec + value.nanosec * 1e-9
    return str(value)


def msg_to_dict(msg):
    """Convert a ROS2 message to a JSON-serializable dict."""
    result = {}
    for field_name in msg.get_fields_and_field_types():
        result[field_name] = _value_to_json(getattr(msg, field_name))
    return result


TARGET_HZ = 10  # Downsample all topics to this frequency

def convert(bag_dir: Path, output_dir: Path, delete_db: bool = False) -> bool:
    """Convert a ROS2 bag directory to JSONL files. Returns True on success."""
    # Find db3 file
    db_files = list(bag_dir.glob('*.db3'))
    if not db_files:
        print(f"No .db3 file found in {bag_dir}", file=sys.stderr)
        return False
    db_path = db_files[0]

    conn = sqlite3.connect(str(db_path))
    cursor = conn.cursor()

    # Read topics
    cursor.execute("SELECT id, name, type FROM topics")
    topics = {}
    for topic_id, name, msg_type in cursor.fetchall():
        msg_class = MSG_TYPE_MAP.get(msg_type)
        if msg_class is None:
            print(f"Unknown type '{msg_type}' on '{name}', skipping", file=sys.stderr)
            continue
        topics[topic_id] = {'name': name, 'type': msg_type, 'msg_class': msg_class}

    if not topics:
        print("No known topics found", file=sys.stderr)
        conn.close()
        return False

    # Open JSONL writers
    writers = {}
    for topic_id, info in topics.items():
        safe_name = info['name'].strip('/').replace('/', '_')
        writers[topic_id] = open(output_dir / f'{safe_name}.jsonl', 'w')

    # Downsample: target-timestamp alignment for precise output Hz
    min_interval_ns = int(1e9 / TARGET_HZ)
    next_target = {}  # topic_id -> next desired output timestamp (ns)

    # Read and convert messages
    cursor.execute("SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp")
    count = 0
    skipped = 0
    for topic_id, timestamp, data in cursor.fetchall():
        if topic_id not in writers:
            continue
        # Downsample: pick first message >= next_target timestamp
        if topic_id in next_target:
            if timestamp < next_target[topic_id]:
                skipped += 1
                continue
            # Align next target to avoid drift
            next_target[topic_id] += min_interval_ns
            # If fell behind, jump to next valid slot
            while next_target[topic_id] < timestamp:
                next_target[topic_id] += min_interval_ns
        else:
            # First message for this topic: set target for next
            next_target[topic_id] = timestamp + min_interval_ns

        info = topics[topic_id]
        try:
            msg = deserialize_message(bytes(data), info['msg_class'])
        except Exception as e:
            print(f"Deserialize error on {info['name']}: {e}", file=sys.stderr)
            continue

        msg_dict = msg_to_dict(msg)
        msg_dict['_timestamp_ns'] = timestamp
        writers[topic_id].write(json.dumps(msg_dict, ensure_ascii=False) + '\n')
        count += 1

    for f in writers.values():
        f.close()
    conn.close()

    print(f"Converted {count} messages to {output_dir} (skipped {skipped} for {TARGET_HZ}Hz downsample)")

    if delete_db and db_path.exists():
        db_path.unlink()
        print(f"Deleted {db_path}")

    return True


def main():
    parser = argparse.ArgumentParser(description='Convert ROS2 bag (.db3) to JSONL')
    parser.add_argument('bag_dir', type=Path, help='Path to rosbag directory (containing .db3)')
    parser.add_argument('output_dir', type=Path, help='Output directory for JSONL files')
    parser.add_argument('--delete-db', action='store_true', help='Delete .db3 after conversion')
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    ok = convert(args.bag_dir, args.output_dir, args.delete_db)
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
