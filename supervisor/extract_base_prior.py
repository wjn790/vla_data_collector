#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extract only /svtrobot_cmd from a calibrated action-record rosbag."
    )
    parser.add_argument("rosbag", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--topic", default="/svtrobot_cmd")
    parser.add_argument("--start-sec", type=float, required=True)
    parser.add_argument("--end-sec", type=float, required=True)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--leading-zero-sec", type=float, default=0.5)
    parser.add_argument("--trailing-zero-sec", type=float, default=0.5)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.fps != 15:
        raise ValueError("SVT base priors must be exported at exactly 15 Hz")
    if not 0 <= args.start_sec < args.end_sec:
        raise ValueError("require 0 <= --start-sec < --end-sec")
    if args.leading_zero_sec < 0.5 or args.trailing_zero_sec < 0.5:
        raise ValueError("base priors require at least 0.5 seconds of zero command at each edge")
    bag = args.rosbag.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if not bag.is_dir():
        raise FileNotFoundError(bag)
    if output.exists():
        raise FileExistsError(f"refusing to overwrite {output}")

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic_types.get(args.topic) != "geometry_msgs/msg/Twist":
        raise ValueError(
            f"{args.topic} is absent or not geometry_msgs/msg/Twist: {topic_types.get(args.topic)}"
        )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[args.topic]))
    message_type = get_message(topic_types[args.topic])
    raw: list[tuple[int, float, float, float]] = []
    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic != args.topic:
            raise RuntimeError(f"rosbag filter leaked non-base topic {topic}")
        message = deserialize_message(data, message_type)
        raw.append(
            (
                int(timestamp_ns),
                float(message.linear.x),
                float(message.linear.y),
                float(message.angular.z),
            )
        )
    if not raw:
        raise RuntimeError(f"no messages found on {args.topic}")
    origin_ns = raw[0][0]
    selected = [
        ((timestamp_ns - origin_ns) / 1_000_000_000.0 - args.start_sec, vx, vy, wz)
        for timestamp_ns, vx, vy, wz in raw
        if args.start_sec <= (timestamp_ns - origin_ns) / 1_000_000_000.0 <= args.end_sec
    ]
    if not selected:
        raise RuntimeError("selected time window contains no base commands")
    if not all(math.isfinite(value) for row in selected for value in row):
        raise ValueError("selected base commands contain NaN or infinity")

    motion_duration = args.end_sec - args.start_sec
    total_duration = args.leading_zero_sec + motion_duration + args.trailing_zero_sec
    frame_count = math.ceil(total_duration * args.fps) + 1
    rows = []
    source_index = 0
    current = (0.0, 0.0, 0.0)
    commanded_distance = 0.0
    previous = (0.0, 0.0, 0.0)
    for index in range(frame_count):
        timestamp_sec = index / args.fps
        motion_time = timestamp_sec - args.leading_zero_sec
        if 0 <= motion_time < motion_duration:
            while source_index < len(selected) and selected[source_index][0] <= motion_time:
                current = selected[source_index][1:]
                source_index += 1
            command = current
        else:
            command = (0.0, 0.0, 0.0)
        if index:
            commanded_distance += math.hypot(previous[0], previous[1]) / args.fps
        previous = command
        rows.append(
            {
                "timestamp_sec": round(timestamp_sec, 9),
                "vx": command[0],
                "vy": command[1],
                "wz": command[2],
            }
        )
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x", encoding="utf-8") as stream:
        for row in rows:
            stream.write(json.dumps(row, separators=(",", ":")) + "\n")
    checksum = hashlib.sha256(output.read_bytes()).hexdigest()
    print(
        json.dumps(
            {
                "output": str(output),
                "sha256": checksum,
                "sample_count": len(rows),
                "duration_sec": rows[-1]["timestamp_sec"],
                "commanded_planar_distance_m": commanded_distance,
                "source_topic": args.topic,
                "source_window_sec": [args.start_sec, args.end_sec],
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"base-prior extraction failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
