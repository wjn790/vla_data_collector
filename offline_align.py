#!/usr/bin/env python3
from __future__ import annotations

import argparse
import bisect
import copy
import json
import os
import statistics
from pathlib import Path
from typing import Any, Callable

import yaml

from camera_sync import sync_timestamp_ns
from common import atomic_write_json, fixed_vector, prefix_vector, reorder_joint_state
from hand_log_source import decode_hand_record


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    records: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(value, dict):
                records.append(value)
    return records


def nearest_record(
    target_ns: int,
    records: list[dict[str, Any]],
    timestamps: list[int],
) -> tuple[dict[str, Any] | None, float | None]:
    if not records:
        return None, None
    position = bisect.bisect_left(timestamps, target_ns)
    candidates = []
    if position < len(records):
        candidates.append(position)
    if position > 0:
        candidates.append(position - 1)
    index = min(candidates, key=lambda item: abs(timestamps[item] - target_ns))
    return records[index], (timestamps[index] - target_ns) / 1_000_000.0


def sorted_records(
    records: list[dict[str, Any]],
    timestamp: Callable[[dict[str, Any]], int | None],
) -> tuple[list[dict[str, Any]], list[int]]:
    values = [(int(value), record) for record in records if (value := timestamp(record))]
    values.sort(key=lambda item: item[0])
    return [item[1] for item in values], [item[0] for item in values]


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = round((len(ordered) - 1) * fraction)
    return ordered[index]


def alignment_stats(values: list[float]) -> dict[str, Any]:
    if not values:
        return {"count": 0, "median_ms": None, "p90_ms": None, "max_ms": None}
    absolute = [abs(value) for value in values]
    return {
        "count": len(absolute),
        "median_ms": statistics.median(absolute),
        "p90_ms": percentile(absolute, 0.9),
        "max_ms": max(absolute),
    }


def image_value(sample: dict[str, Any] | None) -> dict[str, Any] | None:
    if sample is None:
        return None
    return {
        "path": sample.get("path"),
        "sequence": sample.get("sequence"),
        "host_timestamp_ns": sample.get("host_timestamp_ns"),
        "hardware_timestamp_ns": sample.get("hardware_timestamp_ns"),
        "hardware_timestamp_domain": sample.get("hardware_timestamp_domain"),
        "sync_timestamp_ns": sync_timestamp_ns(sample),
        "width": sample.get("width"),
        "height": sample.get("height"),
    }


def refresh_paired_segment_indices(
    episode_dir: Path,
    frames: list[dict[str, Any]],
) -> dict[str, Any]:
    path = episode_dir / "paired_segments.json"
    if not path.is_file():
        return {"present": False, "updated": False, "segment_count": 0}

    document = json.loads(path.read_text(encoding="utf-8"))
    segments = document.get("segments")
    if not isinstance(segments, list) or not segments:
        raise ValueError("paired_segments.json must contain at least one segment")

    summary_segments = []
    assignments: dict[int, dict[str, Any]] = {}
    for segment_index, segment in enumerate(segments):
        if not isinstance(segment, dict):
            raise ValueError("paired_segments.json segments must be mappings")
        skill_id = str(segment.get("skill_id") or "")
        task = str(segment.get("task") or "").strip()
        prompt_version = str(segment.get("prompt_version") or "").strip()
        if not skill_id or not task or not prompt_version:
            raise ValueError(
                "paired segment must define skill_id, task, and prompt_version"
            )
        start_ns = int(segment["start_ns"])
        end_ns = int(segment["end_ns"])
        start_inclusive = bool(segment.get("start_inclusive", True))
        end_inclusive = bool(segment.get("end_inclusive", False))
        indices = []
        for frame in frames:
            timestamp_ns = int(frame["timestamp_ns"])
            after_start = timestamp_ns >= start_ns if start_inclusive else timestamp_ns > start_ns
            before_end = timestamp_ns <= end_ns if end_inclusive else timestamp_ns < end_ns
            if after_start and before_end:
                frame_index = int(frame["frame_index"])
                if frame_index in assignments:
                    raise ValueError(
                        f"paired segment {skill_id} overlaps frame {frame_index}"
                    )
                indices.append(frame_index)
                assignments[frame_index] = {
                    "segment_index": segment_index,
                    "skill_id": skill_id,
                    "task": task,
                    "prompt_version": prompt_version,
                }
        if not indices:
            raise ValueError(f"paired segment {skill_id or '<unknown>'} has no offline-aligned frames")
        if indices != list(range(indices[0], indices[-1] + 1)):
            raise ValueError(f"paired segment {skill_id or '<unknown>'} is not contiguous")

        if "online_source_start_frame" not in segment and "source_start_frame" in segment:
            segment["online_source_start_frame"] = int(segment["source_start_frame"])
        if "online_source_end_frame" not in segment and "source_end_frame" in segment:
            segment["online_source_end_frame"] = int(segment["source_end_frame"])
        if "online_frame_count" not in segment and "frame_count" in segment:
            segment["online_frame_count"] = int(segment["frame_count"])
        segment["source_start_frame"] = indices[0]
        segment["source_end_frame"] = indices[-1]
        segment["frame_count"] = len(indices)
        summary_segments.append(
            {
                "segment_index": segment_index,
                "skill_id": skill_id,
                "task": task,
                "prompt_version": prompt_version,
                "source_start_frame": indices[0],
                "source_end_frame": indices[-1],
                "frame_count": len(indices),
            }
        )

    missing = sorted(
        int(frame["frame_index"])
        for frame in frames
        if int(frame["frame_index"]) not in assignments
    )
    if missing:
        raise ValueError(
            "paired segments do not cover every offline-aligned frame: "
            f"first missing indices={missing[:10]}"
        )

    relabeled_frame_count = 0
    for frame in frames:
        frame_index = int(frame["frame_index"])
        assignment = assignments[frame_index]
        relabeled_frame_count += int(frame.get("task") != assignment["task"])
        frame["task"] = assignment["task"]
        frame["skill_id"] = assignment["skill_id"]
        frame["prompt_version"] = assignment["prompt_version"]
        frame["paired_segment"] = {
            "source": path.name,
            "segment_index": assignment["segment_index"],
            "skill_id": assignment["skill_id"],
        }

    document["source_frame_index_basis"] = "offline_aligned_frames_v1"
    document["offline_aligned_source_frame_count"] = len(frames)
    document["frame_metadata_applied"] = True
    document["frame_metadata_source"] = path.name
    atomic_write_json(path, document)
    return {
        "present": True,
        "updated": True,
        "segment_count": len(segments),
        "covered_frame_count": len(assignments),
        "relabeled_frame_count": relabeled_frame_count,
        "segments": summary_segments,
    }


def source_timestamp(record: dict[str, Any]) -> int | None:
    value = record.get("received_ns")
    return int(value) if value else None


def hand_timestamp(record: dict[str, Any]) -> int | None:
    value = record.get("_timestamp_ns")
    return int(value) if value else None


def hand_actual_timestamp(record: dict[str, Any]) -> int | None:
    value = record.get("_actual_state_timestamp_ns")
    return int(value) if value else None


def _select(
    streams: dict[str, tuple[list[dict[str, Any]], list[int]]],
    name: str,
    target_ns: int,
    deltas: dict[str, list[float]],
) -> tuple[dict[str, Any] | None, float | None]:
    records, timestamps = streams.get(name, ([], []))
    record, delta_ms = nearest_record(target_ns, records, timestamps)
    if delta_ms is not None:
        deltas.setdefault(name, []).append(delta_ms)
    return record, delta_ms


def _within(delta_ms: float | None, limit_ms: float) -> bool:
    return delta_ms is not None and abs(delta_ms) <= limit_ms


def _interpolate_vector(
    before: Any,
    after: Any,
    alpha: float,
    size: int,
) -> list[float] | None:
    left = fixed_vector(before, size)
    right = fixed_vector(after, size)
    if left is None or right is None:
        return None
    return [left[index] + (right[index] - left[index]) * alpha for index in range(size)]


def _max_vector_delta(before: Any, after: Any, size: int) -> float | None:
    left = fixed_vector(before, size)
    right = fixed_vector(after, size)
    if left is None or right is None:
        return None
    return max(abs(right[index] - left[index]) for index in range(size))


def _max_hand_partition_delta(before: Any, after: Any) -> dict[str, float] | None:
    left = fixed_vector(before, 12)
    right = fixed_vector(after, 12)
    if left is None or right is None:
        return None
    return {
        "o6": max(abs(right[index] - left[index]) for index in range(6)),
        "wuji": max(abs(right[index] - left[index]) for index in range(6, 12)),
    }


def _numeric_gap_source(frame: dict[str, Any]) -> str | None:
    validity = frame.get("validity") or {}
    cameras = validity.get("cameras_aligned") or {}
    if not cameras or not all(bool(value) for value in cameras.values()):
        return None
    if not validity.get("wrist_pair_valid"):
        return None
    if not validity.get("arm_aligned") and validity.get("hand_aligned"):
        return "arm"
    if validity.get("arm_aligned") and not validity.get("hand_aligned"):
        return "hand"
    return None


def _interpolate_numeric_frame(
    frame: dict[str, Any],
    before: dict[str, Any],
    after: dict[str, Any],
    alpha: float,
    source: str,
    *,
    repair_state: bool = True,
    repair_action: bool = True,
) -> list[str] | None:
    components: list[str] = []
    state = frame.setdefault("observation", {}).setdefault("state", {})
    action = frame.setdefault("action", {})
    if source == "arm":
        if repair_state:
            for name in ("arm_position", "arm_velocity", "arm_effort"):
                value = _interpolate_vector(
                    before["observation"]["state"].get(name),
                    after["observation"]["state"].get(name),
                    alpha,
                    14,
                )
                if value is None:
                    return None
                state[name] = value
                components.append(f"observation.state.{name}")
        if not repair_action:
            return components
        value = _interpolate_vector(
            before["action"].get("arm_position"),
            after["action"].get("arm_position"),
            alpha,
            14,
        )
        if value is None:
            return None
        action["arm_position"] = value
        frame.setdefault("source", {})["arm_action"] = "interpolated_controller_command"
        components.append("action.arm_position")
        return components
    if source == "hand":
        if repair_state:
            value = _interpolate_vector(
                before["observation"]["state"].get("hand_position"),
                after["observation"]["state"].get("hand_position"),
                alpha,
                12,
            )
            if value is None:
                return None
            state["hand_position"] = value
            frame.setdefault("source", {})["hand_state"] = "interpolated_hardware_feedback"
            components.append("observation.state.hand_position")
        if not repair_action:
            return components
        action_value = _interpolate_vector(
            before["action"].get("hand_position"),
            after["action"].get("hand_position"),
            alpha,
            12,
        )
        if action_value is None:
            return None
        action["hand_position"] = action_value
        components.append("action.hand_position")
        return components
    return None


def repair_isolated_frames(
    frames: list[dict[str, Any]],
    repair_config: dict[str, Any],
    *,
    output_fps: float,
    arm_limit_ms: float,
    hand_limit_ms: float,
    wrist_left_camera: str = "camera_wrist_left",
    wrist_right_camera: str = "camera_wrist_right",
    wrist_pair_limit_ms: float = 30.0,
) -> dict[str, Any]:
    enabled = bool(repair_config.get("enabled", True))
    max_delta_ms = float(repair_config.get("max_delta_ms", 250.0))
    max_neighbor_span_ms = float(
        repair_config.get("max_neighbor_span_ms", 450.0)
    )
    max_consecutive_frames = max(1, int(repair_config.get("max_consecutive_frames", 5)))
    camera_max_delta_ms = float(repair_config.get("camera_max_delta_ms", 75.0))
    camera_max_consecutive_frames = max(
        0, int(repair_config.get("camera_max_consecutive_frames", 1))
    )
    wrist_pair_max_consecutive_frames = max(
        0, int(repair_config.get("wrist_pair_max_consecutive_frames", 0))
    )
    wrist_pair_camera_max_delta_ms = float(
        repair_config.get("wrist_pair_camera_max_delta_ms", camera_max_delta_ms)
    )
    arm_state_endpoint_max_delta = float(
        repair_config.get("arm_state_endpoint_max_delta_rad", 0.15)
    )
    arm_action_endpoint_max_delta = float(
        repair_config.get("arm_action_endpoint_max_delta_rad", 0.15)
    )
    hand_state_o6_endpoint_max_delta = float(
        repair_config.get("hand_state_o6_endpoint_max_delta", 64.0)
    )
    hand_state_wuji_endpoint_max_delta = float(
        repair_config.get("hand_state_wuji_endpoint_max_delta_rad", 0.15)
    )
    hand_action_o6_endpoint_max_delta = float(
        repair_config.get("hand_action_o6_endpoint_max_delta", 64.0)
    )
    hand_action_wuji_endpoint_max_delta = float(
        repair_config.get("hand_action_wuji_endpoint_max_delta_rad", 0.15)
    )
    max_missing_timing_frames = max(
        0, int(repair_config.get("max_missing_timing_frames", 1))
    )
    max_timing_gap_ms = float(repair_config.get("max_timing_gap_ms", 160.0))
    timing_jitter_tolerance_ms = float(
        repair_config.get("timing_jitter_tolerance_ms", 20.0)
    )
    timing_camera_max_delta_ms = float(
        repair_config.get("timing_camera_max_delta_ms", 90.0)
    )
    trim_invalid_edges = bool(repair_config.get("trim_invalid_edges", False))
    summary = {
        "enabled": enabled,
        "policy": "bounded_short_gap_repair_with_provenance",
        "max_delta_ms": max_delta_ms,
        "max_neighbor_span_ms": max_neighbor_span_ms,
        "max_consecutive_frames": max_consecutive_frames,
        "camera_max_delta_ms": camera_max_delta_ms,
        "camera_max_consecutive_frames": camera_max_consecutive_frames,
        "wrist_pair_max_consecutive_frames": wrist_pair_max_consecutive_frames,
        "wrist_pair_camera_max_delta_ms": wrist_pair_camera_max_delta_ms,
        "max_missing_timing_frames": max_missing_timing_frames,
        "timing_camera_max_delta_ms": timing_camera_max_delta_ms,
        "repaired_frame_count": 0,
        "inserted_frame_count": 0,
        "trimmed_leading_frame_count": 0,
        "trimmed_trailing_frame_count": 0,
        "repaired_by_source": {
            "arm": 0,
            "hand": 0,
            "camera": 0,
            "wrist_pair": 0,
            "timing": 0,
        },
    }

    for frame in frames:
        validity = frame.setdefault("validity", {})
        validity["repaired"] = False
        validity["repair_sources"] = []
        frame["repair"] = {"applied": False, "sources": {}}

    if not enabled or len(frames) < 2:
        return summary

    # Repair each camera independently. Camera and numeric streams can miss the
    # same target frame, so neither component is allowed to gate the other.
    if camera_max_consecutive_frames >= 1:
        camera_names = sorted(
            {
                name
                for frame in frames
                for name in ((frame.get("validity") or {}).get("cameras_aligned") or {})
            }
        )
        for camera in camera_names:
            component_valid = [
                bool(((frame.get("validity") or {}).get("cameras_aligned") or {}).get(camera))
                for frame in frames
            ]
            run_start = 1
            while run_start < len(frames) - 1:
                if component_valid[run_start]:
                    run_start += 1
                    continue
                run_end = run_start
                while run_end + 1 < len(frames) and not component_valid[run_end + 1]:
                    run_end += 1
                run_length = run_end - run_start + 1
                bounded = (
                    run_end < len(frames) - 1
                    and run_length <= camera_max_consecutive_frames
                    and component_valid[run_start - 1]
                    and component_valid[run_end + 1]
                )
                if not bounded:
                    run_start = run_end + 1
                    continue

                staged: list[tuple[int, dict[str, Any], float | None, float, str]] = []
                for index in range(run_start, run_end + 1):
                    frame = copy.deepcopy(frames[index])
                    target_ns = int(frame["timestamp_ns"])
                    images = frame.setdefault("observation", {}).setdefault("images", {})
                    deltas = frame.setdefault("alignment_delta_ms", {}).setdefault("cameras", {})
                    original_delta = deltas.get(camera)
                    image = images.get(camera)
                    method = (
                        "accept_isolated_nearest_raw_image"
                        if run_length == 1
                        else "accept_bounded_nearest_raw_images"
                    )
                    selected_delta = original_delta
                    if (
                        not isinstance(image, dict)
                        or not image.get("path")
                        or not isinstance(selected_delta, (int, float))
                        or abs(float(selected_delta)) > camera_max_delta_ms
                    ):
                        candidates = []
                        for endpoint_index in (run_start - 1, run_end + 1):
                            endpoint_image = (
                                ((frames[endpoint_index].get("observation") or {}).get("images") or {})
                                .get(camera)
                            )
                            if not isinstance(endpoint_image, dict) or not endpoint_image.get("path"):
                                continue
                            source_ns = endpoint_image.get("sync_timestamp_ns")
                            if not isinstance(source_ns, int):
                                continue
                            delta = (source_ns - target_ns) / 1_000_000.0
                            if abs(delta) <= camera_max_delta_ms:
                                candidates.append((abs(delta), delta, endpoint_image, endpoint_index))
                        if not candidates:
                            staged = []
                            break
                        _, selected_delta, image, endpoint_index = min(
                            candidates, key=lambda value: value[0]
                        )
                        images[camera] = copy.deepcopy(image)
                        method = "reuse_bounded_neighbor_image"
                    else:
                        endpoint_index = None
                    deltas[camera] = float(selected_delta)
                    staged.append(
                        (
                            index,
                            frame,
                            float(original_delta)
                            if isinstance(original_delta, (int, float))
                            else None,
                            float(selected_delta),
                            method,
                        )
                    )

                if len(staged) == run_length:
                    for index, frame, original_delta, selected_delta, method in staged:
                        validity = frame["validity"]
                        validity["cameras_aligned"][camera] = True
                        validity["repaired"] = True
                        repair = frame.setdefault("repair", {"applied": False, "sources": {}})
                        repair["applied"] = True
                        repair.setdefault("sources", {})[f"camera:{camera}"] = {
                            "method": method,
                            "run_length": run_length,
                            "original_alignment_delta_ms": original_delta,
                            "selected_alignment_delta_ms": selected_delta,
                            "selected_image": frame["observation"]["images"][camera].get("path"),
                        }
                        validity["repair_sources"] = sorted(repair["sources"])
                        frames[index] = frame
                        summary["repaired_by_source"]["camera"] += 1
                run_start = run_end + 1

    # For a short wrist-pair dropout, reuse both wrist images from one valid
    # neighbor. This preserves the strict pair-skew limit and avoids mixing
    # independently selected exposures.
    if wrist_pair_max_consecutive_frames >= 1:
        pair_valid = [bool(frame["validity"].get("wrist_pair_valid")) for frame in frames]
        run_start = 1
        while run_start < len(frames) - 1:
            if pair_valid[run_start]:
                run_start += 1
                continue
            run_end = run_start
            while run_end + 1 < len(frames) and not pair_valid[run_end + 1]:
                run_end += 1
            run_length = run_end - run_start + 1
            bounded = (
                run_end < len(frames) - 1
                and run_length <= wrist_pair_max_consecutive_frames
                and pair_valid[run_start - 1]
                and pair_valid[run_end + 1]
            )
            if not bounded:
                run_start = run_end + 1
                continue
            staged: list[tuple[int, dict[str, Any], int]] = []
            for index in range(run_start, run_end + 1):
                target_ns = int(frames[index]["timestamp_ns"])
                candidates = []
                for endpoint_index in (run_start - 1, run_end + 1):
                    endpoint = frames[endpoint_index]
                    images = (endpoint.get("observation") or {}).get("images") or {}
                    left_image = images.get(wrist_left_camera)
                    right_image = images.get(wrist_right_camera)
                    if not isinstance(left_image, dict) or not isinstance(right_image, dict):
                        continue
                    left_ns = left_image.get("sync_timestamp_ns")
                    right_ns = right_image.get("sync_timestamp_ns")
                    if not isinstance(left_ns, int) or not isinstance(right_ns, int):
                        continue
                    left_delta = (left_ns - target_ns) / 1_000_000.0
                    right_delta = (right_ns - target_ns) / 1_000_000.0
                    skew_ms = abs(left_ns - right_ns) / 1_000_000.0
                    max_delta = max(abs(left_delta), abs(right_delta))
                    if (
                        max_delta <= wrist_pair_camera_max_delta_ms
                        and skew_ms <= wrist_pair_limit_ms
                    ):
                        candidates.append(
                            (max_delta, endpoint_index, left_image, right_image, left_delta, right_delta, skew_ms)
                        )
                if not candidates:
                    staged = []
                    break
                _, endpoint_index, left_image, right_image, left_delta, right_delta, skew_ms = min(
                    candidates, key=lambda value: value[0]
                )
                frame = copy.deepcopy(frames[index])
                images = frame["observation"]["images"]
                images[wrist_left_camera] = copy.deepcopy(left_image)
                images[wrist_right_camera] = copy.deepcopy(right_image)
                deltas = frame["alignment_delta_ms"]["cameras"]
                deltas[wrist_left_camera] = left_delta
                deltas[wrist_right_camera] = right_delta
                validity = frame["validity"]
                validity["cameras_aligned"][wrist_left_camera] = True
                validity["cameras_aligned"][wrist_right_camera] = True
                validity["wrist_pair_valid"] = True
                validity["wrist_pair_skew_ms"] = skew_ms
                validity["wrist_pair_reason"] = "reused_synchronized_neighbor_pair"
                validity["repaired"] = True
                repair = frame.setdefault("repair", {"applied": False, "sources": {}})
                repair["applied"] = True
                repair.setdefault("sources", {})["wrist_pair"] = {
                    "method": "reuse_bounded_synchronized_neighbor_pair",
                    "run_length": run_length,
                    "source_frame_index": int(frames[endpoint_index]["frame_index"]),
                    "original_skew_ms": frames[index]["validity"].get("wrist_pair_skew_ms"),
                    "selected_skew_ms": skew_ms,
                    "camera_alignment_delta_ms": {
                        wrist_left_camera: left_delta,
                        wrist_right_camera: right_delta,
                    },
                }
                validity["repair_sources"] = sorted(repair["sources"])
                staged.append((index, frame, endpoint_index))
            if len(staged) == run_length:
                for index, frame, _ in staged:
                    frames[index] = frame
                    summary["repaired_by_source"]["wrist_pair"] += 1
            run_start = run_end + 1

    # Repair arm and hand independently. A frame may have an arm-state gap and
    # a hand-state gap at the same time; valid command components are preserved.
    for source in ("arm", "hand"):
        aligned_key = f"{source}_aligned"
        run_start = 1
        while run_start < len(frames) - 1:
            if frames[run_start]["validity"].get(aligned_key):
                run_start += 1
                continue
            run_end = run_start
            while run_end + 1 < len(frames) and not frames[run_end + 1]["validity"].get(aligned_key):
                run_end += 1
            run_length = run_end - run_start + 1
            if (
                run_end >= len(frames) - 1
                or run_length > max_consecutive_frames
                or not frames[run_start - 1]["validity"].get(aligned_key)
                or not frames[run_end + 1]["validity"].get(aligned_key)
            ):
                run_start = run_end + 1
                continue

            run_frames = frames[run_start : run_end + 1]

            before = frames[run_start - 1]
            after = frames[run_end + 1]
            before_ns = int(before["timestamp_ns"])
            after_ns = int(after["timestamp_ns"])
            span_ms = (after_ns - before_ns) / 1_000_000.0
            if span_ms <= 0.0 or span_ms > max_neighbor_span_ms:
                run_start = run_end + 1
                continue

            component_flags: list[tuple[bool, bool]] = []
            selected_deltas: list[float | None] = []
            for frame in run_frames:
                deltas = frame.get("alignment_delta_ms") or {}
                if source == "arm":
                    repair_state = not _within(deltas.get("arm"), arm_limit_ms)
                    repair_action = (
                        frame.get("source", {}).get("arm_action") != "controller_command"
                        or not _within(deltas.get("arm_command_left"), arm_limit_ms)
                        or not _within(deltas.get("arm_command_right"), arm_limit_ms)
                    )
                    if repair_state:
                        selected_deltas.append(deltas.get("arm"))
                    if repair_action:
                        selected_deltas.extend(
                            [deltas.get("arm_command_left"), deltas.get("arm_command_right")]
                        )
                else:
                    repair_state = (
                        frame.get("source", {}).get("hand_state") != "hardware_feedback"
                        or not _within(deltas.get("hand_state"), hand_limit_ms)
                    )
                    repair_action = not _within(deltas.get("hand_action"), hand_limit_ms)
                    if repair_state:
                        selected_deltas.append(deltas.get("hand_state", deltas.get("hand")))
                    if repair_action:
                        selected_deltas.append(deltas.get("hand_action", deltas.get("hand")))
                component_flags.append((repair_state, repair_action))

            needs_state = any(value[0] for value in component_flags)
            needs_action = any(value[1] for value in component_flags)
            if (
                not (needs_state or needs_action)
                or not selected_deltas
                or not all(isinstance(value, (int, float)) for value in selected_deltas)
                or max(abs(float(value)) for value in selected_deltas) > max_delta_ms
            ):
                run_start = run_end + 1
                continue

            if source == "arm":
                state_endpoint_delta: Any = _max_vector_delta(
                    before["observation"]["state"].get("arm_position"),
                    after["observation"]["state"].get("arm_position"),
                    14,
                )
                action_endpoint_delta: Any = _max_vector_delta(
                    before["action"].get("arm_position"),
                    after["action"].get("arm_position"),
                    14,
                )
                endpoints_valid = (
                    (not needs_state or (
                        state_endpoint_delta is not None
                        and state_endpoint_delta <= arm_state_endpoint_max_delta
                    ))
                    and (not needs_action or (
                        action_endpoint_delta is not None
                        and action_endpoint_delta <= arm_action_endpoint_max_delta
                    ))
                )
                delta_names = ("arm", "arm_command_left", "arm_command_right")
            else:
                state_endpoint_delta = _max_hand_partition_delta(
                    before["observation"]["state"].get("hand_position"),
                    after["observation"]["state"].get("hand_position"),
                )
                action_endpoint_delta = _max_hand_partition_delta(
                    before["action"].get("hand_position"),
                    after["action"].get("hand_position"),
                )
                endpoints_valid = (
                    (not needs_state or (
                        state_endpoint_delta is not None
                        and state_endpoint_delta["o6"] <= hand_state_o6_endpoint_max_delta
                        and state_endpoint_delta["wuji"] <= hand_state_wuji_endpoint_max_delta
                    ))
                    and (not needs_action or (
                        action_endpoint_delta is not None
                        and action_endpoint_delta["o6"] <= hand_action_o6_endpoint_max_delta
                        and action_endpoint_delta["wuji"] <= hand_action_wuji_endpoint_max_delta
                    ))
                )
                delta_names = ("hand_action", "hand_state", "hand")
            if not endpoints_valid:
                run_start = run_end + 1
                continue

            staged: list[tuple[int, dict[str, Any]]] = []
            for offset, index in enumerate(range(run_start, run_end + 1)):
                frame = copy.deepcopy(frames[index])
                target_ns = int(frame["timestamp_ns"])
                alpha = (target_ns - before_ns) / (after_ns - before_ns)
                repair_state, repair_action = component_flags[offset]
                components = _interpolate_numeric_frame(
                    frame,
                    before,
                    after,
                    alpha,
                    source,
                    repair_state=repair_state,
                    repair_action=repair_action,
                )
                if not components:
                    staged = []
                    break
                validity = frame["validity"]
                validity[aligned_key] = True
                validity["repaired"] = True
                repair = frame.setdefault("repair", {"applied": False, "sources": {}})
                repair["applied"] = True
                repair.setdefault("sources", {})[source] = {
                    "method": "bounded_component_linear_interpolation",
                    "previous_frame_index": int(before["frame_index"]),
                    "next_frame_index": int(after["frame_index"]),
                    "previous_timestamp_ns": before_ns,
                    "next_timestamp_ns": after_ns,
                    "alpha": alpha,
                    "components": components,
                    "endpoint_max_delta": {
                        "state": state_endpoint_delta,
                        "action": action_endpoint_delta,
                    },
                    "original_alignment_delta_ms": {
                        name: (frame.get("alignment_delta_ms") or {}).get(name)
                        for name in delta_names
                    },
                }
                validity["repair_sources"] = sorted(repair["sources"])
                staged.append((index, frame))
            if len(staged) == run_length:
                for index, frame in staged:
                    frames[index] = frame
                    summary["repaired_by_source"][source] += 1
            run_start = run_end + 1

    for frame in frames:
        validity = frame["validity"]
        cameras = validity.get("cameras_aligned") or {}
        validity["valid_for_training"] = (
            bool(cameras)
            and all(bool(value) for value in cameras.values())
            and bool(validity.get("wrist_pair_valid"))
            and bool(validity.get("arm_aligned"))
            and bool(validity.get("hand_aligned"))
        )
    summary["repaired_frame_count"] = sum(
        int(bool(frame["validity"].get("repaired"))) for frame in frames
    )

    # Restore one dropped 15 Hz master-camera timestep. Numeric values are
    # interpolated; each camera references the closest existing raw image.
    period_ns = round(1_000_000_000.0 / output_fps)
    expanded: list[dict[str, Any]] = []
    for index, before in enumerate(frames[:-1]):
        expanded.append(before)
        after = frames[index + 1]
        gap_ns = int(after["timestamp_ns"]) - int(before["timestamp_ns"])
        missing_count = max(0, round(gap_ns / period_ns) - 1)
        expected_gap_ns = (missing_count + 1) * period_ns
        if (
            missing_count < 1
            or missing_count > max_missing_timing_frames
            or gap_ns / 1_000_000.0 > max_timing_gap_ms
            or abs(gap_ns - expected_gap_ns) / 1_000_000.0 > timing_jitter_tolerance_ms
            or not before["validity"].get("valid_for_training")
            or not after["validity"].get("valid_for_training")
        ):
            continue
        staged_insertions: list[dict[str, Any]] = []
        for ordinal in range(1, missing_count + 1):
            target_ns = int(before["timestamp_ns"]) + ordinal * period_ns
            alpha = (target_ns - int(before["timestamp_ns"])) / gap_ns
            frame = copy.deepcopy(before if alpha <= 0.5 else after)
            frame["timestamp_ns"] = target_ns
            components = []
            for source in ("arm", "hand"):
                updated = _interpolate_numeric_frame(frame, before, after, alpha, source)
                if updated is None:
                    components = []
                    break
                components.extend(updated)
            if not components:
                staged_insertions = []
                break
            for container, name, size in (
                (frame["observation"]["state"], "base_velocity", 3),
                (frame["action"], "base_velocity", 3),
            ):
                value = _interpolate_vector(
                    before["observation"]["state"].get(name)
                    if container is frame["observation"]["state"]
                    else before["action"].get(name),
                    after["observation"]["state"].get(name)
                    if container is frame["observation"]["state"]
                    else after["action"].get(name),
                    alpha,
                    size,
                )
                if value is None:
                    components = []
                    break
                container[name] = value
                components.append(
                    f"observation.state.{name}"
                    if container is frame["observation"]["state"]
                    else f"action.{name}"
                )
            if not components:
                staged_insertions = []
                break
            # Reuse a complete synchronized camera group from one endpoint.
            # Mixing independently nearest cameras can destroy wrist-pair sync.
            camera_group_candidates = []
            for endpoint in (before, after):
                endpoint_images = endpoint["observation"]["images"]
                if not endpoint_images or not endpoint["validity"].get("wrist_pair_valid"):
                    continue
                endpoint_deltas = {
                    camera: (int(image["sync_timestamp_ns"]) - target_ns) / 1_000_000.0
                    for camera, image in endpoint_images.items()
                    if image and image.get("sync_timestamp_ns")
                }
                if len(endpoint_deltas) != len(endpoint_images):
                    continue
                max_camera_delta = max(abs(value) for value in endpoint_deltas.values())
                if max_camera_delta <= timing_camera_max_delta_ms:
                    camera_group_candidates.append(
                        (max_camera_delta, endpoint, endpoint_deltas)
                    )
            if not camera_group_candidates:
                staged_insertions = []
                break
            _, selected_endpoint, camera_deltas = min(
                camera_group_candidates, key=lambda value: value[0]
            )
            images = copy.deepcopy(selected_endpoint["observation"]["images"])
            frame["observation"]["images"] = images
            wrist_skew_ms = float(
                selected_endpoint["validity"].get("wrist_pair_skew_ms") or 0.0
            )
            frame["alignment_delta_ms"] = {
                **(frame.get("alignment_delta_ms") or {}),
                "cameras": camera_deltas,
                "arm": None,
                "arm_command_left": None,
                "arm_command_right": None,
                "hand": None,
                "hand_action": None,
                "hand_state": None,
            }
            frame["source_timestamps_ns"] = {
                **(frame.get("source_timestamps_ns") or {}),
                "arm": None,
                "arm_command_left": None,
                "arm_command_right": None,
                "hand_action": None,
                "hand_actual": None,
            }
            validity = frame["validity"]
            validity["valid_for_training"] = True
            validity["offline_aligned"] = True
            validity["arm_aligned"] = True
            validity["hand_aligned"] = True
            validity["cameras_aligned"] = {name: True for name in images}
            validity["wrist_pair_valid"] = True
            validity["wrist_pair_skew_ms"] = wrist_skew_ms
            validity["wrist_pair_reason"] = "nearest_raw_images_for_restored_timestep"
            validity["repaired"] = True
            validity["repair_sources"] = ["timing"]
            frame.setdefault("raw", {})["timing_repair_from_frame_indices"] = [
                int(before["frame_index"]), int(after["frame_index"])
            ]
            frame["repair"] = {
                "applied": True,
                "sources": {
                    "timing": {
                        "method": "missing_timestep_interpolation_nearest_raw_images",
                        "previous_frame_index": int(before["frame_index"]),
                        "next_frame_index": int(after["frame_index"]),
                        "previous_timestamp_ns": int(before["timestamp_ns"]),
                        "next_timestamp_ns": int(after["timestamp_ns"]),
                        "alpha": alpha,
                        "components": components,
                        "camera_alignment_delta_ms": camera_deltas,
                    }
                },
            }
            staged_insertions.append(frame)
        if len(staged_insertions) == missing_count:
            expanded.extend(staged_insertions)
            summary["inserted_frame_count"] += missing_count
            summary["repaired_frame_count"] += missing_count
            summary["repaired_by_source"]["timing"] += missing_count
    expanded.append(frames[-1])
    frames[:] = expanded
    if trim_invalid_edges and frames:
        valid_flags = [bool(frame["validity"].get("valid_for_training")) for frame in frames]
        if any(valid_flags):
            first_valid = valid_flags.index(True)
            last_valid = len(valid_flags) - 1 - valid_flags[::-1].index(True)
            summary["trimmed_leading_frame_count"] = first_valid
            summary["trimmed_trailing_frame_count"] = len(frames) - 1 - last_valid
            frames[:] = frames[first_valid : last_valid + 1]
    for index, frame in enumerate(frames):
        frame["frame_index"] = index

    return summary


def write_lerobot_staging(
    episode_dir: Path,
    frames: list[dict[str, Any]],
    staging_config: dict[str, Any],
    *,
    output_fps: float,
) -> dict[str, Any]:
    raise RuntimeError(
        "training conversion is not part of acquisition alignment; "
        "keep the episode continuous and convert it in a separate step"
    )
    enabled = bool(staging_config.get("enabled", True))
    min_segment_frames = int(staging_config.get("min_segment_frames", 50))
    max_frame_gap_ms = float(
        staging_config.get("max_frame_gap_ms", 1.275 * 1000.0 / output_fps)
    )
    required_cameras = list(
        staging_config.get(
            "required_cameras",
            ["camera_top", "camera_wrist_left", "camera_wrist_right"],
        )
    )
    require_hardware_hand_state = bool(
        staging_config.get("require_hardware_hand_state", True)
    )
    output_path = episode_dir / "lerobot_frames.jsonl"
    temporary = output_path.with_suffix(".jsonl.tmp")

    for frame in frames:
        validity = frame.setdefault("validity", {})
        validity["eligible_for_lerobot"] = False
        validity["lerobot_segment_index"] = None
        validity["lerobot_frame_index"] = None

    summary = {
        "enabled": enabled,
        "path": output_path.name,
        "fps": output_fps,
        "min_segment_frames": min_segment_frames,
        "max_frame_gap_ms": max_frame_gap_ms,
        "required_cameras": required_cameras,
        "require_hardware_hand_state": require_hardware_hand_state,
        "candidate_frame_count": len(frames),
        "aligned_eligible_frame_count": 0,
        "exported_frame_count": 0,
        "dropped_invalid_frame_count": 0,
        "dropped_short_segment_frame_count": 0,
        "timing_break_count": 0,
        "segment_count": 0,
        "segments": [],
        "features": {
            "observation.state.arm.position": 14,
            "observation.state.hand.position": 12,
            "action.arm.position": 14,
            "action.hand.position": 12,
            "images": [f"observation.images.{name}" for name in required_cameras],
        },
    }
    if not enabled:
        if output_path.exists():
            output_path.unlink()
        return summary
    if min_segment_frames < 1:
        raise ValueError("offline_alignment.lerobot.min_segment_frames must be >= 1")
    if max_frame_gap_ms <= 0:
        raise ValueError("offline_alignment.lerobot.max_frame_gap_ms must be > 0")

    def eligible(frame: dict[str, Any]) -> bool:
        validity = frame.get("validity") or {}
        if not validity.get("valid_for_training"):
            return False
        if require_hardware_hand_state and (frame.get("source") or {}).get("hand_state") not in {
            "hardware_feedback",
            "interpolated_hardware_feedback",
        }:
            return False
        state = ((frame.get("observation") or {}).get("state") or {})
        action = frame.get("action") or {}
        if fixed_vector(state.get("arm_position"), 14) is None:
            return False
        if fixed_vector(state.get("hand_position"), 12) is None:
            return False
        if fixed_vector(action.get("arm_position"), 14) is None:
            return False
        if fixed_vector(action.get("hand_position"), 12) is None:
            return False
        images = ((frame.get("observation") or {}).get("images") or {})
        return all(
            isinstance(images.get(camera), dict) and bool(images[camera].get("path"))
            for camera in required_cameras
        )

    runs: list[list[dict[str, Any]]] = []
    current: list[dict[str, Any]] = []
    for frame in frames:
        if not eligible(frame):
            if current:
                runs.append(current)
                current = []
            continue
        if current:
            gap_ms = (int(frame["timestamp_ns"]) - int(current[-1]["timestamp_ns"])) / 1_000_000.0
            if gap_ms > max_frame_gap_ms:
                runs.append(current)
                current = []
                summary["timing_break_count"] += 1
        current.append(frame)
    if current:
        runs.append(current)

    summary["aligned_eligible_frame_count"] = sum(len(run) for run in runs)
    summary["dropped_invalid_frame_count"] = (
        len(frames) - summary["aligned_eligible_frame_count"]
    )
    retained = [run for run in runs if len(run) >= min_segment_frames]
    summary["dropped_short_segment_frame_count"] = sum(
        len(run) for run in runs if len(run) < min_segment_frames
    )

    with temporary.open("w", encoding="utf-8", buffering=1) as output:
        for segment_index, run in enumerate(retained):
            source_start_ns = int(run[0]["timestamp_ns"])
            for lerobot_frame_index, frame in enumerate(run):
                frame["validity"]["eligible_for_lerobot"] = True
                frame["validity"]["lerobot_segment_index"] = segment_index
                frame["validity"]["lerobot_frame_index"] = lerobot_frame_index
                images = frame["observation"]["images"]
                record = {
                    "schema_version": 1,
                    "episode_index": segment_index,
                    "frame_index": lerobot_frame_index,
                    "timestamp": lerobot_frame_index / output_fps,
                    "source_frame_index": int(frame["frame_index"]),
                    "source_timestamp_ns": int(frame["timestamp_ns"]),
                    "source_relative_timestamp_s": (
                        int(frame["timestamp_ns"]) - source_start_ns
                    ) / 1_000_000_000.0,
                    "task": frame.get("task"),
                    "observation.state.arm.position": frame["observation"]["state"]["arm_position"],
                    "observation.state.hand.position": frame["observation"]["state"]["hand_position"],
                    "action.arm.position": frame["action"]["arm_position"],
                    "action.hand.position": frame["action"]["hand_position"],
                    "provenance": {
                        "repaired": bool(frame["validity"].get("repaired")),
                        "repair_sources": frame["validity"].get("repair_sources") or [],
                        "alignment_delta_ms": frame.get("alignment_delta_ms"),
                    },
                }
                for camera in required_cameras:
                    record[f"observation.images.{camera}"] = images[camera]["path"]
                output.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")
            summary["segments"].append(
                {
                    "episode_index": segment_index,
                    "frame_count": len(run),
                    "source_start_frame_index": int(run[0]["frame_index"]),
                    "source_end_frame_index": int(run[-1]["frame_index"]),
                    "source_start_timestamp_ns": source_start_ns,
                    "source_end_timestamp_ns": int(run[-1]["timestamp_ns"]),
                    "duration_s": (len(run) - 1) / output_fps,
                }
            )
    os.replace(temporary, output_path)
    summary["segment_count"] = len(retained)
    summary["exported_frame_count"] = sum(len(run) for run in retained)
    return summary


def align_episode(episode_dir: Path, config: dict[str, Any] | None = None) -> dict[str, Any]:
    episode_dir = episode_dir.resolve()
    manifest_path = episode_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if config is None:
        config = yaml.safe_load((episode_dir / "config.yaml").read_text(encoding="utf-8"))

    alignment_config = config.get("offline_alignment", {})
    master_name = str(alignment_config.get("master_camera", "camera_wrist_left"))
    camera_limit_ms = float(alignment_config.get("camera_max_delta_ms", 30))
    arm_limit_ms = float(alignment_config.get("arm_max_delta_ms", 30))
    hand_limit_ms = float(alignment_config.get("hand_max_delta_ms", 30))
    base_limit_ms = float(alignment_config.get("base_max_delta_ms", 100))
    output_fps = float(alignment_config.get("output_fps", config.get("fps", 15)))
    timeline_mode = str(alignment_config.get("timeline_mode", "master_stride"))
    repair_config = dict(alignment_config.get("repair", {}))

    enabled_cameras = [
        name for name, value in config["cameras"].items() if value.get("enabled", True)
    ]
    camera_streams: dict[str, tuple[list[dict[str, Any]], list[int]]] = {}
    raw_counts: dict[str, int] = {}
    for name in enabled_cameras:
        path = episode_dir / "raw" / "cameras" / name / "timestamps.jsonl"
        records = load_jsonl(path)
        stream = sorted_records(records, sync_timestamp_ns)
        camera_streams[name] = stream
        raw_counts[f"camera:{name}"] = len(stream[0])

    if master_name not in camera_streams or not camera_streams[master_name][0]:
        raise RuntimeError(f"offline alignment has no frames for master camera {master_name}")

    ros_names = (
        "arm_joint_state",
        "arm_command_left",
        "arm_command_right",
        "base_command",
        "base_feedback",
        "base_joint_state",
    )
    streams: dict[str, tuple[list[dict[str, Any]], list[int]]] = dict(camera_streams)
    for name in ros_names:
        records = load_jsonl(episode_dir / "raw" / "ros" / f"{name}.jsonl")
        streams[name] = sorted_records(records, source_timestamp)
        raw_counts[f"ros:{name}"] = len(streams[name][0])
    hand_records = load_jsonl(episode_dir / "raw" / "hands" / "teleop.jsonl")
    streams["hand_action"] = sorted_records(hand_records, hand_timestamp)
    streams["hand_state"] = sorted_records(hand_records, hand_actual_timestamp)
    raw_counts["hand:teleop"] = len(hand_records)
    hardware_hand_state_count = len(streams["hand_state"][0])
    hardware_hand_state_available = hardware_hand_state_count > 0
    raw_counts["hand:hardware_state"] = hardware_hand_state_count
    if not hardware_hand_state_available:
        # Legacy button-controller logs contain the exact commands sent to both
        # hands but no readable Wuji feedback. Keep those episodes alignable for
        # diagnosis without claiming that command fallback is measured state.
        streams["hand_state"] = streams["hand_action"]

    # Training starts only when every required source has a real sample. Camera
    # warmup frames and stale command/state tails remain in raw/ for diagnosis.
    required_for_overlap = enabled_cameras + [
        "arm_joint_state",
        "arm_command_left",
        "arm_command_right",
        "hand_action",
        "hand_state",
    ]
    missing_required = [
        name for name in required_for_overlap if not streams.get(name, ([], []))[1]
    ]
    if missing_required:
        raise RuntimeError(
            "offline alignment is missing required streams: " + ", ".join(missing_required)
        )
    available_ranges = [
        (streams[name][1][0], streams[name][1][-1])
        for name in required_for_overlap
    ]
    overlap_start_ns = max(value[0] for value in available_ranges)
    overlap_end_ns = min(value[1] for value in available_ranges)
    recording_start_ns = int(manifest.get("recording_start_ns", overlap_start_ns))
    recording_end_ns = int(manifest.get("recording_end_ns", overlap_end_ns))
    timeline_start_ns = max(overlap_start_ns, recording_start_ns)
    timeline_end_ns = min(overlap_end_ns, recording_end_ns)
    if timeline_start_ns > timeline_end_ns:
        raise RuntimeError("raw camera, arm, and hand streams have no common time range")

    master_records, master_timestamps = camera_streams[master_name]
    master_fps = float(config["cameras"][master_name].get("fps", output_fps))
    stride = max(1, round(master_fps / output_fps))
    available_master_pairs = [
        (record, timestamp)
        for record, timestamp in zip(master_records, master_timestamps)
        if timeline_start_ns <= timestamp <= timeline_end_ns
    ]
    if timeline_mode == "uniform":
        if not available_master_pairs:
            master_pairs = []
        else:
            period_ns = round(1_000_000_000.0 / output_fps)
            first_reference_ns = available_master_pairs[0][1]
            reference_timestamps = range(first_reference_ns, timeline_end_ns + 1, period_ns)
            master_pairs = []
            for reference_ns in reference_timestamps:
                sample, _ = nearest_record(reference_ns, master_records, master_timestamps)
                if sample is not None:
                    master_pairs.append((sample, reference_ns))
    elif timeline_mode == "master_stride":
        master_pairs = available_master_pairs[::stride]
    else:
        raise ValueError(f"unsupported offline_alignment.timeline_mode: {timeline_mode}")
    if not master_pairs:
        raise RuntimeError("no master camera frames remain after overlap cropping")

    frames_path = episode_dir / "frames.jsonl"
    online_path = episode_dir / "online_frames.jsonl"
    if frames_path.exists() and not online_path.exists():
        os.replace(frames_path, online_path)

    joint_names = list(config["ros"]["arms"]["joint_names"])
    wrist_config = config["timing"]["wrist_pair"]
    left_camera = str(wrist_config["left_camera"])
    right_camera = str(wrist_config["right_camera"])
    wrist_limit_ms = float(wrist_config["max_skew_ms"])
    deltas: dict[str, list[float]] = {}
    frame_count = 0
    valid_count = 0
    temporary = frames_path.with_suffix(".jsonl.tmp")

    with temporary.open("w", encoding="utf-8", buffering=1) as output:
        for master_sample, reference_ns in master_pairs:
            camera_samples: dict[str, dict[str, Any] | None] = {}
            camera_delta_ms: dict[str, float | None] = {}
            for name in enabled_cameras:
                if name == master_name:
                    sample = master_sample
                    delta_ms = (sync_timestamp_ns(sample) - reference_ns) / 1_000_000.0
                    deltas.setdefault(name, []).append(delta_ms)
                else:
                    sample, delta_ms = _select(streams, name, reference_ns, deltas)
                camera_samples[name] = sample
                camera_delta_ms[name] = delta_ms

            arm_raw, arm_delta = _select(streams, "arm_joint_state", reference_ns, deltas)
            left_command, left_command_delta = _select(
                streams, "arm_command_left", reference_ns, deltas
            )
            right_command, right_command_delta = _select(
                streams, "arm_command_right", reference_ns, deltas
            )
            hand_action_raw, hand_action_delta = _select(
                streams, "hand_action", reference_ns, deltas
            )
            hand_state_raw, hand_state_delta = _select(
                streams, "hand_state", reference_ns, deltas
            )
            base_command_raw, base_command_delta = _select(
                streams, "base_command", reference_ns, deltas
            )
            base_feedback_raw, base_feedback_delta = _select(
                streams, "base_feedback", reference_ns, deltas
            )
            base_joint_raw, _ = _select(streams, "base_joint_state", reference_ns, deltas)

            arm_state = reorder_joint_state(arm_raw, joint_names)
            arm_actual = fixed_vector(arm_state.get("position") if arm_state else None, 14)
            left_action = prefix_vector(left_command.get("value") if left_command else None, 7)
            right_action = prefix_vector(right_command.get("value") if right_command else None, 7)
            if left_action is not None and right_action is not None:
                arm_action = left_action + right_action
                arm_action_source = "controller_command"
            else:
                arm_action = arm_actual
                arm_action_source = "state_fallback" if arm_actual is not None else "missing"

            decoded_hand_action = decode_hand_record(hand_action_raw, config["hands"])
            decoded_hand_state = decode_hand_record(hand_state_raw, config["hands"])
            hand_action = decoded_hand_action["action"]
            hand_state = decoded_hand_state["state"]
            hand_state_source = decoded_hand_state["state_source"]
            if not hardware_hand_state_available and hand_state is not None:
                hand_state_source = "command_fallback_unverified"
            hand_delta_candidates = [
                value
                for value in (hand_action_delta, hand_state_delta)
                if isinstance(value, (int, float))
            ]
            hand_delta = (
                max(hand_delta_candidates, key=lambda value: abs(float(value)))
                if hand_delta_candidates
                else None
            )
            base_command = fixed_vector(
                base_command_raw.get("value") if base_command_raw else None, 3
            )
            base_feedback = fixed_vector(
                base_feedback_raw.get("value") if base_feedback_raw else None, 3
            )
            base_action_source = "controller_command"
            base_state_source = "cmd_feedback"
            if base_command is None:
                base_command = [0.0, 0.0, 0.0]
                base_action_source = "zero_fallback"
            if base_feedback is None:
                base_feedback = [0.0, 0.0, 0.0]
                base_state_source = "zero_fallback"

            camera_validity = {
                name: camera_samples[name] is not None
                and _within(camera_delta_ms[name], camera_limit_ms)
                for name in enabled_cameras
            }
            left_sample = camera_samples.get(left_camera)
            right_sample = camera_samples.get(right_camera)
            wrist_skew_ms = None
            if left_sample is not None and right_sample is not None:
                wrist_skew_ms = abs(
                    sync_timestamp_ns(left_sample) - sync_timestamp_ns(right_sample)
                ) / 1_000_000.0
            wrist_valid = wrist_skew_ms is not None and wrist_skew_ms <= wrist_limit_ms
            arm_valid = (
                arm_state is not None
                and arm_action_source == "controller_command"
                and _within(arm_delta, arm_limit_ms)
                and _within(left_command_delta, arm_limit_ms)
                and _within(right_command_delta, arm_limit_ms)
            )
            hand_valid = (
                hand_state is not None
                and hand_action is not None
                and hand_state_source == "hardware_feedback"
                and _within(hand_action_delta, hand_limit_ms)
                and _within(hand_state_delta, hand_limit_ms)
            )
            hand_fallback_valid = (
                not hardware_hand_state_available
                and hand_state is not None
                and hand_action is not None
                and _within(hand_action_delta, hand_limit_ms)
                and _within(hand_state_delta, hand_limit_ms)
            )
            valid = all(camera_validity.values()) and wrist_valid and arm_valid and hand_valid
            aligned_with_available_hand_state = (
                all(camera_validity.values())
                and wrist_valid
                and arm_valid
                and (hand_valid or hand_fallback_valid)
            )

            frame = {
                "frame_index": frame_count,
                "timestamp_ns": reference_ns,
                "task": manifest.get("task"),
                "skill_id": manifest.get("skill_id"),
                "prompt_version": manifest.get("prompt_version"),
                "observation": {
                    "images": {
                        name: image_value(camera_samples[name]) for name in enabled_cameras
                    },
                    "state": {
                        "arm_position": arm_state.get("position") if arm_state else None,
                        "arm_velocity": arm_state.get("velocity") if arm_state else None,
                        "arm_effort": arm_state.get("effort") if arm_state else None,
                        "hand_position": hand_state,
                        "base_velocity": base_feedback,
                    },
                },
                "action": {
                    "arm_position": arm_action,
                    "hand_position": hand_action,
                    "base_velocity": base_command,
                },
                "source": {
                    "arm_action": arm_action_source,
                    "hand_state": hand_state_source,
                    "base_action": base_action_source,
                    "base_state": base_state_source,
                },
                "source_timestamps_ns": {
                    "arm": source_timestamp(arm_raw) if arm_raw else None,
                    "arm_command_left": source_timestamp(left_command) if left_command else None,
                    "arm_command_right": source_timestamp(right_command) if right_command else None,
                    "hand_action": hand_timestamp(hand_action_raw) if hand_action_raw else None,
                    "hand_actual": (
                        hand_actual_timestamp(hand_state_raw) if hand_state_raw else None
                    ),
                    "base_feedback": source_timestamp(base_feedback_raw) if base_feedback_raw else None,
                    "base_command": source_timestamp(base_command_raw) if base_command_raw else None,
                },
                "alignment_delta_ms": {
                    "cameras": camera_delta_ms,
                    "arm": arm_delta,
                    "arm_command_left": left_command_delta,
                    "arm_command_right": right_command_delta,
                    "hand": hand_delta,
                    "hand_action": hand_action_delta,
                    "hand_state": hand_state_delta,
                    "base_feedback": base_feedback_delta,
                    "base_command": base_command_delta,
                },
                "validity": {
                    "valid_for_training": valid,
                    "offline_aligned": True,
                    "arm_aligned": arm_valid,
                    "hand_aligned": hand_valid,
                    "hardware_hand_state_available": hardware_hand_state_available,
                    "hand_state_mode": (
                        "hardware_feedback"
                        if hardware_hand_state_available
                        else "command_fallback_unverified"
                    ),
                    "aligned_with_available_hand_state": aligned_with_available_hand_state,
                    "base_feedback_aligned": _within(base_feedback_delta, base_limit_ms),
                    "cameras_aligned": camera_validity,
                    "wrist_pair_valid": wrist_valid,
                    "wrist_pair_skew_ms": wrist_skew_ms,
                    "wrist_pair_reason": "matched" if wrist_valid else "skew_exceeded",
                },
                "raw": {
                    "hand": decoded_hand_action["raw"],
                    "hand_state": decoded_hand_state["raw"],
                    "base_feedback": base_feedback_raw,
                    "base_command": base_command_raw,
                    "base_joint_state": base_joint_raw,
                },
            }
            output.write(json.dumps(frame, ensure_ascii=False, separators=(",", ":")) + "\n")
            frame_count += 1
            valid_count += int(valid)

    pre_repair_valid_count = valid_count
    frames = load_jsonl(temporary)
    repair_summary = repair_isolated_frames(
        frames,
        repair_config,
        output_fps=output_fps,
        arm_limit_ms=arm_limit_ms,
        hand_limit_ms=hand_limit_ms,
        wrist_left_camera=left_camera,
        wrist_right_camera=right_camera,
        wrist_pair_limit_ms=wrist_limit_ms,
    )
    frame_count = len(frames)
    valid_count = sum(
        int(bool((frame.get("validity") or {}).get("valid_for_training")))
        for frame in frames
    )
    available_hand_aligned_count = (
        valid_count
        if hardware_hand_state_available
        else sum(
            int(
                bool(
                    (frame.get("validity") or {}).get(
                        "aligned_with_available_hand_state"
                    )
                )
            )
            for frame in frames
        )
    )
    # Acquisition repair keeps one continuous episode. Training conversion is
    # deliberately separate and must not split or retime the recorded action.
    for generated_name in ("lerobot_frames.jsonl", "training_frames.jsonl"):
        generated_path = episode_dir / generated_name
        if generated_path.exists():
            generated_path.unlink()
    paired_segments = refresh_paired_segment_indices(episode_dir, frames)
    with temporary.open("w", encoding="utf-8", buffering=1) as output:
        for frame in frames:
            output.write(json.dumps(frame, ensure_ascii=False, separators=(",", ":")) + "\n")
    os.replace(temporary, frames_path)
    source_stats = {name: alignment_stats(values) for name, values in deltas.items()}
    valid_flags = [
        bool((frame.get("validity") or {}).get("valid_for_training")) for frame in frames
    ]
    if any(valid_flags):
        first_valid = valid_flags.index(True)
        last_valid = len(valid_flags) - 1 - valid_flags[::-1].index(True)
        leading_invalid_count = first_valid
        trailing_invalid_count = len(valid_flags) - 1 - last_valid
        internal_invalid_count = sum(
            int(not valid_flags[index]) for index in range(first_valid, last_valid + 1)
        )
    else:
        leading_invalid_count = len(valid_flags)
        trailing_invalid_count = 0
        internal_invalid_count = 0
    frame_gaps_ms = [
        (int(frames[index]["timestamp_ns"]) - int(frames[index - 1]["timestamp_ns"]))
        / 1_000_000.0
        for index in range(1, len(frames))
    ]
    gap_limit_ms = 1.5 * 1000.0 / output_fps
    continuity = {
        "single_continuous_episode": True,
        "leading_invalid_frame_count": leading_invalid_count,
        "trailing_invalid_frame_count": trailing_invalid_count,
        "internal_invalid_frame_count": internal_invalid_count,
        "max_frame_gap_ms": max(frame_gaps_ms) if frame_gaps_ms else 0.0,
        "timing_gap_count": sum(gap > gap_limit_ms for gap in frame_gaps_ms),
        "timing_gap_limit_ms": gap_limit_ms,
        "complete_episode_candidate": bool(valid_flags)
        and internal_invalid_count == 0
        and not any(gap > gap_limit_ms for gap in frame_gaps_ms),
    }
    invalid_reason_counts: dict[str, int] = {}
    for frame in frames:
        validity = frame.get("validity") or {}
        if validity.get("valid_for_training"):
            continue
        for camera, aligned in (validity.get("cameras_aligned") or {}).items():
            if not aligned:
                key = f"camera:{camera}"
                invalid_reason_counts[key] = invalid_reason_counts.get(key, 0) + 1
        for key, label in (
            ("wrist_pair_valid", "wrist_pair"),
            ("arm_aligned", "arm"),
            ("hand_aligned", "hand"),
        ):
            if not validity.get(key):
                invalid_reason_counts[label] = invalid_reason_counts.get(label, 0) + 1
    report = {
        "method": "offline_nearest_neighbor_continuous_episode_with_short_gap_repair",
        "master_camera": master_name,
        "timeline_mode": timeline_mode,
        "output_fps": output_fps,
        "master_stride": stride,
        "overlap_start_ns": overlap_start_ns,
        "overlap_end_ns": overlap_end_ns,
        "recording_start_ns": recording_start_ns,
        "recording_end_ns": recording_end_ns,
        "timeline_start_ns": timeline_start_ns,
        "timeline_end_ns": timeline_end_ns,
        "overlap_sources": required_for_overlap,
        "thresholds_ms": {
            "camera": camera_limit_ms,
            "wrist_pair": wrist_limit_ms,
            "arm": arm_limit_ms,
            "hand": hand_limit_ms,
            "base": base_limit_ms,
        },
        "frame_count": frame_count,
        "pre_repair_valid_frame_count": pre_repair_valid_count,
        "valid_frame_count": valid_count,
        "available_hand_aligned_frame_count": available_hand_aligned_count,
        "invalid_frame_count": frame_count - valid_count,
        "invalid_reason_counts": invalid_reason_counts,
        "hand_state": {
            "mode": (
                "hardware_feedback"
                if hardware_hand_state_available
                else "command_fallback_unverified"
            ),
            "hardware_feedback_record_count": hardware_hand_state_count,
            "command_fallback_used": not hardware_hand_state_available,
            "strict_training_valid": hardware_hand_state_available,
        },
        "repair": repair_summary,
        "continuity": continuity,
        "paired_segments": paired_segments,
        "raw_stream_counts": raw_counts,
        "source_alignment": source_stats,
    }
    atomic_write_json(episode_dir / "alignment_report.json", report)
    manifest["frame_count"] = frame_count
    manifest["valid_frame_count"] = valid_count
    manifest.pop("lerobot_frame_count", None)
    manifest.pop("lerobot_segment_count", None)
    manifest.pop("training_frame_count", None)
    manifest.pop("training_segment_count", None)
    manifest["offline_alignment"] = report
    manifest.pop("offline_alignment_error", None)
    atomic_write_json(manifest_path, manifest)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description="Offline-align one raw SVT LingBot episode.")
    parser.add_argument("episode", type=Path)
    args = parser.parse_args()
    report = align_episode(args.episode)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
