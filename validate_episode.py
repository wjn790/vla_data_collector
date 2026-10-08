#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from collections import Counter
from pathlib import Path
from typing import Any


VECTOR_SIZES = {
    ("observation", "state", "arm_position"): 14,
    ("observation", "state", "hand_position"): 12,
    ("observation", "state", "base_velocity"): 3,
    ("action", "arm_position"): 14,
    ("action", "hand_position"): 12,
    ("action", "base_velocity"): 3,
}

LEROBOT_VECTOR_SIZES = {
    "observation.state.arm.position": 14,
    "observation.state.hand.position": 12,
    "action.arm.position": 14,
    "action.hand.position": 12,
}


def nested(value: dict[str, Any], path: tuple[str, ...]) -> Any:
    current: Any = value
    for key in path:
        if not isinstance(current, dict):
            return None
        current = current.get(key)
    return current


def valid_vector(value: Any, size: int) -> bool:
    return (
        isinstance(value, list)
        and len(value) == size
        and all(isinstance(item, (int, float)) and math.isfinite(item) for item in value)
    )


def camera_delta_allowed(
    camera: str,
    delta: Any,
    standard_limit_ms: Any,
    repair_sources: dict[str, Any],
    repair_policy: dict[str, Any],
) -> bool:
    if not isinstance(delta, (int, float)) or not isinstance(
        standard_limit_ms, (int, float)
    ):
        return False
    absolute_delta = abs(float(delta))
    if absolute_delta <= float(standard_limit_ms):
        return True
    camera_repair = repair_sources.get(f"camera:{camera}") or {}
    camera_repair_limit = repair_policy.get("camera_max_delta_ms")
    if (
        camera_repair.get("method") == "accept_isolated_nearest_raw_image"
        and isinstance(camera_repair_limit, (int, float))
        and absolute_delta <= float(camera_repair_limit)
    ):
        return True
    wrist_pair_repair = repair_sources.get("wrist_pair") or {}
    wrist_pair_repair_limit = repair_policy.get(
        "wrist_pair_camera_max_delta_ms", camera_repair_limit
    )
    if (
        wrist_pair_repair.get("method")
        == "reuse_bounded_synchronized_neighbor_pair"
        and isinstance(wrist_pair_repair_limit, (int, float))
        and absolute_delta <= float(wrist_pair_repair_limit)
    ):
        return True
    timing_repair = repair_sources.get("timing") or {}
    timing_repair_limit = repair_policy.get("timing_camera_max_delta_ms")
    return bool(
        timing_repair.get("method")
        == "missing_timestep_interpolation_nearest_raw_images"
        and isinstance(timing_repair_limit, (int, float))
        and absolute_delta <= float(timing_repair_limit)
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate a raw LingBot episode.")
    parser.add_argument("episode", type=Path)
    parser.add_argument("--decode-images", action="store_true")
    args = parser.parse_args()
    episode = args.episode.resolve()
    manifest_path = episode / "manifest.json"
    frames_path = episode / "frames.jsonl"
    if not manifest_path.is_file() or not frames_path.is_file():
        print("ERROR: episode must contain manifest.json and frames.jsonl", file=sys.stderr)
        return 2

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    alignment = manifest.get("offline_alignment") or {}
    thresholds = alignment.get("thresholds_ms") or {}
    repair_policy = alignment.get("repair") or {}
    errors: list[str] = []
    warnings: list[str] = []
    source_counts: Counter[str] = Counter()
    invalid_reasons: Counter[str] = Counter()
    image_paths: set[Path] = set()
    previous_timestamp = 0
    frame_count = 0
    valid_count = 0
    repaired_count = 0
    lerobot_eligible_count = 0
    source_frames: dict[int, dict[str, Any]] = {}
    paired_source = manifest.get("paired_continuous_source")
    is_paired_continuous = isinstance(paired_source, dict)
    atomic_skill = bool(manifest.get("skill_id")) and not is_paired_continuous
    paired_frame_metadata: dict[int, dict[str, Any]] = {}
    if is_paired_continuous:
        segments_name = str(paired_source.get("segments_file") or "paired_segments.json")
        segments_path = episode / segments_name
        if not segments_path.is_file():
            errors.append(f"paired continuous source is missing {segments_name}")
        else:
            segments_document = json.loads(segments_path.read_text(encoding="utf-8"))
            segments = segments_document.get("segments")
            if not isinstance(segments, list) or not segments:
                errors.append("paired segments must be a non-empty list")
            else:
                for segment_index, segment in enumerate(segments):
                    if not isinstance(segment, dict):
                        errors.append(f"paired segment {segment_index} is not a mapping")
                        continue
                    try:
                        start_frame = int(segment["source_start_frame"])
                        end_frame = int(segment["source_end_frame"])
                        skill_id = str(segment["skill_id"])
                        task = str(segment["task"])
                        prompt_version = str(segment["prompt_version"])
                        start_ns = int(segment["start_ns"])
                        end_ns = int(segment["end_ns"])
                    except (KeyError, TypeError, ValueError) as exc:
                        errors.append(f"paired segment {segment_index} is invalid: {exc}")
                        continue
                    for frame_index in range(start_frame, end_frame + 1):
                        if frame_index in paired_frame_metadata:
                            errors.append(
                                f"paired segments overlap at frame {frame_index}"
                            )
                            continue
                        paired_frame_metadata[frame_index] = {
                            "segment_index": segment_index,
                            "skill_id": skill_id,
                            "task": task,
                            "prompt_version": prompt_version,
                            "start_ns": start_ns,
                            "end_ns": end_ns,
                            "start_inclusive": bool(segment.get("start_inclusive", True)),
                            "end_inclusive": bool(segment.get("end_inclusive", False)),
                        }
    base_motion = manifest.get("base_motion")
    base_motion_allowed = bool(
        manifest.get("skill_id") == "S1"
        and manifest.get("prompt_version") == "v2"
        and isinstance(base_motion, dict)
        and base_motion.get("allowed") is True
        and base_motion.get("concurrent_prior") == "B1"
        and base_motion.get("included_in_training_action") is False
    )
    task_mismatch_count = 0
    nonzero_base_command_count = 0
    invalid_base_command_count = 0

    if atomic_skill:
        if float(manifest.get("fps", -1)) != 15:
            errors.append("atomic skill manifest must declare exactly 15 Hz")
        for key in (
            "prompt_version",
            "attempt_outcome",
            "perturbation",
            "hand_control_mode",
            "operator",
            "config_checksums",
        ):
            if not manifest.get(key):
                errors.append(f"atomic skill manifest is missing {key}")
        snapshot = (manifest.get("config_checksums") or {}).get("episode_config_snapshot") or {}
        snapshot_path = episode / str(snapshot.get("path", "config.yaml"))
        if not snapshot_path.is_file():
            errors.append("episode config snapshot is missing")
        elif hashlib.sha256(snapshot_path.read_bytes()).hexdigest() != snapshot.get("sha256"):
            errors.append("episode config snapshot checksum does not match manifest")

    with frames_path.open("r", encoding="utf-8") as handle:
        for line_number, line in enumerate(handle, start=1):
            try:
                frame = json.loads(line)
            except json.JSONDecodeError as exc:
                errors.append(f"line {line_number}: invalid JSON: {exc}")
                continue
            if frame.get("frame_index") != frame_count:
                errors.append(
                    f"line {line_number}: frame_index={frame.get('frame_index')} expected={frame_count}"
                )
            frame_count += 1
            if atomic_skill:
                task_mismatch_count += int(frame.get("task") != manifest.get("task"))
            base_command = nested(frame, ("action", "base_velocity"))
            if not valid_vector(base_command, 3):
                invalid_base_command_count += 1
            elif any(abs(float(value)) > 1e-4 for value in base_command):
                nonzero_base_command_count += 1
            timestamp = frame.get("timestamp_ns")
            if not isinstance(timestamp, int) or timestamp <= previous_timestamp:
                errors.append(f"line {line_number}: non-monotonic timestamp")
            else:
                previous_timestamp = timestamp

            if is_paired_continuous:
                expected = paired_frame_metadata.get(int(frame.get("frame_index", -1)))
                if expected is None:
                    errors.append(
                        f"line {line_number}: frame is not assigned to a paired segment"
                    )
                else:
                    for key in ("skill_id", "task", "prompt_version"):
                        if frame.get(key) != expected[key]:
                            errors.append(
                                f"line {line_number}: paired {key}={frame.get(key)!r} "
                                f"expected={expected[key]!r}"
                            )
                    paired = frame.get("paired_segment") or {}
                    if (
                        paired.get("segment_index") != expected["segment_index"]
                        or paired.get("skill_id") != expected["skill_id"]
                    ):
                        errors.append(
                            f"line {line_number}: paired_segment provenance mismatch"
                        )
                    if isinstance(timestamp, int):
                        after_start = (
                            timestamp >= expected["start_ns"]
                            if expected["start_inclusive"]
                            else timestamp > expected["start_ns"]
                        )
                        before_end = (
                            timestamp <= expected["end_ns"]
                            if expected["end_inclusive"]
                            else timestamp < expected["end_ns"]
                        )
                        if not (after_start and before_end):
                            errors.append(
                                f"line {line_number}: timestamp is outside paired segment"
                            )

            for path, size in VECTOR_SIZES.items():
                if not valid_vector(nested(frame, path), size):
                    invalid_reasons[".".join(path)] += 1

            images = nested(frame, ("observation", "images")) or {}
            for camera, image in images.items():
                if not isinstance(image, dict) or not image.get("path"):
                    invalid_reasons[f"image:{camera}"] += 1
                    continue
                path = episode / image["path"]
                image_paths.add(path)
                if not path.is_file():
                    errors.append(f"line {line_number}: missing image {image['path']}")

            source = frame.get("source") or {}
            for key, value in source.items():
                source_counts[f"{key}={value}"] += 1
            validity = frame.get("validity") or {}
            valid_count += int(bool(validity.get("valid_for_training")))
            lerobot_eligible_count += int(bool(validity.get("eligible_for_lerobot")))
            source_frames[int(frame.get("frame_index", -1))] = {
                "timestamp_ns": timestamp,
                "valid_for_training": bool(validity.get("valid_for_training")),
                "eligible_for_lerobot": bool(validity.get("eligible_for_lerobot")),
                "lerobot_segment_index": validity.get("lerobot_segment_index"),
                "lerobot_frame_index": validity.get("lerobot_frame_index"),
            }
            repair = frame.get("repair") or {}
            if validity.get("repaired"):
                repaired_count += 1
                if not repair.get("applied") or not (repair.get("sources") or {}):
                    errors.append(f"line {line_number}: repaired frame has no repair metadata")
            if validity.get("valid_for_training") and validity.get("offline_aligned"):
                alignment_delta = frame.get("alignment_delta_ms") or {}
                repair_sources = repair.get("sources") or {}
                camera_limit = thresholds.get("camera")
                if isinstance(camera_limit, (int, float)):
                    for camera, delta in (alignment_delta.get("cameras") or {}).items():
                        if not camera_delta_allowed(
                            camera,
                            delta,
                            camera_limit,
                            repair_sources,
                            repair_policy,
                        ):
                            errors.append(
                                f"line {line_number}: valid frame has {camera} delta {delta} ms"
                            )
                wrist_limit = thresholds.get("wrist_pair")
                wrist_skew = validity.get("wrist_pair_skew_ms")
                if (
                    isinstance(wrist_limit, (int, float))
                    and (not isinstance(wrist_skew, (int, float)) or wrist_skew > wrist_limit)
                ):
                    errors.append(
                        f"line {line_number}: valid frame has wrist skew {wrist_skew} ms"
                    )
                for source, keys in {
                    "arm": ("arm", "arm_command_left", "arm_command_right"),
                    "hand": ("hand",),
                }.items():
                    limit = thresholds.get(source)
                    if not isinstance(limit, (int, float)):
                        continue
                    source_deltas = [alignment_delta.get(key) for key in keys]
                    exceeds = any(
                        isinstance(delta, (int, float)) and abs(delta) > limit
                        for delta in source_deltas
                    )
                    if exceeds and source not in repair_sources:
                        errors.append(
                            f"line {line_number}: valid frame has unrepaired {source} delta"
                        )

    if frame_count != int(manifest.get("frame_count", -1)):
        errors.append(
            f"manifest frame_count={manifest.get('frame_count')} but frames.jsonl has {frame_count}"
        )
    if valid_count != int(manifest.get("valid_frame_count", -1)):
        errors.append(
            f"manifest valid_frame_count={manifest.get('valid_frame_count')} but counted {valid_count}"
        )
    if is_paired_continuous and len(paired_frame_metadata) != frame_count:
        errors.append(
            f"paired segments cover {len(paired_frame_metadata)} frames but episode has {frame_count}"
        )
    if atomic_skill:
        if valid_count < 75:
            errors.append(f"atomic skill has only {valid_count} valid frames; minimum is 75")
        if task_mismatch_count:
            errors.append(f"atomic skill has {task_mismatch_count} frame prompt mismatch(es)")
        if invalid_base_command_count:
            errors.append(
                f"atomic skill has {invalid_base_command_count} invalid base command frame(s)"
            )
        if nonzero_base_command_count and not base_motion_allowed:
            errors.append(
                f"atomic skill has {nonzero_base_command_count} nonzero or invalid base command frame(s)"
            )
    reported_repaired = int((alignment.get("repair") or {}).get("repaired_frame_count", 0))
    if repaired_count != reported_repaired:
        errors.append(
            f"alignment repaired_frame_count={reported_repaired} but counted {repaired_count}"
        )

    continuity = alignment.get("continuity") or {}
    internal_invalid = int(continuity.get("internal_invalid_frame_count", 0))
    timing_gaps = int(continuity.get("timing_gap_count", 0))
    if internal_invalid:
        errors.append(
            f"continuous episode contains {internal_invalid} invalid frame(s) inside the action"
        )
    if timing_gaps:
        errors.append(f"continuous episode contains {timing_gaps} source timing gap(s)")
    for legacy_name in ("lerobot_frames.jsonl", "training_frames.jsonl"):
        if (episode / legacy_name).exists():
            warnings.append(
                f"legacy generated file {legacy_name} exists; rerun offline alignment to remove it"
            )

    # Training conversion is intentionally outside the acquisition validator.
    staging: dict[str, Any] = {}
    lerobot_frame_count = 0
    lerobot_segment_count = 0
    if staging.get("enabled"):
        staging_path = episode / str(staging.get("path", "lerobot_frames.jsonl"))
        if not staging_path.is_file():
            errors.append(f"missing LeRobot staging file {staging_path.name}")
        else:
            fps = float(staging.get("fps", manifest.get("fps", 0)))
            max_gap_ms = float(staging.get("max_frame_gap_ms", 0))
            required_cameras = list(staging.get("required_cameras") or [])
            current_segment = -1
            expected_frame_index = 0
            previous_source_timestamp_ns: int | None = None
            seen_sources: set[int] = set()
            with staging_path.open("r", encoding="utf-8") as handle:
                for line_number, line in enumerate(handle, start=1):
                    try:
                        record = json.loads(line)
                    except json.JSONDecodeError as exc:
                        errors.append(
                            f"{staging_path.name} line {line_number}: invalid JSON: {exc}"
                        )
                        continue
                    segment_index = record.get("episode_index")
                    frame_index = record.get("frame_index")
                    if segment_index != current_segment:
                        if segment_index != current_segment + 1:
                            errors.append(
                                f"{staging_path.name} line {line_number}: non-contiguous episode_index"
                            )
                        current_segment = int(segment_index)
                        lerobot_segment_count += 1
                        expected_frame_index = 0
                        previous_source_timestamp_ns = None
                    if frame_index != expected_frame_index:
                        errors.append(
                            f"{staging_path.name} line {line_number}: frame_index={frame_index} "
                            f"expected={expected_frame_index}"
                        )
                    expected_timestamp = expected_frame_index / fps
                    timestamp = record.get("timestamp")
                    if (
                        not isinstance(timestamp, (int, float))
                        or not math.isfinite(timestamp)
                        or abs(float(timestamp) - expected_timestamp) > 1e-6
                    ):
                        errors.append(
                            f"{staging_path.name} line {line_number}: timestamp={timestamp} "
                            f"expected={expected_timestamp}"
                        )
                    expected_frame_index += 1
                    lerobot_frame_count += 1

                    source_index = record.get("source_frame_index")
                    source = source_frames.get(source_index)
                    if source is None:
                        errors.append(
                            f"{staging_path.name} line {line_number}: unknown source frame {source_index}"
                        )
                    else:
                        if source_index in seen_sources:
                            errors.append(
                                f"{staging_path.name} line {line_number}: duplicate source frame {source_index}"
                            )
                        seen_sources.add(source_index)
                        if not source["valid_for_training"] or not source["eligible_for_lerobot"]:
                            errors.append(
                                f"{staging_path.name} line {line_number}: source frame {source_index} is not eligible"
                            )
                        if (
                            source["lerobot_segment_index"] != segment_index
                            or source["lerobot_frame_index"] != frame_index
                        ):
                            errors.append(
                                f"{staging_path.name} line {line_number}: source staging metadata mismatch"
                            )
                        source_timestamp_ns = int(source["timestamp_ns"])
                        if previous_source_timestamp_ns is not None:
                            gap_ms = (
                                source_timestamp_ns - previous_source_timestamp_ns
                            ) / 1_000_000.0
                            if gap_ms <= 0 or gap_ms > max_gap_ms:
                                errors.append(
                                    f"{staging_path.name} line {line_number}: source gap {gap_ms} ms"
                                )
                        previous_source_timestamp_ns = source_timestamp_ns

                    for key, size in LEROBOT_VECTOR_SIZES.items():
                        if not valid_vector(record.get(key), size):
                            errors.append(
                                f"{staging_path.name} line {line_number}: invalid {key}"
                            )
                    for camera in required_cameras:
                        key = f"observation.images.{camera}"
                        image_path = record.get(key)
                        if not isinstance(image_path, str) or not image_path:
                            errors.append(
                                f"{staging_path.name} line {line_number}: missing {key}"
                            )
                        elif not (episode / image_path).is_file():
                            errors.append(
                                f"{staging_path.name} line {line_number}: missing image {image_path}"
                            )

        reported_frames = int(staging.get("exported_frame_count", -1))
        reported_segments = int(staging.get("segment_count", -1))
        if lerobot_frame_count != reported_frames:
            errors.append(
                f"LeRobot staging exported_frame_count={reported_frames} but counted {lerobot_frame_count}"
            )
        if lerobot_segment_count != reported_segments:
            errors.append(
                f"LeRobot staging segment_count={reported_segments} but counted {lerobot_segment_count}"
            )
        if lerobot_eligible_count != lerobot_frame_count:
            errors.append(
                f"frames.jsonl has {lerobot_eligible_count} eligible frames but staging has {lerobot_frame_count}"
            )
        if int(manifest.get("lerobot_frame_count", -1)) != lerobot_frame_count:
            errors.append("manifest lerobot_frame_count does not match staging")
        if int(manifest.get("lerobot_segment_count", -1)) != lerobot_segment_count:
            errors.append("manifest lerobot_segment_count does not match staging")
    for key, count in invalid_reasons.items():
        warnings.append(f"{key} missing/invalid in {count}/{frame_count} frames")

    if args.decode_images and image_paths:
        try:
            import cv2
        except ImportError:
            errors.append("--decode-images requested but cv2 is unavailable")
        else:
            for path in sorted(image_paths):
                image = cv2.imread(str(path), cv2.IMREAD_COLOR)
                if image is None or image.size == 0:
                    errors.append(f"cannot decode image {path.relative_to(episode)}")

    print(f"episode: {episode}")
    print(f"status: {manifest.get('status')}")
    print(f"frames: {frame_count}; valid_for_training: {valid_count}")
    print(f"repaired isolated frames: {repaired_count}")
    if continuity:
        print(
            "continuity: "
            f"leading_invalid={continuity.get('leading_invalid_frame_count')} "
            f"internal_invalid={continuity.get('internal_invalid_frame_count')} "
            f"trailing_invalid={continuity.get('trailing_invalid_frame_count')} "
            f"timing_gaps={continuity.get('timing_gap_count')}"
        )
    print(f"unique images: {len(image_paths)}")
    if alignment:
        print(
            "offline alignment: "
            f"{alignment.get('method')} master={alignment.get('master_camera')} "
            f"valid={alignment.get('valid_frame_count')}/{alignment.get('frame_count')}"
        )
        for name, stats in sorted((alignment.get("source_alignment") or {}).items()):
            print(
                f"  {name}: median={stats.get('median_ms')} ms "
                f"p90={stats.get('p90_ms')} ms max={stats.get('max_ms')} ms"
            )
    if source_counts:
        print("sources:")
        for key, count in sorted(source_counts.items()):
            print(f"  {key}: {count}")
    for warning in warnings:
        print(f"WARNING: {warning}")
    for error in errors:
        print(f"ERROR: {error}", file=sys.stderr)
    return 2 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
