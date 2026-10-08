#!/usr/bin/env python3
"""Align completed paired episodes and emit read-only quality assessments."""

from __future__ import annotations

import argparse
import bisect
import datetime as dt
import fcntl
import hashlib
import json
import math
import os
import shutil
import signal
import time
from pathlib import Path
from typing import Any

import yaml

from common import atomic_write_json
from offline_align import align_episode
from realign_day import merge_alignment_policy


MONITOR_VERSION = 1


def load_json(path: Path, default: Any = None) -> Any:
    if not path.is_file():
        return default
    return json.loads(path.read_text(encoding="utf-8"))


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    values = []
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            if not line.strip():
                continue
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(value, dict):
                values.append(value)
    return values


def vector_distance(before: Any, after: Any, length: int = 7) -> float | None:
    if not isinstance(before, list) or not isinstance(after, list):
        return None
    if len(before) < length or len(after) < length:
        return None
    return math.sqrt(
        sum((float(after[index]) - float(before[index])) ** 2 for index in range(length))
    )


def record_timestamp(record: dict[str, Any]) -> int:
    return int(record.get("received_ns") or record.get("timestamp_ns") or 0)


def nearest_frame_index(timestamps: list[int], target_ns: int) -> int:
    position = bisect.bisect_left(timestamps, target_ns)
    candidates = [min(position, len(timestamps) - 1)]
    if position > 0:
        candidates.append(position - 1)
    return min(candidates, key=lambda index: abs(timestamps[index] - target_ns))


def input_fingerprint(episode: Path) -> str:
    digest = hashlib.sha256()
    relative_paths = [
        "manifest.json",
        "config.yaml",
        "paired_segments.json",
        "frames.jsonl",
        "online_frames.jsonl",
        "alignment_report.json",
        "raw/ros/arm_command_left.jsonl",
        "raw/cameras/camera_top/timestamps.jsonl",
        "raw/cameras/camera_wrist_left/timestamps.jsonl",
        "raw/cameras/camera_wrist_right/timestamps.jsonl",
    ]
    for relative in relative_paths:
        path = episode / relative
        digest.update(relative.encode("utf-8"))
        if path.is_file():
            stat = path.stat()
            digest.update(str(stat.st_size).encode("ascii"))
            digest.update(str(stat.st_mtime_ns).encode("ascii"))
        else:
            digest.update(b"missing")
    return digest.hexdigest()


def episode_is_selected(episode: Path, config: dict[str, Any]) -> bool:
    selection = config.get("selection") or {}
    minimum_day = str(selection.get("minimum_day") or "")
    minimum_name = str(selection.get("minimum_episode_name") or "")
    day = episode.parent.name
    if minimum_day and day < minimum_day:
        return False
    if minimum_day and day == minimum_day and minimum_name and episode.name < minimum_name:
        return False
    suffix = str(selection.get("required_name_suffix") or "")
    return not suffix or episode.name.endswith(suffix)


def episode_is_complete(episode: Path, settle_seconds: float) -> bool:
    required = [
        episode / "manifest.json",
        episode / "config.yaml",
        episode / "paired_segments.json",
        episode / "frames.jsonl",
        episode / "raw",
    ]
    if not all(path.exists() for path in required):
        return False
    manifest = load_json(episode / "manifest.json", {}) or {}
    if manifest.get("status") != "complete":
        return False
    newest = max(path.stat().st_mtime for path in required[:4])
    return time.time() - newest >= settle_seconds


def backup_derived_files(episode: Path, data_root: Path) -> Path:
    backup = (
        data_root
        / "_backups"
        / "auto_align_v1"
        / episode.parent.name
        / episode.name
    )
    backup.mkdir(parents=True, exist_ok=True)
    for name in ("manifest.json", "frames.jsonl", "paired_segments.json"):
        source = episode / name
        target = backup / name
        if source.is_file() and not target.exists():
            shutil.copy2(source, target)
    atomic_write_json(
        backup / "backup_metadata.json",
        {
            "schema_version": 1,
            "created_at": dt.datetime.now().astimezone().isoformat(),
            "source_episode": str(episode),
            "purpose": "pre_auto_offline_alignment",
        },
    )
    return backup


def assess_start_capture(
    episode: Path,
    manifest: dict[str, Any],
    segment_document: dict[str, Any],
    frames: list[dict[str, Any]],
    thresholds: dict[str, Any],
) -> dict[str, Any]:
    issues: list[str] = []
    markers = segment_document.get("phase_markers") or {}
    ready_ns = markers.get("s1_operator_ready_ns")
    grasp_ns = markers.get("s1_grasp_complete_ns")
    if not frames:
        return {"status": "failed", "issues": ["no_aligned_frames"]}
    if not isinstance(ready_ns, int):
        issues.append("missing_s1_operator_ready_marker")
    if not isinstance(grasp_ns, int):
        issues.append("missing_s1_grasp_complete_marker")

    raw_commands = load_jsonl(episode / "raw" / "ros" / "arm_command_left.jsonl")
    recording_start_ns = int(manifest.get("recording_start_ns") or 0)
    before_recording = [
        record for record in raw_commands if record_timestamp(record) <= recording_start_ns
    ]
    raw_initial = raw_commands[0].get("value") if raw_commands else None
    raw_at_recording = (
        before_recording[-1].get("value") if before_recording else raw_initial
    )
    pre_recording_drift = vector_distance(raw_initial, raw_at_recording)
    first_action = (frames[0].get("action") or {}).get("arm_position")
    first_aligned_from_raw_initial = vector_distance(raw_initial, first_action)
    drift_limit = float(thresholds["pre_recording_left_arm_drift_max_rad"])
    if pre_recording_drift is None:
        issues.append("missing_raw_left_arm_commands")
    elif pre_recording_drift > drift_limit:
        issues.append("left_arm_moved_before_recording")
    if (
        first_aligned_from_raw_initial is not None
        and first_aligned_from_raw_initial
        > float(thresholds["first_aligned_from_raw_initial_max_rad"])
    ):
        issues.append("first_aligned_frame_is_not_initial_pose")

    timestamps = [int(frame["timestamp_ns"]) for frame in frames]
    ready_index = (
        nearest_frame_index(timestamps, ready_ns) if isinstance(ready_ns, int) else 0
    )
    grasp_index = (
        nearest_frame_index(timestamps, grasp_ns)
        if isinstance(grasp_ns, int)
        else len(frames) - 1
    )
    ready_action = (frames[ready_index].get("action") or {}).get("arm_position")
    motion_threshold = float(thresholds["left_arm_motion_onset_rad"])
    first_motion_index = None
    for index in range(ready_index, len(frames)):
        action = (frames[index].get("action") or {}).get("arm_position")
        distance = vector_distance(ready_action, action)
        if distance is not None and distance >= motion_threshold:
            first_motion_index = index
            break
    ready_to_motion_seconds = (
        (timestamps[first_motion_index] - int(ready_ns)) / 1_000_000_000.0
        if first_motion_index is not None and isinstance(ready_ns, int)
        else None
    )
    if ready_to_motion_seconds is None:
        issues.append("no_left_arm_approach_motion")
    elif ready_to_motion_seconds < float(thresholds["ready_to_motion_min_sec"]):
        issues.append("left_arm_moved_before_operator_could_confirm_ready")
    approach_motion = vector_distance(
        ready_action,
        (frames[grasp_index].get("action") or {}).get("arm_position"),
    )
    if (
        approach_motion is None
        or approach_motion < float(thresholds["approach_motion_min_rad"])
    ):
        issues.append("s1_approach_motion_too_small")

    unique_issues = list(dict.fromkeys(issues))
    return {
        "status": "pass" if not unique_issues else "failed",
        "issues": unique_issues,
        "recording_start_ns": recording_start_ns,
        "s1_operator_ready_ns": ready_ns,
        "s1_grasp_complete_ns": grasp_ns,
        "ready_frame_index": ready_index,
        "grasp_frame_index": grasp_index,
        "first_motion_frame_index": first_motion_index,
        "pre_recording_left_arm_drift_rad": pre_recording_drift,
        "first_aligned_from_raw_initial_rad": first_aligned_from_raw_initial,
        "ready_to_first_motion_sec": ready_to_motion_seconds,
        "ready_to_grasp_sec": (
            (int(grasp_ns) - int(ready_ns)) / 1_000_000_000.0
            if isinstance(ready_ns, int) and isinstance(grasp_ns, int)
            else None
        ),
        "approach_motion_rad": approach_motion,
    }


def assess_episode(episode: Path, config: dict[str, Any]) -> dict[str, Any]:
    manifest = load_json(episode / "manifest.json", {}) or {}
    alignment = load_json(episode / "alignment_report.json", {}) or {}
    segments = load_json(episode / "paired_segments.json", {}) or {}
    frames = load_jsonl(episode / "frames.jsonl")
    thresholds = config["quality_thresholds"]
    issues: list[str] = []
    frame_count = len(frames)
    valid_count = sum(
        bool((frame.get("validity") or {}).get("valid_for_training")) for frame in frames
    )
    repaired_count = sum(
        bool((frame.get("validity") or {}).get("repaired")) for frame in frames
    )
    camera_names = ("camera_top", "camera_wrist_left", "camera_wrist_right")
    camera_present = {camera: 0 for camera in camera_names}
    bad_action_dimensions = []
    nonfinite_actions = []
    label_mismatches = []
    expected_by_frame: dict[int, tuple[str, str, str]] = {}
    for segment in segments.get("segments") or []:
        expected = (
            str(segment.get("skill_id")),
            str(segment.get("prompt_version")),
            str(segment.get("task")),
        )
        for index in range(
            int(segment.get("source_start_frame", 0)),
            int(segment.get("source_end_frame", -1)) + 1,
        ):
            expected_by_frame[index] = expected
    for position, frame in enumerate(frames):
        images = ((frame.get("observation") or {}).get("images") or {})
        for camera in camera_names:
            image = images.get(camera)
            camera_present[camera] += int(
                isinstance(image, dict) and bool(image.get("path"))
            )
        action = frame.get("action") or {}
        arm = action.get("arm_position")
        hand = action.get("hand_position")
        if not isinstance(arm, list) or len(arm) != 14 or not isinstance(hand, list) or len(hand) != 12:
            bad_action_dimensions.append(position)
        values = (arm if isinstance(arm, list) else []) + (
            hand if isinstance(hand, list) else []
        )
        try:
            if any(not math.isfinite(float(value)) for value in values):
                nonfinite_actions.append(position)
        except (TypeError, ValueError):
            nonfinite_actions.append(position)
        expected = expected_by_frame.get(position)
        actual = (
            str(frame.get("skill_id")),
            str(frame.get("prompt_version")),
            str(frame.get("task")),
        )
        if expected is None or expected != actual:
            label_mismatches.append(position)

    continuity = alignment.get("continuity") or {}
    valid_rate = valid_count / frame_count if frame_count else 0.0
    repair_rate = repaired_count / frame_count if frame_count else 0.0
    if valid_rate < float(thresholds["valid_rate_min"]):
        issues.append("valid_rate_below_threshold")
    if repair_rate > float(thresholds["repair_rate_max"]):
        issues.append("repair_rate_above_threshold")
    if int(continuity.get("internal_invalid_frame_count") or 0) > int(
        thresholds["max_internal_invalid_frames"]
    ):
        issues.append("internal_invalid_frames")
    if int(continuity.get("timing_gap_count") or 0) > int(
        thresholds["max_timing_gaps"]
    ):
        issues.append("timing_gaps")
    if any(count != frame_count for count in camera_present.values()):
        issues.append("missing_aligned_camera_images")
    if bad_action_dimensions:
        issues.append("bad_action_dimensions")
    if nonfinite_actions:
        issues.append("nonfinite_actions")
    if label_mismatches:
        issues.append("prompt_or_segment_label_mismatch")

    start_capture = assess_start_capture(
        episode, manifest, segments, frames, thresholds
    )
    if start_capture["status"] != "pass":
        issues.append("s1_initial_motion_missing_or_suspect")
    unique_issues = list(dict.fromkeys(issues))
    return {
        "monitor_version": MONITOR_VERSION,
        "generated_at": dt.datetime.now().astimezone().isoformat(),
        "episode": str(episode),
        "day": episode.parent.name,
        "episode_name": episode.name,
        "overall_status": "pass" if not unique_issues else "alert",
        "issues": unique_issues,
        "alignment": {
            "method": alignment.get("method"),
            "frame_count": frame_count,
            "valid_frame_count": valid_count,
            "valid_rate": valid_rate,
            "repaired_frame_count": repaired_count,
            "repair_rate": repair_rate,
            "internal_invalid_frame_count": int(
                continuity.get("internal_invalid_frame_count") or 0
            ),
            "timing_gap_count": int(continuity.get("timing_gap_count") or 0),
            "complete_episode_candidate": bool(
                continuity.get("complete_episode_candidate")
            ),
            "camera_present_frame_count": camera_present,
            "bad_action_dimension_frames": bad_action_dimensions,
            "nonfinite_action_frames": nonfinite_actions,
            "label_mismatch_frames": label_mismatches,
            "segments": [
                {
                    "skill_id": segment.get("skill_id"),
                    "prompt_version": segment.get("prompt_version"),
                    "start_frame": segment.get("source_start_frame"),
                    "end_frame": segment.get("source_end_frame"),
                    "frame_count": segment.get("frame_count"),
                }
                for segment in segments.get("segments") or []
            ],
        },
        "start_capture": start_capture,
    }


class EpisodeMonitor:
    def __init__(self, config_path: Path) -> None:
        self.config_path = config_path.resolve()
        self.config = yaml.safe_load(self.config_path.read_text(encoding="utf-8"))
        if not isinstance(self.config, dict):
            raise ValueError(f"invalid monitor config: {self.config_path}")
        self.data_root = Path(self.config["data_root"]).expanduser().resolve()
        self.policy_path = Path(self.config["policy_config"]).expanduser().resolve()
        self.policy = yaml.safe_load(self.policy_path.read_text(encoding="utf-8"))
        self.report_dir = Path(self.config["report_dir"]).expanduser().resolve()
        self.report_dir.mkdir(parents=True, exist_ok=True)
        self.stop_requested = False

    def candidates(self) -> list[Path]:
        values = []
        if not self.data_root.is_dir():
            return values
        for day in sorted(path for path in self.data_root.iterdir() if path.is_dir()):
            if day.name.startswith("_"):
                continue
            for episode in sorted(path for path in day.iterdir() if path.is_dir()):
                if episode_is_selected(episode, self.config):
                    values.append(episode)
        return values

    def report_path(self, episode: Path) -> Path:
        return self.report_dir / f"{episode.parent.name}__{episode.name}.json"

    def should_process(self, episode: Path) -> bool:
        if not episode_is_complete(
            episode, float(self.config.get("settle_seconds_after_complete", 10))
        ):
            return False
        existing = load_json(self.report_path(episode), {}) or {}
        return not (
            existing.get("monitor_version") == MONITOR_VERSION
            and existing.get("input_fingerprint") == input_fingerprint(episode)
        )

    def align_if_needed(self, episode: Path) -> Path | None:
        if (episode / "alignment_report.json").is_file():
            return None
        backup = backup_derived_files(episode, self.data_root)
        episode_config = yaml.safe_load(
            (episode / "config.yaml").read_text(encoding="utf-8")
        )
        if not isinstance(episode_config, dict):
            raise ValueError(f"invalid episode config: {episode / 'config.yaml'}")
        alignment_config = merge_alignment_policy(episode_config, self.policy)
        align_episode(episode, alignment_config)
        return backup

    def write_event(self, report: dict[str, Any]) -> None:
        path = self.report_dir / "events.jsonl"
        with path.open("a", encoding="utf-8", buffering=1) as stream:
            stream.write(json.dumps(report, ensure_ascii=False, separators=(",", ":")) + "\n")

    def refresh_latest(self) -> None:
        reports = []
        for path in self.report_dir.glob("20*.json"):
            value = load_json(path, {}) or {}
            if value.get("episode_name"):
                reports.append(value)
        reports.sort(key=lambda value: (value.get("day", ""), value.get("episode_name", "")))
        atomic_write_json(
            self.report_dir / "latest.json",
            {
                "monitor_version": MONITOR_VERSION,
                "generated_at": dt.datetime.now().astimezone().isoformat(),
                "episode_count": len(reports),
                "alert_count": sum(value.get("overall_status") != "pass" for value in reports),
                "episodes": reports,
            },
        )

    def process(self, episode: Path) -> dict[str, Any]:
        backup = None
        try:
            backup = self.align_if_needed(episode)
            report = assess_episode(episode, self.config)
            report["alignment_result"] = "aligned" if backup else "already_aligned"
            report["backup"] = str(backup) if backup else None
        except Exception as exc:  # noqa: BLE001
            report = {
                "monitor_version": MONITOR_VERSION,
                "generated_at": dt.datetime.now().astimezone().isoformat(),
                "episode": str(episode),
                "day": episode.parent.name,
                "episode_name": episode.name,
                "overall_status": "failed",
                "issues": ["offline_alignment_failed"],
                "error": f"{type(exc).__name__}: {exc}",
                "backup": str(backup) if backup else None,
            }
        report["input_fingerprint"] = input_fingerprint(episode)
        atomic_write_json(self.report_path(episode), report)
        self.write_event(report)
        self.refresh_latest()
        print(
            f"[quality-monitor] {episode.parent.name}/{episode.name} "
            f"status={report['overall_status']} issues={report.get('issues') or []}",
            flush=True,
        )
        return report

    def run_once(self) -> list[dict[str, Any]]:
        reports = []
        for episode in self.candidates():
            if self.should_process(episode):
                reports.append(self.process(episode))
        if not (self.report_dir / "latest.json").is_file():
            self.refresh_latest()
        return reports

    def run(self) -> None:
        interval = float(self.config.get("poll_interval_sec", 15))
        while not self.stop_requested:
            self.run_once()
            deadline = time.monotonic() + interval
            while not self.stop_requested and time.monotonic() < deadline:
                time.sleep(min(0.5, deadline - time.monotonic()))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Monitor, offline-align, and assess completed paired episodes."
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).resolve().parent / "config" / "data_quality_monitor.yaml",
    )
    parser.add_argument("--once", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    lock_path = Path("/tmp/svt_data_quality_monitor.lock")
    with lock_path.open("a+", encoding="utf-8") as lock:
        try:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print("quality monitor is already running", flush=True)
            return 0
        monitor = EpisodeMonitor(args.config)
        if args.once:
            reports = monitor.run_once()
            return int(any(report.get("overall_status") == "failed" for report in reports))

        def request_stop(_signum: int, _frame: Any) -> None:
            monitor.stop_requested = True

        signal.signal(signal.SIGINT, request_stop)
        signal.signal(signal.SIGTERM, request_stop)
        monitor.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
