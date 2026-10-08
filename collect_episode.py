#!/usr/bin/env python3
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import multiprocessing as mp
import os
import queue
import re
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any

import yaml

from camera_capture import camera_worker
from camera_sync import CameraSynchronizer, sync_timestamp_ns
from common import (
    age_ms,
    atomic_write_json,
    fixed_vector,
    is_fresh,
    prefix_vector,
    reorder_joint_state,
)
from hand_log_source import TeleopLogSource, decode_hand_record
from raw_recording import JsonlWriter


COLLECTOR_VERSION = "0.6.0"


CAMERA_TIMING_METRICS = (
    "grab_duration_ms",
    "hardware_timestamp_gap_ms",
    "encoder_queue_delay_ms",
    "encoding_duration_ms",
    "capture_to_encoding_complete_ms",
    "encoder_queue_depth_at_submit",
)


def timing_summary(values: list[float]) -> dict[str, Any]:
    if not values:
        return {"count": 0, "median": None, "p90": None, "p99": None, "max": None}
    ordered = sorted(values)

    def percentile(fraction: float) -> float:
        index = (len(ordered) - 1) * fraction
        lower = int(index)
        upper = min(lower + 1, len(ordered) - 1)
        weight = index - lower
        return ordered[lower] * (1.0 - weight) + ordered[upper] * weight

    return {
        "count": len(ordered),
        "median": percentile(0.5),
        "p90": percentile(0.9),
        "p99": percentile(0.99),
        "max": ordered[-1],
    }


class CameraDiagnostics:
    def __init__(
        self,
        camera_configs: dict[str, dict[str, Any]],
        config: dict[str, Any] | None = None,
    ) -> None:
        self.config = dict(config or {})
        self.print_warnings = bool(self.config.get("print_warnings", True))
        self.hardware_gap_warn_ms = float(
            self.config.get("hardware_gap_warn_ms", 50.0)
        )
        self.grab_duration_warn_ms = float(
            self.config.get("grab_duration_warn_ms", 50.0)
        )
        self.encoder_queue_delay_warn_ms = float(
            self.config.get("encoder_queue_delay_warn_ms", 50.0)
        )
        self.data: dict[str, dict[str, Any]] = {}
        for name, camera_config in camera_configs.items():
            if not camera_config.get("enabled", True):
                continue
            self.data[name] = {
                "backend": str(camera_config.get("backend", "unknown")),
                "frame_count_observed": 0,
                "capture_drop_count": 0,
                "zed_grab_error_count": 0,
                "zed_grab_error_codes": {},
                "hardware_gap_warning_count": 0,
                "grab_duration_warning_count": 0,
                "encoder_queue_delay_warning_count": 0,
                "estimated_missing_frames": 0,
                "metrics": {metric: [] for metric in CAMERA_TIMING_METRICS},
            }

    def _update_zed_errors(self, state: dict[str, Any], item: dict[str, Any]) -> None:
        count = item.get("zed_grab_error_count")
        if isinstance(count, (int, float)):
            state["zed_grab_error_count"] = max(
                int(state["zed_grab_error_count"]), int(count)
            )
        for code, value in (item.get("zed_grab_error_codes") or {}).items():
            state["zed_grab_error_codes"][str(code)] = max(
                int(state["zed_grab_error_codes"].get(str(code), 0)), int(value)
            )

    def observe(self, item: dict[str, Any]) -> None:
        camera = str(item.get("camera"))
        state = self.data.get(camera)
        if state is None:
            return
        kind = item.get("kind")
        self._update_zed_errors(state, item)

        if kind == "capture_drop":
            state["capture_drop_count"] = max(
                int(state["capture_drop_count"]), int(item.get("dropped_count", 0))
            )
            if self.print_warnings:
                print(
                    f"[camera-warning] {camera} encoder queue full; "
                    f"dropped_before_encoding={state['capture_drop_count']}",
                    flush=True,
                )
            return

        if kind == "camera_diagnostic":
            if self.print_warnings and item.get("event") == "zed_grab_error":
                print(
                    f"[camera-warning] {camera} ZED grab failed: "
                    f"status={item.get('status')} "
                    f"total={state['zed_grab_error_count']} "
                    f"grab_ms={float(item.get('grab_duration_ms', 0.0)):.2f}",
                    flush=True,
                )
            return

        if kind != "frame":
            return

        state["frame_count_observed"] += 1
        for metric in CAMERA_TIMING_METRICS:
            value = item.get(metric)
            if isinstance(value, (int, float)):
                state["metrics"][metric].append(float(value))

        warnings: list[str] = []
        gap_ms = item.get("hardware_timestamp_gap_ms")
        if item.get("hardware_gap_warning"):
            state["hardware_gap_warning_count"] += 1
            state["estimated_missing_frames"] += max(
                0, int(item.get("estimated_missing_frames", 0))
            )
            warnings.append(
                f"hardware_gap_ms={float(gap_ms):.2f} "
                f"estimated_missing={int(item.get('estimated_missing_frames', 0))}"
            )

        grab_ms = item.get("grab_duration_ms")
        if isinstance(grab_ms, (int, float)) and float(grab_ms) > self.grab_duration_warn_ms:
            state["grab_duration_warning_count"] += 1
            warnings.append(f"grab_ms={float(grab_ms):.2f}")

        queue_ms = item.get("encoder_queue_delay_ms")
        if (
            isinstance(queue_ms, (int, float))
            and float(queue_ms) > self.encoder_queue_delay_warn_ms
        ):
            state["encoder_queue_delay_warning_count"] += 1
            warnings.append(f"encoder_queue_ms={float(queue_ms):.2f}")

        if warnings and self.print_warnings:
            print(f"[camera-warning] {camera} " + " ".join(warnings), flush=True)

    def live_status(self, camera: str) -> dict[str, Any]:
        state = self.data.get(camera) or {}
        queue_values = (state.get("metrics") or {}).get("encoder_queue_delay_ms") or []
        return {
            "zed_grab_errors": int(state.get("zed_grab_error_count", 0)),
            "hardware_gaps": int(state.get("hardware_gap_warning_count", 0)),
            "encoder_drops": int(state.get("capture_drop_count", 0)),
            "encoder_queue_max_ms": max(queue_values) if queue_values else 0.0,
        }

    def rebuild_from_raw(self, episode_dir: Path) -> None:
        """Use the complete persisted stream for the final manifest summary."""
        print_warnings = self.print_warnings
        self.print_warnings = False
        try:
            for camera, state in self.data.items():
                state["frame_count_observed"] = 0
                state["hardware_gap_warning_count"] = 0
                state["grab_duration_warning_count"] = 0
                state["encoder_queue_delay_warning_count"] = 0
                state["estimated_missing_frames"] = 0
                state["metrics"] = {
                    metric: [] for metric in CAMERA_TIMING_METRICS
                }
                timestamp_path = (
                    episode_dir
                    / "raw"
                    / "cameras"
                    / camera
                    / "timestamps.jsonl"
                )
                if not timestamp_path.is_file():
                    continue
                previous_sequence: int | None = None
                sequence_drop_count = 0
                with timestamp_path.open("r", encoding="utf-8", errors="replace") as handle:
                    for line in handle:
                        try:
                            record = json.loads(line)
                        except (json.JSONDecodeError, TypeError):
                            continue
                        record["kind"] = "frame"
                        record["camera"] = camera
                        self.observe(record)
                        sequence = record.get("capture_sequence")
                        if isinstance(sequence, int):
                            if previous_sequence is None:
                                sequence_drop_count += max(0, sequence)
                            else:
                                sequence_drop_count += max(
                                    0, sequence - previous_sequence - 1
                                )
                            previous_sequence = sequence
                state["capture_drop_count"] = max(
                    int(state["capture_drop_count"]), sequence_drop_count
                )
        finally:
            self.print_warnings = print_warnings

    def summary(self) -> dict[str, Any]:
        result: dict[str, Any] = {}
        thresholds = {
            "hardware_gap_warn_ms": self.hardware_gap_warn_ms,
            "grab_duration_warn_ms": self.grab_duration_warn_ms,
            "encoder_queue_delay_warn_ms": self.encoder_queue_delay_warn_ms,
        }
        for camera, state in self.data.items():
            result[camera] = {
                key: value for key, value in state.items() if key != "metrics"
            }
            result[camera]["warning_thresholds"] = thresholds
            result[camera]["timing"] = {
                metric: timing_summary(values)
                for metric, values in state["metrics"].items()
            }
        return result


class RosAgent:
    def __init__(
        self,
        section: str,
        config_path: Path,
        config: dict[str, Any],
        raw_dir: Path,
    ) -> None:
        self.section = section
        self.config_path = config_path
        self.config = config
        self.process: subprocess.Popen[str] | None = None
        self.latest: dict[str, dict[str, Any]] = {}
        self.errors: list[str] = []
        self.lock = threading.Lock()
        self.threads: list[threading.Thread] = []
        self.raw_dir = raw_dir
        self.raw_writers: dict[str, JsonlWriter] = {}

    def start(self) -> None:
        script = Path(__file__).with_name("ros_domain_agent.py")
        sources = " ; ".join(
            f"[ ! -f {shlex.quote(path)} ] || source {shlex.quote(path)}"
            for path in self.config["ros"].get("setup_files", [])
        )
        command = (
            f"{sources} ; exec /usr/bin/python3 {shlex.quote(str(script))} "
            f"--config {shlex.quote(str(self.config_path))} --section {shlex.quote(self.section)}"
        )
        self.process = subprocess.Popen(
            ["bash", "-lc", command],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        self.threads = [
            threading.Thread(target=self._read_stdout, daemon=True),
            threading.Thread(target=self._read_stderr, daemon=True),
        ]
        for thread in self.threads:
            thread.start()

    def _read_stdout(self) -> None:
        assert self.process is not None and self.process.stdout is not None
        for line in self.process.stdout:
            try:
                value = json.loads(line)
                stream = str(value.pop("stream"))
            except (json.JSONDecodeError, KeyError, TypeError):
                with self.lock:
                    self.errors.append(f"invalid stdout: {line.rstrip()}")
                continue
            with self.lock:
                self.latest[stream] = value
            if stream != "agent_status":
                writer = self.raw_writers.get(stream)
                if writer is None:
                    writer = JsonlWriter(self.raw_dir / f"{stream}.jsonl")
                    self.raw_writers[stream] = writer
                writer.write(value)

    def _read_stderr(self) -> None:
        assert self.process is not None and self.process.stderr is not None
        for line in self.process.stderr:
            if line.strip():
                with self.lock:
                    self.errors.append(line.rstrip())

    def snapshot(self) -> tuple[dict[str, dict[str, Any]], list[str]]:
        with self.lock:
            return dict(self.latest), self.errors[-20:]

    def ready(self) -> bool:
        snapshot, _ = self.snapshot()
        return snapshot.get("agent_status", {}).get("status") == "ready"

    def has_stream(self, stream: str) -> bool:
        snapshot, _ = self.snapshot()
        return stream in snapshot

    def diagnostics(self) -> dict[str, Any]:
        snapshot, errors = self.snapshot()
        section = self.config["ros"][self.section]
        return {
            "domain_id": section["domain_id"],
            "localhost_only": section["localhost_only"],
            "ready": snapshot.get("agent_status", {}).get("status") == "ready",
            "streams": sorted(name for name in snapshot if name != "agent_status"),
            "exit_code": self.process.poll() if self.process is not None else None,
            "errors": errors,
        }

    def stop(self) -> None:
        if self.process is not None and self.process.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGTERM)
                self.process.wait(timeout=3)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(self.process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        for thread in self.threads:
            thread.join(timeout=1)
        for writer in self.raw_writers.values():
            writer.close()


def safe_name(value: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", value.strip()).strip("._")
    return cleaned or "episode"


def episode_directory(root: Path, name: str | None) -> Path:
    now = dt.datetime.now().astimezone()
    suffix = f"_{safe_name(name)}" if name else ""
    base = root / now.strftime("%Y-%m-%d") / f"{now.strftime('%H%M%S')}{suffix}"
    candidate = base
    index = 1
    while candidate.exists():
        candidate = Path(f"{base}_{index:02d}")
        index += 1
    candidate.mkdir(parents=True)
    return candidate


def web_service_is_active() -> bool:
    result = subprocess.run(
        ["systemctl", "is-active", "--quiet", "svtrobo-web.service"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return result.returncode == 0


def drain_camera_queues(
    input_queues: dict[str, Any],
    synchronizer: CameraSynchronizer,
    errors: dict[str, str],
    dropped: dict[str, int] | None = None,
    diagnostics: CameraDiagnostics | None = None,
) -> None:
    for expected_camera, input_queue in input_queues.items():
        while True:
            try:
                item = input_queue.get_nowait()
            except queue.Empty:
                break
            kind = item.get("kind")
            camera = str(item.get("camera"))
            if camera != expected_camera:
                errors[expected_camera] = f"queue returned frame for {camera}"
                continue
            if diagnostics is not None:
                diagnostics.observe(item)
            if kind == "frame":
                synchronizer.add(item)
            elif kind == "error":
                errors[camera] = str(item.get("error"))
            elif kind == "capture_drop" and dropped is not None:
                dropped[camera] = max(
                    dropped.get(camera, 0), int(item.get("dropped_count", 0))
                )


def hand_record_is_ready(
    record: dict[str, Any] | None,
    config: dict[str, Any],
    reference_ns: int,
) -> bool:
    hand = decode_hand_record(record, config["hands"])
    stale_after_ms = float(config["hands"]["stale_after_ms"])
    return (
        hand["action"] is not None
        and hand["state"] is not None
        and hand["state_source"] == "hardware_feedback"
        and is_fresh(hand["timestamp_ns"], reference_ns, stale_after_ms)
        and is_fresh(hand["actual_timestamp_ns"], reference_ns, stale_after_ms)
    )


def save_camera_frame(
    episode_dir: Path,
    camera: str,
    sample: dict[str, Any],
    saved: dict[str, tuple[int, str]],
) -> dict[str, Any]:
    timestamp_ns = int(sample["host_timestamp_ns"])
    if sample.get("path"):
        relative = Path(str(sample["path"]))
    else:
        previous = saved.get(camera)
        if previous is None or previous[0] != timestamp_ns:
            relative = Path("images") / camera / f"{timestamp_ns}.jpg"
            output = episode_dir / relative
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_bytes(sample["jpeg"])
            saved[camera] = (timestamp_ns, relative.as_posix())
        else:
            relative = Path(previous[1])
    return {
        "path": relative.as_posix(),
        "host_timestamp_ns": timestamp_ns,
        "hardware_timestamp_ns": sample.get("hardware_timestamp_ns"),
        "hardware_timestamp_domain": sample.get("hardware_timestamp_domain"),
        "sync_timestamp_ns": sync_timestamp_ns(sample),
        "width": sample.get("width"),
        "height": sample.get("height"),
    }


def combine_arm(
    ros_snapshot: dict[str, dict[str, Any]],
    joint_names: list[str],
) -> tuple[dict[str, Any] | None, list[float] | None, str]:
    state = reorder_joint_state(ros_snapshot.get("arm_joint_state"), joint_names)
    actual = fixed_vector(state.get("position") if state else None, 14)
    left = prefix_vector(ros_snapshot.get("arm_command_left", {}).get("value"), 7)
    right = prefix_vector(ros_snapshot.get("arm_command_right", {}).get("value"), 7)
    if left is not None and right is not None:
        return state, left + right, "controller_command"
    return state, actual, "state_fallback" if actual is not None else "missing"


def build_frame(
    frame_index: int,
    frame_timestamp_ns: int,
    task: str,
    episode_dir: Path,
    config: dict[str, Any],
    arm_ros: dict[str, dict[str, Any]],
    base_ros: dict[str, dict[str, Any]],
    hand_record: dict[str, Any] | None,
    camera_selected: dict[str, Any],
    camera_sync_status: dict[str, Any],
    saved_images: dict[str, tuple[int, str]],
) -> dict[str, Any]:
    timing = config["timing"]
    arm_state, arm_action, arm_action_source = combine_arm(
        arm_ros, list(config["ros"]["arms"]["joint_names"])
    )
    arm_received_ns = arm_state.get("received_ns") if arm_state else None

    hand = decode_hand_record(hand_record, config["hands"])
    hand_action_age = age_ms(hand["timestamp_ns"], frame_timestamp_ns)
    hand_actual_age = age_ms(hand["actual_timestamp_ns"], frame_timestamp_ns)

    base_command = fixed_vector(base_ros.get("base_command", {}).get("value"), 3)
    base_action_source = "controller_command"
    if base_command is None:
        base_command = [0.0, 0.0, 0.0]
        base_action_source = "zero_fallback"
    base_feedback_sample = base_ros.get("base_feedback")
    base_feedback = fixed_vector(base_feedback_sample.get("value") if base_feedback_sample else None, 3)
    base_state_source = "cmd_feedback"
    if base_feedback is None:
        base_feedback = [0.0, 0.0, 0.0]
        base_state_source = "zero_fallback"

    images: dict[str, Any] = {}
    camera_validity: dict[str, bool] = {}
    camera_ages: dict[str, float | None] = {}
    for camera, camera_config in config["cameras"].items():
        if not camera_config.get("enabled", True):
            continue
        sample = camera_selected.get(camera)
        if sample is None:
            images[camera] = None
            camera_validity[camera] = False
            camera_ages[camera] = None
            continue
        images[camera] = save_camera_frame(episode_dir, camera, sample, saved_images)
        camera_timestamp_ns = sync_timestamp_ns(sample)
        camera_ages[camera] = age_ms(camera_timestamp_ns, frame_timestamp_ns)
        camera_validity[camera] = is_fresh(
            camera_timestamp_ns, frame_timestamp_ns, float(timing["camera_stale_after_ms"])
        )

    arm_fresh = is_fresh(
        arm_received_ns, frame_timestamp_ns, float(timing["arm_stale_after_ms"])
    )
    hand_action_fresh = is_fresh(
        hand["timestamp_ns"], frame_timestamp_ns, float(config["hands"]["stale_after_ms"])
    )
    hand_actual_fresh = is_fresh(
        hand["actual_timestamp_ns"],
        frame_timestamp_ns,
        float(config["hands"]["stale_after_ms"]),
    )
    base_received_ns = base_feedback_sample.get("received_ns") if base_feedback_sample else None
    base_fresh = is_fresh(
        base_received_ns, frame_timestamp_ns, float(timing["base_stale_after_ms"])
    )
    wrist_pair = camera_sync_status["wrist_pair"]
    valid = (
        all(camera_validity.values())
        and bool(wrist_pair["valid"])
        and arm_fresh
        and hand_action_fresh
        and hand_actual_fresh
        and arm_action is not None
        and hand["action"] is not None
        and hand["state"] is not None
        and hand["state_source"] == "hardware_feedback"
    )

    return {
        "frame_index": frame_index,
        "timestamp_ns": frame_timestamp_ns,
        "task": task,
        "observation": {
            "images": images,
            "state": {
                "arm_position": arm_state.get("position") if arm_state else None,
                "arm_velocity": arm_state.get("velocity") if arm_state else None,
                "arm_effort": arm_state.get("effort") if arm_state else None,
                "hand_position": hand["state"],
                # This is velocity feedback/echo, not odometry or a base pose.
                "base_velocity": base_feedback,
            },
        },
        "action": {
            "arm_position": arm_action,
            "hand_position": hand["action"],
            "base_velocity": base_command,
        },
        "source": {
            "arm_action": arm_action_source,
            "hand_state": hand["state_source"],
            "base_action": base_action_source,
            "base_state": base_state_source,
        },
        "source_timestamps_ns": {
            "arm": arm_received_ns,
            "hand_action": hand["timestamp_ns"],
            "hand_actual": hand["actual_timestamp_ns"],
            "base_feedback": base_received_ns,
            "base_command": base_ros.get("base_command", {}).get("received_ns"),
        },
        "age_ms": {
            "arm": age_ms(arm_received_ns, frame_timestamp_ns),
            "hand": max(
                value
                for value in (hand_action_age, hand_actual_age)
                if isinstance(value, (int, float))
            )
            if any(
                isinstance(value, (int, float))
                for value in (hand_action_age, hand_actual_age)
            )
            else None,
            "hand_action": hand_action_age,
            "hand_actual": hand_actual_age,
            "base_feedback": age_ms(base_received_ns, frame_timestamp_ns),
            "cameras": camera_ages,
        },
        "validity": {
            "valid_for_training": valid,
            "arm_fresh": arm_fresh,
            "hand_fresh": hand_action_fresh and hand_actual_fresh,
            "hand_action_fresh": hand_action_fresh,
            "hand_actual_fresh": hand_actual_fresh,
            "base_feedback_fresh": base_fresh,
            "cameras_fresh": camera_validity,
            "wrist_pair_valid": bool(wrist_pair["valid"]),
            "wrist_pair_skew_ms": wrist_pair["skew_ms"],
            "wrist_pair_reason": wrist_pair["reason"],
        },
        "raw": {
            "hand": hand["raw"],
            "base_feedback": base_feedback_sample,
            "base_command": base_ros.get("base_command"),
            "base_joint_state": base_ros.get("base_joint_state"),
        },
    }


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def resolve_collection_metadata(
    args: argparse.Namespace, config_path: Path
) -> tuple[str, dict[str, Any]]:
    task = str(args.task or "").strip()
    skill_id = str(args.skill_id or "").strip() or None
    skills_path = args.skills_config.expanduser().resolve()
    skill: dict[str, Any] | None = None
    allow_base_motion = bool(getattr(args, "allow_base_motion", False))
    concurrent_base_prior: str | None = None
    if skill_id is not None:
        if not skills_path.is_file():
            raise FileNotFoundError(f"skills config does not exist: {skills_path}")
        skills_document = yaml.safe_load(skills_path.read_text(encoding="utf-8"))
        skills = skills_document.get("skills") if isinstance(skills_document, dict) else None
        if not isinstance(skills, dict) or skill_id not in skills:
            raise ValueError(f"skill_id {skill_id!r} is not defined in {skills_path}")
        skill = skills[skill_id]
        if not isinstance(skill, dict):
            raise ValueError(f"skill {skill_id} must be a mapping")
        configured_task = str(skill.get("prompt", "")).strip()
        if not configured_task:
            raise ValueError(f"skill {skill_id} has no prompt")
        if task and task != configured_task:
            raise ValueError(
                f"--task does not exactly match the versioned prompt for {skill_id}: {configured_task!r}"
            )
        task = configured_task
        configured_version = str(skill.get("prompt_version", "")).strip()
        if not configured_version:
            raise ValueError(f"skill {skill_id} has no prompt_version")
        if args.prompt_version and args.prompt_version != configured_version:
            raise ValueError(
                f"--prompt-version {args.prompt_version!r} does not match {configured_version!r}"
            )
        args.prompt_version = configured_version
        expected_hand_mode = str(skill.get("collection_hand_mode", "")).strip()
        if expected_hand_mode and args.hand_control_mode != expected_hand_mode:
            raise ValueError(
                f"{skill_id} requires --hand-control-mode {expected_hand_mode!r}, "
                f"got {args.hand_control_mode!r}"
            )
        concurrent_base_prior = str(skill.get("concurrent_base_prior", "")).strip() or None
        if allow_base_motion and concurrent_base_prior is None:
            raise ValueError(
                f"{skill_id} does not authorize --allow-base-motion in {skills_path}"
            )
        if concurrent_base_prior is not None and not allow_base_motion:
            raise ValueError(
                f"{skill_id} requires --allow-base-motion because it runs with "
                f"base prior {concurrent_base_prior}"
            )
    elif allow_base_motion:
        raise ValueError("--allow-base-motion requires a versioned --skill-id")
    if not task:
        raise ValueError("provide --skill-id for a versioned skill or a non-empty --task")

    checksum_paths = {"collector_config": config_path}
    if skill_id is not None:
        checksum_paths["skills_config"] = skills_path
    hand_config = args.hand_config.expanduser().resolve() if args.hand_config else None
    if hand_config is not None:
        if not hand_config.is_file():
            raise FileNotFoundError(f"hand config does not exist: {hand_config}")
        checksum_paths["hand_config"] = hand_config
    checksums = {
        name: {"path": str(path), "sha256": sha256_file(path)}
        for name, path in checksum_paths.items()
    }
    metadata = {
        "skill_id": skill_id,
        "prompt_version": args.prompt_version,
        "attempt_outcome": args.attempt_outcome,
        "perturbation": args.perturbation,
        "hand_control_mode": args.hand_control_mode,
        "operator": args.operator,
        "config_checksums": checksums,
        "base_motion": {
            "allowed": allow_base_motion,
            "concurrent_prior": concurrent_base_prior,
            "included_in_training_action": False,
        },
    }
    if skill is not None:
        metadata["skill_definition"] = {
            "allowed_executors": skill.get("allowed_executors"),
            "standard_end_state": skill.get("standard_end_state"),
            "concurrent_base_prior": concurrent_base_prior,
        }
    return task, metadata


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record one synchronized raw episode for LingBot VLA.")
    parser.add_argument("--config", type=Path, default=Path(__file__).parent / "config" / "svt.yaml")
    parser.add_argument("--task", help="Natural-language instruction; derived from --skill-id when omitted.")
    parser.add_argument("--skill-id", choices=("S1", "S2", "S3", "S4", "S5", "S6"))
    parser.add_argument(
        "--skills-config",
        type=Path,
        default=Path(__file__).parent / "config" / "skills.yaml",
    )
    parser.add_argument("--prompt-version")
    parser.add_argument(
        "--attempt-outcome",
        choices=("unreviewed", "success", "failure", "aborted"),
        default="unreviewed",
    )
    parser.add_argument(
        "--perturbation",
        choices=("standard", "object_pose", "end_effector", "base_residual", "combined"),
        default="standard",
    )
    parser.add_argument(
        "--hand-control-mode",
        choices=("button_primitive", "s5_index_only", "full_glove", "manual"),
        default="button_primitive",
    )
    parser.add_argument("--operator", default=os.environ.get("USER", "unknown"))
    parser.add_argument(
        "--allow-base-motion",
        action="store_true",
        help=(
            "Permit a nonzero audited base command only for a versioned skill that "
            "declares concurrent_base_prior; base is never added to VLA action labels."
        ),
    )
    parser.add_argument(
        "--hand-config",
        type=Path,
        default=Path("/home/svt/glove_control/wuji_bridge/dual_hand_config.json"),
    )
    parser.add_argument(
        "--episode-name",
        help="Optional suffix; omit it to name the episode with the current HHMMSS time.",
    )
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds; 0 records until Ctrl-C.")
    parser.add_argument("--output-root", type=Path, help="Override config output_root.")
    parser.add_argument("--mock-cameras", action="store_true", help="Use generated frames for testing.")
    parser.add_argument("--no-cameras", action="store_true")
    parser.add_argument("--no-ros", action="store_true")
    parser.add_argument(
        "--no-offline-align",
        action="store_true",
        help="Keep raw streams but do not generate nearest-timestamp frames.jsonl.",
    )
    parser.add_argument(
        "--allow-missing-cameras",
        action="store_true",
        help="Continue if a camera fails; affected frames are invalid for training.",
    )
    parser.add_argument(
        "--ignore-web-service",
        action="store_true",
        help="Try opening ZED even when svtrobo-web is active.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config_path = args.config.resolve()
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    task, collection_metadata = resolve_collection_metadata(args, config_path)
    args.task = task
    if args.mock_cameras:
        for camera in config["cameras"].values():
            camera["backend"] = "mock"
    if args.no_cameras:
        for camera in config["cameras"].values():
            camera["enabled"] = False

    output_root = (args.output_root or Path(config["output_root"])).expanduser()
    output_root.mkdir(parents=True, exist_ok=True)
    free_gb = shutil.disk_usage(output_root).free / (1024**3)
    if free_gb < float(config.get("minimum_free_gb", 20)):
        raise RuntimeError(f"only {free_gb:.1f} GiB free under {output_root}")

    has_web_camera_conflict = any(
        value.get("enabled", True)
        and value.get("backend") != "mock"
        and value.get("conflicts_service") == "svtrobo-web.service"
        for value in config["cameras"].values()
    )
    if has_web_camera_conflict and not args.ignore_web_service and web_service_is_active():
        raise RuntimeError(
            "svtrobo-web.service is active and owns the ZED camera; stop it first with "
            "'sudo systemctl stop svtrobo-web.service'"
        )

    episode_dir = episode_directory(output_root, args.episode_name)
    episode_config_path = episode_dir / "config.yaml"
    episode_config_path.write_text(
        yaml.safe_dump(config, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    collection_metadata["config_checksums"]["episode_config_snapshot"] = {
        "path": "config.yaml",
        "sha256": sha256_file(episode_config_path),
    }
    started_ns = time.time_ns()
    manifest: dict[str, Any] = {
        "schema_version": int(config["schema_version"]),
        "collector_version": COLLECTOR_VERSION,
        "status": "recording",
        "robot_id": config["robot_id"],
        "task": task,
        "skill_id": collection_metadata["skill_id"],
        "prompt_version": collection_metadata["prompt_version"],
        "attempt_outcome": collection_metadata["attempt_outcome"],
        "perturbation": collection_metadata["perturbation"],
        "hand_control_mode": collection_metadata["hand_control_mode"],
        "operator": collection_metadata["operator"],
        "config_checksums": collection_metadata["config_checksums"],
        "collection_metadata": collection_metadata,
        "base_motion": collection_metadata["base_motion"],
        "fps": int(config["fps"]),
        "started_ns": started_ns,
        "started_at": dt.datetime.now().astimezone().isoformat(),
        "frame_count": 0,
        "valid_frame_count": 0,
        "camera_errors": {},
        "camera_dropped_before_encoding": {},
        "camera_diagnostics": {},
        "wrist_pair_rejected_count": 0,
        "ros_errors": {},
        "camera_sync": config["timing"]["wrist_pair"],
        "raw_recording": {"enabled": True, "root": "raw"},
        "hand_layout": {
            "left": config["hands"]["left_names"],
            "right": [item["name"] for item in config["hands"]["right_virtual_projection"]],
            "right_raw_order": [
                f"{finger}.j{joint}"
                for finger in ("thumb", "index", "middle", "ring", "little")
                for joint in range(4)
            ],
            "right_virtual_projection": config["hands"]["right_virtual_projection"],
        },
    }
    atomic_write_json(episode_dir / "manifest.json", manifest)

    stop_event = mp.Event()
    camera_processes: list[mp.Process] = []
    enabled_cameras = [
        name for name, value in config["cameras"].items() if value.get("enabled", True)
    ]
    zed_camera = next(
        (
            name
            for name in enabled_cameras
            if str(config["cameras"][name].get("backend", "")).lower() == "zed"
        ),
        None,
    )
    camera_queues: dict[str, Any] = {
        name: mp.Queue(maxsize=max(8, int(config["cameras"][name]["fps"])))
        for name in enabled_cameras
    }
    camera_synchronizer = CameraSynchronizer(
        enabled_cameras,
        max_frames=int(config["timing"]["camera_buffer_frames"]),
    )
    camera_errors: dict[str, str] = {}
    camera_dropped_before_encoding: dict[str, int] = {}
    camera_diagnostics = CameraDiagnostics(
        config["cameras"], config.get("camera_diagnostics")
    )
    saved_images: dict[str, tuple[int, str]] = {}
    camera_configs: dict[str, dict[str, Any]] = {}
    for name, camera_config in config["cameras"].items():
        if not camera_config.get("enabled", True):
            continue
        camera_config = dict(camera_config)
        camera_config["jpeg_quality"] = int(config.get("jpeg_quality", 92))
        camera_config["encoder_queue_frames"] = int(
            config.get("camera_encoder_queue_frames", 8)
        )
        camera_config["encoder_sentinel_timeout_sec"] = float(
            config.get("camera_encoder_sentinel_timeout_sec", 0.5)
        )
        camera_config["encoder_shutdown_timeout_sec"] = float(
            config.get("camera_encoder_shutdown_timeout_sec", 2.0)
        )
        camera_config["diagnostics"] = dict(config.get("camera_diagnostics", {}))
        camera_configs[name] = camera_config

    zed_ready_event = mp.Event() if zed_camera is not None else None
    launch_order = (
        [zed_camera] + [name for name in enabled_cameras if name != zed_camera]
        if zed_camera is not None
        else enabled_cameras
    )
    for name in launch_order:
        camera_config = camera_configs[name]
        ready_event = zed_ready_event if name == zed_camera else None
        process = mp.Process(
            target=camera_worker,
            name=f"camera-{name}",
            args=(
                name,
                camera_config,
                camera_queues[name],
                stop_event,
                str(episode_dir),
                ready_event,
            ),
        )
        process.start()
        camera_processes.append(process)
        if name == zed_camera and zed_ready_event is not None:
            gate_deadline = time.monotonic() + float(
                config.get("zed_startup_gate_timeout_sec", 12.0)
            )
            while (
                process.is_alive()
                and not zed_ready_event.is_set()
                and time.monotonic() < gate_deadline
            ):
                zed_ready_event.wait(timeout=0.05)
            if zed_ready_event.is_set():
                print(
                    "[camera-startup] ZED ready; starting wrist cameras.",
                    flush=True,
                )
            else:
                print(
                    "[camera-warning] ZED did not become ready before wrist-camera "
                    "startup.",
                    flush=True,
                )

    ros_agents: dict[str, RosAgent] = {}
    if not args.no_ros:
        for section in ("arms", "base"):
            agent = RosAgent(
                section,
                config_path,
                config,
                episode_dir / "raw" / "ros",
            )
            agent.start()
            ros_agents[section] = agent

    hand_source = TeleopLogSource(config["hands"]["log_glob"], config["hands"])
    hand_writer = JsonlWriter(episode_dir / "raw" / "hands" / "teleop.jsonl")
    last_hand_timestamp_ns: int | None = None
    startup_deadline = time.monotonic() + float(config["timing"]["startup_timeout_sec"])
    frame_count = 0
    valid_count = 0
    wrist_pair_rejected_count = 0
    hand_unready_since_ns: int | None = None
    try:
        while time.monotonic() < startup_deadline:
            drain_camera_queues(
                camera_queues,
                camera_synchronizer,
                camera_errors,
                camera_dropped_before_encoding,
                camera_diagnostics,
            )
            cameras_ready = all(
                not value.get("enabled", True)
                or camera_synchronizer.has_any(name)
                or name in camera_errors
                for name, value in config["cameras"].items()
            )
            ros_ready = args.no_ros or (
                all(agent.ready() for agent in ros_agents.values())
                and ros_agents["arms"].has_stream("arm_joint_state")
                and ros_agents["arms"].has_stream("arm_command_left")
                and ros_agents["arms"].has_stream("arm_command_right")
                and ros_agents["base"].has_stream("base_feedback")
            )
            hand_record = hand_source.poll()
            hand_ready = hand_record_is_ready(hand_record, config, time.time_ns())
            if cameras_ready and ros_ready and hand_ready:
                break
            time.sleep(0.05)

        missing_cameras = [
            name for name in enabled_cameras if not camera_synchronizer.has_any(name)
        ]
        if missing_cameras and not args.allow_missing_cameras:
            details = "; ".join(
                f"{name}: {camera_errors.get(name, 'no frame before startup timeout')}"
                for name in missing_cameras
            )
            raise RuntimeError(f"camera startup failed: {details}")
        ros_data_ready = args.no_ros or (
            all(agent.ready() for agent in ros_agents.values())
            and ros_agents["arms"].has_stream("arm_joint_state")
            and ros_agents["arms"].has_stream("arm_command_left")
            and ros_agents["arms"].has_stream("arm_command_right")
            and ros_agents["base"].has_stream("base_feedback")
        )
        if not ros_data_ready:
            details = {name: agent.diagnostics() for name, agent in ros_agents.items()}
            raise RuntimeError(f"ROS agent startup failed: {details}")
        hand_record = hand_source.poll()
        hand_data_ready = hand_record_is_ready(hand_record, config, time.time_ns())
        if not hand_data_ready:
            raise RuntimeError(
                "hand data startup failed: no fresh command plus hardware feedback; "
                "start the hand controller and verify both hands report actual state"
            )

        # Raw sources retain startup samples for diagnosis. Offline alignment
        # starts here, after every required source has produced real data.
        manifest["recording_start_ns"] = time.time_ns()
        manifest["recording_started_at"] = dt.datetime.now().astimezone().isoformat()
        atomic_write_json(episode_dir / "manifest.json", manifest)

        fps = float(config["fps"])
        period_ns = int(1_000_000_000 / fps)
        next_tick_ns = time.monotonic_ns()
        deadline = time.monotonic() + args.duration if args.duration > 0 else None
        frames_path = episode_dir / "frames.jsonl"
        with frames_path.open("w", encoding="utf-8", buffering=1) as output:
            while deadline is None or time.monotonic() < deadline:
                wrist_config = config["timing"]["wrist_pair"]
                pair_wait_deadline = time.monotonic() + float(
                    config["timing"]["camera_pair_wait_ms"]
                ) / 1000.0
                while True:
                    drain_camera_queues(
                        camera_queues,
                        camera_synchronizer,
                        camera_errors,
                        camera_dropped_before_encoding,
                        camera_diagnostics,
                    )
                    wait_reference_ns = time.time_ns()
                    camera_synchronizer.prune(
                        wait_reference_ns,
                        float(config["timing"]["camera_buffer_retention_ms"]),
                    )
                    left = str(wrist_config["left_camera"])
                    right = str(wrist_config["right_camera"])
                    wrists_enabled = left in enabled_cameras and right in enabled_cameras
                    wrists_ready = not wrists_enabled or (
                        camera_synchronizer.has_unselected(
                            left, wait_reference_ns, float(wrist_config["max_age_ms"])
                        )
                        and camera_synchronizer.has_unselected(
                            right, wait_reference_ns, float(wrist_config["max_age_ms"])
                        )
                    )
                    all_cameras_ready = wrists_ready and all(
                        camera in (left, right)
                        or camera_synchronizer.has_unselected(
                            camera,
                            wait_reference_ns,
                            float(config["timing"]["camera_stale_after_ms"]),
                        )
                        for camera in enabled_cameras
                    )
                    if all_cameras_ready or time.monotonic() >= pair_wait_deadline:
                        break
                    time.sleep(0.001)

                frame_timestamp_ns = time.time_ns()
                camera_selected, camera_sync_status = camera_synchronizer.select(
                    frame_timestamp_ns,
                    enabled_cameras,
                    float(config["timing"]["camera_stale_after_ms"]),
                    wrist_config,
                )
                arm_snapshot = ros_agents.get("arms").snapshot()[0] if "arms" in ros_agents else {}
                base_snapshot = ros_agents.get("base").snapshot()[0] if "base" in ros_agents else {}
                record = hand_source.poll()
                hand_check_ns = time.time_ns()
                if hand_record_is_ready(record, config, hand_check_ns):
                    hand_unready_since_ns = None
                elif hand_unready_since_ns is None:
                    hand_unready_since_ns = hand_check_ns
                elif (hand_check_ns - hand_unready_since_ns) / 1_000_000.0 >= float(
                    config["hands"].get("runtime_abort_after_ms", 1000)
                ):
                    raise RuntimeError(
                        "hand hardware feedback was unavailable for longer than "
                        f"{config['hands'].get('runtime_abort_after_ms', 1000)} ms"
                    )
                new_hand_records = hand_source.drain_pending()
                if record is not None and last_hand_timestamp_ns is None:
                    new_hand_records.insert(0, record)
                for hand_item in new_hand_records:
                    timestamp_ns = int(hand_item["_timestamp_ns"])
                    if last_hand_timestamp_ns is None or timestamp_ns > last_hand_timestamp_ns:
                        hand_writer.write(hand_item)
                        last_hand_timestamp_ns = timestamp_ns
                frame = build_frame(
                    frame_count,
                    frame_timestamp_ns,
                    args.task,
                    episode_dir,
                    config,
                    arm_snapshot,
                    base_snapshot,
                    record,
                    camera_selected,
                    camera_sync_status,
                    saved_images,
                )
                if (
                    args.skill_id is not None
                    and not collection_metadata["base_motion"]["allowed"]
                    and any(
                        abs(float(value)) > 1e-4
                        for value in frame["action"]["base_velocity"]
                    )
                ):
                    raise RuntimeError(
                        "atomic-skill collection observed a nonzero base command; "
                        "stop the base publisher and discard this attempt"
                    )
                output.write(json.dumps(frame, ensure_ascii=False, separators=(",", ":")) + "\n")
                frame_count += 1
                valid_count += int(frame["validity"]["valid_for_training"])
                wrist_pair_rejected_count += int(
                    not frame["validity"]["wrist_pair_valid"]
                )
                if frame_count % max(1, int(fps)) == 0:
                    wrist_status = frame["validity"]
                    zed_status = (
                        camera_diagnostics.live_status(zed_camera)
                        if zed_camera is not None
                        else None
                    )
                    zed_text = ""
                    if zed_status is not None:
                        zed_text = (
                            f" zed_grab_errors={zed_status['zed_grab_errors']}"
                            f" zed_gaps={zed_status['hardware_gaps']}"
                            f" zed_encoder_drops={zed_status['encoder_drops']}"
                            f" zed_queue_max_ms="
                            f"{float(zed_status['encoder_queue_max_ms']):.2f}"
                        )
                    print(
                        f"frames={frame_count} valid={valid_count} "
                        f"camera_errors={len(camera_errors)} "
                        f"wrist_pair={wrist_status['wrist_pair_reason']} "
                        f"skew_ms={wrist_status['wrist_pair_skew_ms']}"
                        f"{zed_text}",
                        flush=True,
                    )

                next_tick_ns += period_ns
                delay = (next_tick_ns - time.monotonic_ns()) / 1_000_000_000
                if delay > 0:
                    time.sleep(delay)
                elif delay < -period_ns / 1_000_000_000:
                    next_tick_ns = time.monotonic_ns()

        recording_end_ns = time.time_ns()
        manifest.update(
            {
                "status": "complete",
                "ended_ns": recording_end_ns,
                "recording_end_ns": recording_end_ns,
                "ended_at": dt.datetime.now().astimezone().isoformat(),
                "frame_count": frame_count,
                "valid_frame_count": valid_count,
                "wrist_pair_rejected_count": wrist_pair_rejected_count,
                "camera_errors": camera_errors,
                "camera_dropped_before_encoding": camera_dropped_before_encoding,
                "ros_errors": {
                    name: agent.snapshot()[1] for name, agent in ros_agents.items()
                },
            }
        )
        atomic_write_json(episode_dir / "manifest.json", manifest)
        print(f"episode={episode_dir}")
        return 0
    except KeyboardInterrupt:
        manifest["status"] = "complete"
        manifest["ended_ns"] = time.time_ns()
        manifest["recording_end_ns"] = manifest["ended_ns"]
        manifest["ended_at"] = dt.datetime.now().astimezone().isoformat()
        manifest["frame_count"] = frame_count
        manifest["valid_frame_count"] = valid_count
        manifest["wrist_pair_rejected_count"] = wrist_pair_rejected_count
        manifest["camera_errors"] = camera_errors
        manifest["camera_dropped_before_encoding"] = camera_dropped_before_encoding
        manifest["ros_errors"] = {
            name: agent.snapshot()[1] for name, agent in ros_agents.items()
        }
        atomic_write_json(episode_dir / "manifest.json", manifest)
        print(f"\ncomplete episode={episode_dir}")
        return 0
    except BaseException as exc:
        manifest["status"] = "failed"
        manifest["ended_ns"] = time.time_ns()
        manifest["recording_end_ns"] = manifest["ended_ns"]
        manifest["frame_count"] = frame_count
        manifest["valid_frame_count"] = valid_count
        manifest["wrist_pair_rejected_count"] = wrist_pair_rejected_count
        manifest["camera_errors"] = camera_errors
        manifest["camera_dropped_before_encoding"] = camera_dropped_before_encoding
        manifest["error"] = f"{type(exc).__name__}: {exc}"
        atomic_write_json(episode_dir / "manifest.json", manifest)
        raise
    finally:
        final_hand_record = hand_source.poll()
        final_hand_records = hand_source.drain_pending()
        if final_hand_record is not None and last_hand_timestamp_ns is None:
            final_hand_records.insert(0, final_hand_record)
        for hand_item in final_hand_records:
            timestamp_ns = int(hand_item["_timestamp_ns"])
            if last_hand_timestamp_ns is None or timestamp_ns > last_hand_timestamp_ns:
                hand_writer.write(hand_item)
                last_hand_timestamp_ns = timestamp_ns
        hand_source.close()
        hand_writer.close()
        stop_event.set()
        camera_shutdown_deadline = time.monotonic() + float(
            config.get("camera_shutdown_timeout_sec", 4.0)
        )
        for process in camera_processes:
            process.join(
                timeout=max(0.0, camera_shutdown_deadline - time.monotonic())
            )
        for process in camera_processes:
            if process.is_alive():
                print(
                    f"[camera-warning] shutdown deadline exceeded: {process.name}",
                    flush=True,
                )
                process.terminate()
        for process in camera_processes:
            if process.is_alive():
                process.join(timeout=1)
        drain_camera_queues(
            camera_queues,
            camera_synchronizer,
            camera_errors,
            camera_dropped_before_encoding,
            camera_diagnostics,
        )
        camera_diagnostics.rebuild_from_raw(episode_dir)
        manifest["camera_errors"] = camera_errors
        manifest["camera_dropped_before_encoding"] = camera_dropped_before_encoding
        manifest["camera_diagnostics"] = camera_diagnostics.summary()
        atomic_write_json(episode_dir / "manifest.json", manifest)
        for camera, summary in manifest["camera_diagnostics"].items():
            timing = summary["timing"]
            print(
                f"[camera-summary] {camera} "
                f"frames={summary['frame_count_observed']} "
                f"grab_errors={summary['zed_grab_error_count']} "
                f"hardware_gaps={summary['hardware_gap_warning_count']} "
                f"estimated_missing={summary['estimated_missing_frames']} "
                f"encoder_drops={summary['capture_drop_count']} "
                f"queue_p90_ms={timing['encoder_queue_delay_ms']['p90']} "
                f"queue_max_ms={timing['encoder_queue_delay_ms']['max']}",
                flush=True,
            )
        for agent in ros_agents.values():
            agent.stop()
        if not args.no_offline_align and frame_count > 0:
            try:
                from offline_align import align_episode

                align_episode(episode_dir, config)
            except BaseException as exc:
                manifest["offline_alignment_error"] = f"{type(exc).__name__}: {exc}"
                atomic_write_json(episode_dir / "manifest.json", manifest)
                print(f"offline alignment failed: {exc}", file=sys.stderr, flush=True)


if __name__ == "__main__":
    mp.freeze_support()
    sys.exit(main())
