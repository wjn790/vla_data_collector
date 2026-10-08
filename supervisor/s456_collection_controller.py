from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import queue
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any


COLLECTOR_ROOT = Path(__file__).resolve().parents[1]
if str(COLLECTOR_ROOT) not in sys.path:
    sys.path.insert(0, str(COLLECTOR_ROOT))

from supervisor.core import SafetyStop, load_yaml
from supervisor.paired_collection_controller import (
    EventLog,
    FileLease,
    ManagedServiceTakeover,
    PairedCollectionRuntime,
)
from supervisor.s456_collection_core import (
    DEFAULT_HAND_MODE,
    SEGMENTS_FILE,
    SUPPORTED_HAND_MODES,
    WORKFLOW,
    ButtonMapping,
    S456Action,
    S456State,
    S456StateMachine,
    build_s456_segments_document,
    hand_mode_for_state,
    validate_config,
)

START_SKILL = "S4"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Button-driven continuous S4-S5-S6 collection. The base is latched at "
            "zero for the whole take; B0-B3 priors are not used and the robot is "
            "positioned manually before recording."
        )
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=COLLECTOR_ROOT / "config" / "s456_collection.yaml",
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--acknowledge-motion")
    return parser.parse_args()


def resolve_path(config_path: Path, value: Any) -> Path:
    path = Path(str(value)).expanduser()
    return path if path.is_absolute() else (config_path.parent / path).resolve()


def load_all(config_path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    config = load_yaml(config_path)
    validate_config(config)
    collector_path = resolve_path(config_path, config["collector_config"])
    collector = load_yaml(collector_path)
    if config["robot_id"] != collector["robot_id"]:
        raise ValueError("robot_id differs between S456 and collector configs")
    if int(config["timing"]["hz"]) != 15:
        raise ValueError("S456 collection must run at 15 Hz")
    config["collector_config"] = str(collector_path)
    collection = dict(config["collection"])
    collection["skills_config"] = str(
        resolve_path(config_path, collection.get("skills_config", "skills.yaml"))
    )
    config["collection"] = collection
    config["hand_mode_file"] = str(resolve_path(config_path, config["hand_mode_file"]))
    skills_document = load_yaml(Path(collection["skills_config"]))
    skills = skills_document.get("skills") if isinstance(skills_document, dict) else None
    if not isinstance(skills, dict):
        raise ValueError(f"skills config has no skills mapping: {collection['skills_config']}")
    return config, skills


def dry_run(config_path: Path) -> dict[str, Any]:
    config, skills = load_all(config_path)
    machine = S456StateMachine()
    trace = [machine.state.value]
    machine.press("right_a")
    machine.collector_started("S4")
    trace.append(machine.state.value)
    machine.press("right_b")
    trace.append(machine.state.value)
    machine.press("right_b")
    trace.append(machine.state.value)
    machine.press("right_a")
    machine.collector_stopped("S4")
    trace.append(machine.state.value)
    machine.reset_completed_attempt()
    trace.append(machine.state.value)

    # A fresh probe machine exercises the ignored-press table.
    probe = S456StateMachine()
    probe.press("right_a")
    invalid_a_in_s4 = probe.press("right_a")
    invalid_b_in_s6 = None
    probe.press("right_b")
    probe.press("right_b")
    invalid_b_in_s6 = probe.press("right_b")
    idle_d = S456StateMachine().press("right_d")
    segment_modes = {
        skill_id: str(skills[skill_id].get("collection_hand_mode"))
        for skill_id in ("S4", "S5", "S6")
    }
    return {
        "mode": "dry_run",
        "robot_id": config["robot_id"],
        "buttons": {
            "right_a": (
                "first press starts the continuous S456 recording and prints "
                "[S456 READY]; press again after S6 to stop and finalize segments"
            ),
            "right_b": (
                "first press marks the S4/S5 boundary and switches the glove teleop "
                "to s5_index_only; second press marks S5/S6 and restores full_glove"
            ),
            "right_c": "latched emergency stop",
            "right_d": (
                "rewrite the expected hand mode to the teleop control-mode file "
                "(escape hatch if a mode switch was missed)"
            ),
            "left_c": (
                "not used by this controller; the mixed glove teleop process owns "
                "the dual-hand lease and both hands follow the gloves"
            ),
        },
        "excluded_motion": {
            "B0_B3_priors": True,
            "note": "no base prior is loaded; the base is only latched at zero",
        },
        "base_publisher_takeover": {
            "startup": "F710 remains active while idle",
            "acquire": "first right A",
            "release": "after recording ends or aborts",
        },
        "hand_mode_file": config["hand_mode_file"],
        "segment_hand_modes": segment_modes,
        "state_trace": trace,
        "ignored_presses": {
            "right_a_during_s4": invalid_a_in_s4,
            "right_b_during_s6": invalid_b_in_s6,
            "right_d_while_idle": idle_d,
        },
        "motion_published": False,
        "ready_for_real_execution": True,
    }


class S456CollectionRuntime(PairedCollectionRuntime):
    def __init__(
        self,
        config: dict[str, Any],
        skills: dict[str, dict[str, Any]],
        takeover: ManagedServiceTakeover,
    ) -> None:
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.executors import MultiThreadedExecutor
        from sensor_msgs.msg import Joy

        self.rclpy = rclpy
        self.Twist = Twist
        self.config = config
        self.priors = {"priors": {}}
        self.collector = {}
        self.prior_streams = {}
        self.takeover = takeover
        for skill_id in ("S4", "S5", "S6"):
            if not isinstance(skills.get(skill_id), dict):
                raise RuntimeError(f"S456 collection requires the {skill_id} skill definition")
        self.skills: dict[str, dict[str, Any]] = skills
        self.machine = S456StateMachine()
        self.machine_lock = threading.Lock()
        self.shutdown_event = threading.Event()
        self.emergency_event = threading.Event()
        self.base_control_active = threading.Event()
        self.actions: queue.Queue[tuple[str, S456State] | str] = queue.Queue()
        self.feedback_lock = threading.Lock()
        self.base_feedback: tuple[float, float, float] | None = None
        self.base_received_at = 0.0
        self.arm_position = None
        self.arm_effort = None
        self.arm_received_at = 0.0
        self.previous_buttons: list[int] | None = None
        self.last_button_at = {"right_a": 0.0, "right_b": 0.0, "right_c": 0.0, "right_d": 0.0}
        self.collector_lock = threading.Lock()
        self.collector_process: subprocess.Popen[Any] | None = None
        self.collector_log_handle: Any | None = None
        self.collector_log_path: Path | None = None
        self.collector_episode_path: Path | None = None
        self.collector_started_at = 0.0
        self.collector_recording_start_ns: int | None = None
        self.current_skill: str | None = None
        self.hand_node = None
        self.hand_lease = None
        self.executor: Any | None = None
        self.managed_base_publisher_gids: set[bytes] = set()
        self.attempt_id: str | None = None
        self.attempt_path: Path | None = None
        self.attempt_record: dict[str, Any] = {}
        self.optional_feedback_logged: set[str] = set()
        self.event_log = EventLog(Path(str(config["event_log"])).expanduser())

        os.environ["ROS_DOMAIN_ID"] = str(config["ros"]["domain_id"])
        os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node("svt_s456_collection_controller")
        time.sleep(0.25)
        self._remember_managed_base_publishers(before_own_publisher=True)
        ros = config["ros"]
        self.base_publisher = self.node.create_publisher(Twist, ros["base_command_topic"], 10)
        self.node.create_subscription(Joy, config["joy_topic"], self._joy_callback, 10)
        self.node.create_subscription(Twist, ros["base_feedback_topic"], self._base_callback, 10)
        self.zero_timer = self.node.create_timer(1.0 / 15.0, self._zero_timer)
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self.worker = threading.Thread(target=self._worker_loop, name="s456-collection", daemon=True)
        self.worker.start()
        self.event_log.write(
            "runtime_ready",
            state=self.machine.state.value,
            workflow=WORKFLOW,
            hand_mode_file=str(config["hand_mode_file"]),
            base_motion="zero_latch_only",
        )

    @property
    def state(self) -> S456State:
        with self.machine_lock:
            return self.machine.state

    def _zero_timer(self) -> None:
        if self.base_control_active.is_set():
            self._publish_zero()

    def _joy_callback(self, message: Any) -> None:
        buttons = [int(value) for value in message.buttons]
        mapping: ButtonMapping = validate_config(self.config)
        required = max(mapping.right_a, mapping.right_b, mapping.right_c, mapping.right_d)
        if len(buttons) <= required:
            return
        if self.previous_buttons is None:
            self.previous_buttons = buttons
            return
        now = time.monotonic()
        debounce = float(self.config["timing"]["button_debounce_sec"])
        values = {
            "right_a": mapping.right_a,
            "right_b": mapping.right_b,
            "right_c": mapping.right_c,
            "right_d": mapping.right_d,
        }
        rises: list[str] = []
        for name, index in values.items():
            if self._pressed(buttons[index]) and not self._pressed(
                self.previous_buttons[index]
            ):
                if now - self.last_button_at[name] >= debounce:
                    self.last_button_at[name] = now
                    rises.append(name)
        self.previous_buttons = buttons
        if "right_c" in rises:
            self._emergency_stop("right_c")
            return
        observed_state = self.state
        for name in ("right_a", "right_b", "right_d"):
            if name in rises:
                self.actions.put((name, observed_state))

    def _write_hand_mode(self, mode: str, *, reason: str) -> None:
        if mode not in SUPPORTED_HAND_MODES:
            raise RuntimeError(f"unsupported hand control mode: {mode!r}")
        path = Path(str(self.config["hand_mode_file"]))
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(mode + "\n", encoding="utf-8")
        temporary.replace(path)
        at_ns = time.time_ns()
        if self.attempt_record is not None and self.attempt_record:
            self.attempt_record.setdefault("mode_switch_events", {}).append(
                {"at_ns": at_ns, "mode": mode, "reason": reason}
            )
            self._write_attempt()
        self.event_log.write("hand_mode_written", mode=mode, reason=reason, at_ns=at_ns)
        print(
            f"[HAND MODE] teleop control-mode file set to {mode!r} ({reason}). "
            "Confirm the teleop terminal prints the matching switch line.",
            flush=True,
        )

    def _sync_hand_mode_for_state(self, *, reason: str) -> None:
        mode = hand_mode_for_state(self.state)
        if mode is not None:
            self._write_hand_mode(mode, reason=reason)

    def _recording_too_short(self, button: str) -> bool:
        state = self.state
        guard_active = (button == "right_a" and state == S456State.RECORDING_S6) or (
            button == "right_b" and state in {S456State.RECORDING_S4, S456State.RECORDING_S5}
        )
        if not guard_active:
            return False
        elapsed = time.monotonic() - self.collector_started_at
        minimum = float(self.config["timing"]["minimum_skill_recording_sec"])
        if elapsed >= minimum:
            return False
        self.event_log.write(
            "button_ignored_recording_too_short",
            button=button,
            elapsed_sec=elapsed,
            minimum_sec=minimum,
        )
        print(
            f"Button ignored: the take has recorded {elapsed:.1f}s; wait until "
            f"{minimum:.1f}s before marking or stopping.",
            flush=True,
        )
        return True

    def _worker_loop(self) -> None:
        while not self.shutdown_event.is_set():
            try:
                queued_action = self.actions.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                if queued_action == "emergency":
                    button = "emergency"
                    observed_state = S456State.ABORTED
                else:
                    button, observed_state = queued_action
                if button == "emergency":
                    self._acquire_base_control()
                    self._stop_collector(emergency=True)
                    self._finish_attempt("aborted")
                    continue
                if self.state != observed_state:
                    self.event_log.write(
                        "stale_button_ignored",
                        button=button,
                        state_when_pressed=observed_state.value,
                        state_when_processed=self.state.value,
                    )
                    continue
                if self._recording_too_short(button):
                    continue
                with self.machine_lock:
                    before = self.machine.state
                    action = self.machine.press(button)
                    after = self.machine.state
                self.event_log.write(
                    "button",
                    button=button,
                    state_before=before.value,
                    state_after=after.value,
                    action=action.value if action else None,
                )
                if action is None:
                    continue
                if action == S456Action.START_RECORDING:
                    self._begin_attempt()
                    self._acquire_base_control()
                    self._write_hand_mode(DEFAULT_HAND_MODE, reason="take_start")
                    self._start_collector()
                    with self.machine_lock:
                        self.machine.collector_started(START_SKILL)
                    self._update_attempt("recording_s456")
                    self.event_log.write(
                        "s456_ready",
                        recording_start_ns=self.collector_recording_start_ns,
                        episode=str(self.collector_episode_path),
                    )
                    print(
                        "\n"
                        "========================================================================\n"
                        "[S456 READY] Collector is ready and the base is latched at zero.\n"
                        "Perform S4 (right hand picks up the spray). At the stable S4 end\n"
                        "press right B, teach S5 (one index press), press right B again,\n"
                        "then teach S6 (place the box) and press right A to stop.\n"
                        "========================================================================",
                        flush=True,
                    )
                elif action == S456Action.MARK_S5_BOUNDARY:
                    marker_ns = time.time_ns()
                    markers = self.attempt_record.setdefault("phase_markers", {})
                    markers["s5_start_ns"] = marker_ns
                    self.attempt_record["source_episode"] = str(self.collector_episode_path)
                    self._write_attempt()
                    self._write_hand_mode("s5_index_only", reason="s5_boundary")
                    self.event_log.write(
                        "s5_boundary_marked",
                        marker_ns=marker_ns,
                        episode=str(self.collector_episode_path),
                    )
                    print(
                        "[S5 BOUNDARY] marked; teleop switched to s5_index_only. "
                        "Teach exactly one index press and release, keep the right "
                        "hand stable, then press right B again.",
                        flush=True,
                    )
                elif action == S456Action.MARK_S6_BOUNDARY:
                    marker_ns = time.time_ns()
                    markers = self.attempt_record.setdefault("phase_markers", {})
                    markers["s6_start_ns"] = marker_ns
                    self._write_attempt()
                    self._write_hand_mode(DEFAULT_HAND_MODE, reason="s6_boundary")
                    self.event_log.write(
                        "s6_boundary_marked",
                        marker_ns=marker_ns,
                        episode=str(self.collector_episode_path),
                    )
                    print(
                        "[S6 BOUNDARY] marked; teleop restored full_glove. Teach S6 "
                        "(left hand places the box, right hand keeps holding the "
                        "spray), then press right A to stop.",
                        flush=True,
                    )
                elif action == S456Action.STOP_RECORDING:
                    stop_ns = time.time_ns()
                    self.attempt_record.setdefault("phase_markers", {})[
                        "s456_complete_ns"
                    ] = stop_ns
                    source_episode = self.collector_episode_path
                    self._write_attempt()
                    self._stop_collector()
                    self._finalize_continuous_segments(source_episode)
                    with self.machine_lock:
                        self.machine.collector_stopped(START_SKILL)
                    self._finish_attempt("unreviewed")
                    self._release_base_control()
                    with self.machine_lock:
                        self.machine.reset_completed_attempt()
                    self.attempt_id = None
                    self.attempt_path = None
                    self.attempt_record = {}
                    self.event_log.write("s456_reset", state=self.state.value)
                    print(
                        "[S456 COMPLETE] take finalized. Reset the scene, then press "
                        "right A for the next take.",
                        flush=True,
                    )
                elif action == S456Action.FORCE_MODE_SYNC:
                    self._sync_hand_mode_for_state(reason="right_d_force_sync")
            except Exception as exc:  # noqa: BLE001
                detail = f"{type(exc).__name__}: {exc}"
                self.event_log.write("safety_stop", error=detail)
                print(f"SAFETY STOP: {detail}", flush=True)
                self.emergency_event.set()
                with self.machine_lock:
                    self.machine.abort()
                self._publish_zero()
                self._stop_collector(emergency=True)
                self._finish_attempt("aborted", error=detail)

    def _begin_attempt(self) -> None:
        stamp = dt.datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
        self.attempt_id = f"s456_{stamp}_{time.time_ns() % 1_000_000:06d}"
        directory = Path(str(self.config["collection"]["attempt_directory"])).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        self.attempt_path = directory / f"{self.attempt_id}.json"
        self.attempt_record = {
            "schema_version": 1,
            "workflow": WORKFLOW,
            "parent_attempt_id": self.attempt_id,
            "created_at": dt.datetime.now().astimezone().isoformat(),
            "button_sequence": ["right_a", "right_b", "right_b", "right_a"],
            "button_indices": {
                "right_a": 11,
                "right_b": 12,
                "right_c_emergency": 13,
                "right_d_mode_sync": 14,
            },
            "excluded_base_motion": ["B0", "B1", "B2", "B3"],
            "hand_mode_file": str(self.config["hand_mode_file"]),
            "mode_switch_events": [],
            "episodes": {},
            "phase_markers": {},
            "continuous_capture": {
                "single_collector": True,
                "camera_restart_between_segments": False,
                "segments_file": SEGMENTS_FILE,
            },
            "status": "running",
        }
        self._write_attempt()

    def _collector_command(self, skill: str, episode_name: str) -> list[str]:
        collection = self.config["collection"]
        command = [
            str(collection["python"]),
            str(collection["script"]),
            "--skill-id",
            skill,
            "--skills-config",
            str(collection["skills_config"]),
            "--episode-name",
            episode_name,
            "--attempt-outcome",
            "unreviewed",
            "--perturbation",
            str(collection["perturbation"]),
            "--hand-control-mode",
            str(collection["hand_control_mode"]),
            "--operator",
            str(collection["operator"]),
        ]
        if collection.get("no_offline_align", True):
            command.append("--no-offline-align")
        return command

    def _start_collector(self) -> None:
        with self.collector_lock:
            self._start_collector_locked()

    def _start_collector_locked(self) -> None:
        if self.attempt_id is None:
            raise RuntimeError("S456 attempt has no id")
        if self.collector_process is not None:
            raise RuntimeError("a collector process is already running")
        suffix = f"{self.attempt_id}_continuous_s4_s5_s6"
        log_dir = Path(str(self.config["collection"]["log_directory"])).expanduser()
        log_dir.mkdir(parents=True, exist_ok=True)
        self.collector_log_path = log_dir / f"collect_{suffix}.log"
        self.collector_log_handle = self.collector_log_path.open(
            "a", encoding="utf-8", buffering=1
        )
        command = self._collector_command(START_SKILL, suffix)
        self.collector_log_handle.write("command: " + " ".join(command) + "\n")
        try:
            self.collector_process = subprocess.Popen(
                command,
                cwd=str(self.config["collection"]["working_directory"]),
                stdout=self.collector_log_handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except Exception:
            self.collector_log_handle.close()
            self.collector_log_handle = None
            self.collector_log_path = None
            raise
        self.current_skill = START_SKILL
        timeout = float(self.config["timing"]["collector_start_timeout_sec"])
        deadline = time.monotonic() + timeout
        episode_path = None
        episode_manifest: dict[str, Any] | None = None
        while time.monotonic() < deadline:
            if self.emergency_event.is_set():
                raise SafetyStop("emergency stop while collector was starting")
            code = self.collector_process.poll()
            if code is not None:
                text = self.collector_log_path.read_text(encoding="utf-8", errors="replace")
                tail = text[-2000:]
                raise RuntimeError(
                    f"{START_SKILL} collector exited during startup with code {code}; "
                    f"log tail:\n{tail}"
                )
            episode_path = self._find_episode(suffix)
            if episode_path is not None:
                manifest_path = episode_path / "manifest.json"
                if manifest_path.is_file():
                    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
                    if manifest.get("recording_start_ns"):
                        episode_manifest = manifest
                        break
            self._publish_zero()
            time.sleep(0.1)
        else:
            raise RuntimeError(
                f"{START_SKILL} collector did not become ready within {timeout}s"
            )
        self.collector_episode_path = episode_path
        self.collector_started_at = time.monotonic()
        self.collector_recording_start_ns = int(episode_manifest["recording_start_ns"])
        self.attempt_record["source_episode"] = str(episode_path)
        self.attempt_record["continuous_capture"]["collector_pid"] = self.collector_process.pid
        self.attempt_record["episodes"][START_SKILL] = {
            "path": str(episode_path),
            "source_episode": str(episode_path),
            "prompt_version": (
                str(episode_manifest.get("prompt_version"))
                if episode_manifest is not None
                else None
            ),
            "status": "recording",
            "segment_start_ns": self.collector_recording_start_ns,
            "start_inclusive": True,
        }
        self._write_attempt()
        self.event_log.write(
            "collector_started", skill=START_SKILL, episode=str(episode_path)
        )

    def _finalize_continuous_segments(self, source_episode: Path | None) -> None:
        if source_episode is None:
            raise RuntimeError("continuous source episode is missing")
        manifest_path = source_episode / "manifest.json"
        frames_path = source_episode / "frames.jsonl"
        if not manifest_path.is_file() or not frames_path.is_file():
            raise RuntimeError(f"continuous source episode is incomplete: {source_episode}")
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        markers = self.attempt_record.get("phase_markers")
        if not isinstance(markers, dict) or "s5_start_ns" not in markers or "s6_start_ns" not in markers:
            raise RuntimeError("S456 attempt is missing the S5/S6 boundary markers")
        frame_rows: list[dict[str, Any]] = []
        with frames_path.open("r", encoding="utf-8") as stream:
            for line_number, line in enumerate(stream, start=1):
                if not line.strip():
                    continue
                try:
                    row = json.loads(line)
                    int(row["frame_index"])
                    int(row["timestamp_ns"])
                except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                    raise RuntimeError(
                        f"invalid continuous frame at {frames_path}:{line_number}: {exc}"
                    ) from exc
                frame_rows.append(row)
        document = build_s456_segments_document(
            attempt_id=str(self.attempt_id),
            source_episode=str(source_episode),
            manifest=manifest,
            frame_rows=frame_rows,
            phase_markers=markers,
            s5_start_ns=int(markers["s5_start_ns"]),
            s6_start_ns=int(markers["s6_start_ns"]),
            skills=self.skills,
            minimum_frames=int(
                round(
                    float(self.config["timing"]["minimum_skill_recording_sec"])
                    * float(self.config["timing"]["hz"])
                )
            ),
            mode_switch_events=self.attempt_record.get("mode_switch_events", []),
        )
        segments_path = source_episode / SEGMENTS_FILE
        self._atomic_write_json(segments_path, document)

        manifest["training_eligible_as_atomic_episode"] = False
        manifest["paired_continuous_source"] = {
            "workflow": WORKFLOW,
            "segments_file": segments_path.name,
            "camera_pipeline_continuous": True,
            "camera_restart_count_between_segments": 0,
        }
        self._atomic_write_json(manifest_path, manifest)

        self.attempt_record["segments_file"] = str(segments_path)
        for segment in document["segments"]:
            skill_id = str(segment["skill_id"])
            self.attempt_record["episodes"][skill_id] = {
                "path": str(source_episode),
                "source_episode": str(source_episode),
                "status": "segment_complete",
                "segment_start_ns": segment["start_ns"],
                "segment_end_ns": segment["end_ns"],
                "source_start_frame": segment["source_start_frame"],
                "source_end_frame": segment["source_end_frame"],
                "frame_count": segment["frame_count"],
                "hand_control_mode": segment["hand_control_mode"],
                "segments_file": str(segments_path),
            }
        self._write_attempt()
        self.event_log.write(
            "continuous_segments_finalized",
            source_episode=str(source_episode),
            segments_file=str(segments_path),
            frame_counts={
                segment["skill_id"]: segment["frame_count"]
                for segment in document["segments"]
            },
        )
        counts = ", ".join(
            f"{segment['skill_id']}={segment['frame_count']}"
            for segment in document["segments"]
        )
        print(f"[SEGMENTS] finalized {segments_path} ({counts}).", flush=True)

    def _release_base_control(self) -> None:
        if not self.base_control_active.is_set():
            return
        count = int(self.config["safety"]["emergency_zero_publish_count"])
        for _ in range(count):
            self._publish_zero()
            time.sleep(1.0 / 15.0)
        self.base_control_active.clear()
        self.takeover.restore()
        self.event_log.write("base_control_released")
        print("[BASE RELEASED] F710 restored; reposition the robot for the next take.", flush=True)


def execute(config_path: Path, skills: dict[str, dict[str, Any]]) -> int:
    config, _skills = load_all(config_path)
    os.environ["ROS_DOMAIN_ID"] = str(config["ros"]["domain_id"])
    os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
    lease = FileLease(Path(str(config["safety"]["base_command_lock"])).expanduser())
    takeover = ManagedServiceTakeover(config)
    lease.acquire()
    runtime = None
    try:
        runtime = S456CollectionRuntime(config, skills, takeover)
        print(
            "S456 collection ready: right A starts the continuous take (base latched "
            "at zero, F710 disabled), right B marks S4/S5 then S5/S6 and switches "
            "the teleop hand mode, right A stops and finalizes; right C is "
            "emergency; right D re-syncs the teleop hand mode file. Position the "
            "robot manually before pressing right A.",
            flush=True,
        )
        runtime.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if runtime is not None:
                runtime.close()
        finally:
            try:
                takeover.restore()
            finally:
                lease.close()
    return 0


def main() -> int:
    args = parse_args()
    config_path = args.config.expanduser().resolve()
    config, skills = load_all(config_path)
    if not args.execute:
        print(json.dumps(dry_run(config_path), indent=2, ensure_ascii=False))
        return 0
    acknowledgement = str(config["safety"]["motion_acknowledgement"])
    if args.acknowledge_motion != acknowledgement:
        raise SystemExit(
            f"real execution blocked: pass --acknowledge-motion {acknowledgement}"
        )
    try:
        return execute(config_path, skills)
    except SafetyStop as exc:
        raise SystemExit(f"real execution blocked: {exc}") from None


if __name__ == "__main__":
    raise SystemExit(main())
