from __future__ import annotations

import argparse
import contextlib
import datetime as dt
import importlib.util
import json
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Any


COLLECTOR_ROOT = Path(__file__).resolve().parents[1]
if str(COLLECTOR_ROOT) not in sys.path:
    sys.path.insert(0, str(COLLECTOR_ROOT))

from supervisor.core import SafetyStop, load_base_prior_stream, load_yaml
from supervisor.paired_collection_controller import (
    EventLog,
    FileLease,
    ManagedServiceTakeover,
    PairedCollectionRuntime,
)
from supervisor.s3_collection_core import (
    S3CollectionAction,
    S3CollectionState,
    S3CollectionStateMachine,
    validate_config,
)


STAGES = ("B2",)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Button-driven S3 collection plus an independent one-shot B2 replay."
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=COLLECTOR_ROOT / "config" / "s3_collection.yaml",
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--acknowledge-motion")
    return parser.parse_args()


def resolve_path(config_path: Path, value: Any) -> Path:
    path = Path(str(value)).expanduser()
    return path if path.is_absolute() else (config_path.parent / path).resolve()


def load_all(config_path: Path) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    config = load_yaml(config_path)
    validate_config(config)
    priors_path = resolve_path(config_path, config["base_priors"])
    collector_path = resolve_path(config_path, config["collector_config"])
    priors = load_yaml(priors_path)
    collector = load_yaml(collector_path)
    if config["robot_id"] != priors["robot_id"] or config["robot_id"] != collector["robot_id"]:
        raise ValueError("robot_id differs across S3, prior, and collector configs")
    if int(config["timing"]["hz"]) != 15 or int(priors["sample_rate_hz"]) != 15:
        raise ValueError("S3 collection and priors must both run at 15 Hz")
    missing = [stage for stage in STAGES if stage not in priors.get("priors", {})]
    if missing:
        raise ValueError(f"base prior config is missing {missing}")
    config["base_priors"] = str(priors_path)
    config["collector_config"] = str(collector_path)
    hand = dict(config["hand_controller"])
    hand["module"] = str(resolve_path(config_path, hand["module"]))
    hand["config"] = str(resolve_path(config_path, hand["config"]))
    config["hand_controller"] = hand
    collection = dict(config["collection"])
    collection["skills_config"] = str(
        resolve_path(config_path, collection.get("skills_config", "skills.yaml"))
    )
    config["collection"] = collection
    return config, priors, collector


def load_s3_hand_controller(config: dict[str, Any]) -> tuple[Any, dict[str, Any]]:
    """Load the shared hand config and override S3-only bindings in memory."""
    embedded = config["hand_controller"]
    module_path = Path(str(embedded["module"])).expanduser()
    hand_config_path = Path(str(embedded["config"])).expanduser()
    if not module_path.is_file() or not hand_config_path.is_file():
        raise RuntimeError(
            f"embedded hand controller files are missing: module={module_path}, "
            f"config={hand_config_path}"
        )
    spec = importlib.util.spec_from_file_location(
        "svt_s3_embedded_o6_button_controller", module_path
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load embedded hand controller from {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    hand_config = module.load_config(hand_config_path)

    buttons = hand_config.setdefault("o6_button_controller", {})
    left = dict(buttons.get("left", {}))
    left["button_index"] = int(embedded["left_c_button_index"])
    left["action_key"] = str(embedded["left_c_action_key"])
    buttons["left"] = left
    right = dict(buttons.get("right", {}))
    right["button_enabled"] = False
    buttons["right"] = right
    collection = dict(buttons.get("collection", {}))
    collection["enabled"] = False
    buttons["collection"] = collection

    action_key = str(embedded["left_c_action_key"])
    action = hand_config.get("o6_hotkeys", {}).get("actions", {}).get(action_key, {})
    if action.get("enabled") is not True:
        raise RuntimeError(f"embedded O6 action {action_key} is missing or disabled")
    if str(buttons.get("topic")) != str(config["joy_topic"]):
        raise RuntimeError("S3 and embedded hand controllers must use the same Joy topic")
    return module, hand_config


def dry_run(config_path: Path) -> dict[str, Any]:
    config, priors, _collector = load_all(config_path)
    machine = S3CollectionStateMachine()
    trace = [machine.state.value]
    machine.press("right_b")
    trace.append(machine.state.value)
    machine.base_complete("B2")
    trace.append(machine.state.value)
    machine.press("right_a")
    trace.append(machine.state.value)
    machine.collector_started("S3")
    trace.append(machine.state.value)
    machine.press("right_a")
    trace.append(machine.state.value)
    machine.collector_stopped("S3")
    trace.append(machine.state.value)
    repeated_b2_action = machine.press("right_b")

    prior_status: dict[str, dict[str, Any]] = {}
    for stage in STAGES:
        entry = priors["priors"][stage]
        candidate = entry.get("candidate") or {}
        prior_status[stage] = {
            "version": entry.get("version"),
            "calibrated": entry.get("calibrated") is True,
            "active_stream": entry.get("stream"),
            "candidate_stream": candidate.get("stream"),
            "candidate_validation": candidate.get("twist_stream_validation"),
            "candidate_replay_approved": candidate.get("replay_approved") is True,
        }
    return {
        "mode": "dry_run",
        "robot_id": config["robot_id"],
        "buttons": {
            "right_a": "first press starts S3 and prints [S3 READY]; second press stops S3",
            "right_b": "run independent B2 forward once in any normal state; later presses report [ERROR]",
            "right_c": "latched emergency stop",
            "right_d": "reserved",
            "left_c": "toggle O6 action 3",
        },
        "excluded_motion": {
            "B3R": True,
            "B3_lateral": True,
            "B3_forward_correction": True,
            "note": "no B3 variant is loaded by this controller",
        },
        "base_publisher_takeover": {
            "startup": "F710 remains active except during the independent B2 replay",
            "acquire": "first right B only",
            "release": "immediately after B2 completes",
        },
        "state_trace": trace,
        "repeated_right_b_action": (
            repeated_b2_action.value if repeated_b2_action is not None else None
        ),
        "priors": prior_status,
        "motion_published": False,
        "ready_for_real_execution": all(
            value["calibrated"] and value["active_stream"]
            for value in prior_status.values()
        ),
    }


class S3CollectionRuntime(PairedCollectionRuntime):
    def __init__(
        self,
        config: dict[str, Any],
        priors: dict[str, Any],
        collector: dict[str, Any],
        prior_streams: dict[str, list[Any]],
        takeover: ManagedServiceTakeover,
    ) -> None:
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState, Joy

        self.rclpy = rclpy
        self.Twist = Twist
        self.config = config
        self.priors = priors
        self.collector = collector
        self.prior_streams = prior_streams
        self.takeover = takeover
        skills_document = load_yaml(Path(str(config["collection"]["skills_config"])))
        skills = skills_document.get("skills") if isinstance(skills_document, dict) else None
        if not isinstance(skills, dict) or not isinstance(skills.get("S3"), dict):
            raise RuntimeError("S3 collection requires an S3 skill definition")
        self.skills: dict[str, dict[str, Any]] = skills
        self.machine = S3CollectionStateMachine()
        self.machine_lock = threading.Lock()
        self.shutdown_event = threading.Event()
        self.emergency_event = threading.Event()
        self.base_control_active = threading.Event()
        self.base_replay_active = threading.Event()
        self.actions: queue.Queue[tuple[str, S3CollectionState] | str] = queue.Queue()
        self.feedback_lock = threading.Lock()
        self.base_feedback: tuple[float, float, float] | None = None
        self.base_received_at = 0.0
        self.arm_position = None
        self.arm_effort = None
        self.arm_received_at = 0.0
        self.previous_buttons: list[int] | None = None
        self.last_button_at = {
            "right_a": 0.0,
            "right_b": 0.0,
            "right_c": 0.0,
            "right_d": 0.0,
        }
        self.collector_lock = threading.Lock()
        self.collector_process: subprocess.Popen[Any] | None = None
        self.collector_log_handle: Any | None = None
        self.collector_log_path: Path | None = None
        self.collector_episode_path: Path | None = None
        self.collector_started_at = 0.0
        self.collector_recording_start_ns: int | None = None
        self.current_skill: str | None = None
        self.hand_node: Any | None = None
        self.hand_lease: Any | None = None
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
        self.node = rclpy.create_node("svt_s3_collection_controller")
        time.sleep(0.25)
        self._remember_managed_base_publishers(before_own_publisher=True)
        ros = config["ros"]
        self.base_publisher = self.node.create_publisher(Twist, ros["base_command_topic"], 10)
        self.node.create_subscription(Joy, config["joy_topic"], self._joy_callback, 10)
        self.node.create_subscription(Twist, ros["base_feedback_topic"], self._base_callback, 10)
        self.node.create_subscription(
            JointState,
            ros["joint_state_topic"],
            self._joint_callback,
            qos_profile_sensor_data,
        )
        self.zero_timer = self.node.create_timer(1.0 / 15.0, self._zero_timer)
        try:
            hand_module, hand_config = load_s3_hand_controller(config)
            lock_file = hand_config["o6_button_controller"].get(
                "lock_file", "/tmp/svt_dual_hand_command.lock"
            )
            self.hand_lease = hand_module.HandCommandLease(
                lock_file,
                mode="s3_collection",
                owner="s3_collection_controller",
            )
            self.hand_lease.acquire()
            self.hand_node = hand_module.HandButtonController(hand_config, dry_run=False)
        except Exception:
            if self.hand_lease is not None:
                self.hand_lease.close()
                self.hand_lease = None
            self.node.destroy_node()
            if self.rclpy.ok():
                self.rclpy.shutdown()
            self.event_log.close()
            raise
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self.executor.add_node(self.hand_node)
        self.worker = threading.Thread(
            target=self._worker_loop,
            name="s3-collection",
            daemon=True,
        )
        self.worker.start()
        self.event_log.write(
            "runtime_ready",
            state=self.machine.state.value,
            embedded_hand_controller=True,
            left_c_action="3",
            base_motion="independent_one_shot_b2",
        )

    @property
    def state(self) -> S3CollectionState:
        with self.machine_lock:
            return self.machine.state

    def _zero_timer(self) -> None:
        if self.base_control_active.is_set() and not self.base_replay_active.is_set():
            self._publish_zero()

    def _joy_callback(self, message: Any) -> None:
        buttons = [int(value) for value in message.buttons]
        mapping = validate_config(self.config)
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

    def _acquire_base_control(self) -> None:
        if self.base_control_active.is_set():
            return
        self._remember_managed_base_publishers()
        self.takeover.acquire()
        self._wait_for_exclusive_base_publisher()
        self.base_control_active.set()
        for _ in range(3):
            self._publish_zero()
            time.sleep(1.0 / 15.0)
        self.event_log.write("base_control_acquired")
        print("S3 base control acquired; F710 is temporarily disabled.", flush=True)

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
        print("[B2 COMPLETE] B2 ended; F710 has been restored.", flush=True)

    def _recording_too_short(self, button: str) -> bool:
        if self.state != S3CollectionState.RECORDING_S3 or button != "right_a":
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
            f"Button ignored: S3 has recorded {elapsed:.1f}s; wait until {minimum:.1f}s.",
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
                    observed_state = S3CollectionState.ABORTED
                else:
                    button, observed_state = queued_action
                if button == "emergency":
                    self._acquire_base_control()
                    self._stop_collector(emergency=True)
                    self._finish_attempt("aborted")
                    continue
                if button == "right_b" and self.machine.b2_executed_once:
                    self.event_log.write(
                        "b2_repeat_rejected",
                        state_when_pressed=observed_state.value,
                        state_when_processed=self.state.value,
                    )
                    print(
                        "[ERROR] B2 has already been executed once in this script "
                        "process. Restart the controller before another B2 replay.",
                        flush=True,
                    )
                    continue
                if self.state != observed_state and button != "right_b":
                    self.event_log.write(
                        "stale_button_ignored",
                        button=button,
                        state_when_pressed=observed_state.value,
                        state_when_processed=self.state.value,
                    )
                    continue
                if button == "right_d":
                    self.event_log.write("reserved_button_ignored", button=button)
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
                if action == S3CollectionAction.START_S3:
                    self._begin_attempt()
                    self._start_s3()
                elif action == S3CollectionAction.RUN_B2:
                    self.event_log.write(
                        "independent_b2_triggered",
                        s3_state=self.state.value,
                    )
                    self.base_replay_active.set()
                    try:
                        self._acquire_base_control()
                        self._run_prior("B2")
                        with self.machine_lock:
                            self.machine.base_complete("B2")
                    finally:
                        self.base_replay_active.clear()
                    self._release_base_control()
                elif action == S3CollectionAction.STOP_S3:
                    self._stop_collector()
                    with self.machine_lock:
                        self.machine.collector_stopped("S3")
                    self._finish_attempt("unreviewed")
                    with self.machine_lock:
                        self.machine.reset_completed_s3()
                        b2_executed_once = self.machine.b2_executed_once
                    b2_status = (
                        "B2 remains locked because it has already run once."
                        if b2_executed_once
                        else "B2 is still available once, independently of S3 recording."
                    )
                    print(
                        "[S3 COMPLETE] S3 recording stopped. Press right A to start "
                        f"another S3 episode. {b2_status}",
                        flush=True,
                    )
                elif action == S3CollectionAction.B2_ALREADY_EXECUTED:
                    print(
                        "[ERROR] B2 has already been executed once in this script process.",
                        flush=True,
                    )
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
        self.attempt_id = f"s3_{stamp}_{time.time_ns() % 1_000_000:06d}"
        directory = Path(str(self.config["collection"]["attempt_directory"])).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        self.attempt_path = directory / f"{self.attempt_id}.json"
        self.attempt_record = {
            "schema_version": 1,
            "workflow": "s3_capture_with_independent_one_shot_b2_v3",
            "attempt_id": self.attempt_id,
            "created_at": dt.datetime.now().astimezone().isoformat(),
            "button_sequence": ["right_a", "right_a"],
            "button_indices": {
                "right_a": 11,
                "right_b": 12,
                "right_c_emergency": 13,
                "left_c_action3": 7,
            },
            "independent_controls": {
                "right_b": {
                    "stage": "B2",
                    "relationship_to_s3": "independent",
                    "maximum_executions_per_process": 1,
                }
            },
            "excluded_base_motion": ["B3R", "B3_lateral", "B3_forward_correction"],
            "episodes": {},
            "phase_markers": {},
            "status": "running",
        }
        self._write_attempt()

    def _start_s3(self) -> None:
        print(
            "\n"
            "========================================================================\n"
            "[WAIT] Starting and checking the S3 collector.\n"
            "Wait for [S3 READY] before beginning S3.\n"
            "========================================================================",
            flush=True,
        )
        self._start_collector("S3")
        with self.machine_lock:
            self.machine.collector_started("S3")
        ready_ns = time.time_ns()
        self.attempt_record["phase_markers"]["s3_ready_ns"] = ready_ns
        self._update_attempt("recording_s3")
        self.event_log.write(
            "s3_ready",
            marker_ns=ready_ns,
            recording_start_ns=self.collector_recording_start_ns,
            episode=str(self.collector_episode_path),
        )
        print(
            "\n"
            "========================================================================\n"
            "[S3 READY] Collector is ready. Start raising the arm and perform S3.\n"
            "Press right A again after S3 is complete to stop recording.\n"
            "========================================================================",
            flush=True,
        )

    def _optional_feedback_stage(self, stage: str) -> bool:
        return stage in {
            str(value)
            for value in self.config["timing"].get("base_feedback_optional_stages", [])
        }

    def _feedback_snapshot(
        self, stage: str
    ) -> tuple[tuple[float, float, float], Any | None, Any | None]:
        with self.feedback_lock:
            base = self.base_feedback
            base_age = time.monotonic() - self.base_received_at
            arm = None if self.arm_position is None else self.arm_position.copy()
            effort = None if self.arm_effort is None else self.arm_effort.copy()
        timeout = float(self.config["timing"]["base_feedback_timeout_sec"])
        if base is None or base_age > timeout:
            if not self._optional_feedback_stage(stage):
                raise SafetyStop(f"base feedback is unavailable or stale ({base_age:.3f}s)")
            if stage not in self.optional_feedback_logged:
                self.optional_feedback_logged.add(stage)
                self.event_log.write(
                    "base_feedback_optional_fallback",
                    stage=stage,
                    age_sec=base_age,
                )
                print(
                    f"{stage}: base feedback unavailable; using checked open-loop replay "
                    "and a fixed zero hold.",
                    flush=True,
                )
            return (0.0, 0.0, 0.0), arm, effort
        return base, arm, effort

    def _wait_base_zero(self, stage: str) -> None:
        with self.feedback_lock:
            base_missing = self.base_feedback is None or (
                time.monotonic() - self.base_received_at
                > float(self.config["timing"]["base_feedback_timeout_sec"])
            )
        if self._optional_feedback_stage(stage) and base_missing:
            deadline = time.monotonic() + float(
                self.config["timing"]["base_open_loop_zero_hold_sec"]
            )
            while time.monotonic() < deadline:
                if self.emergency_event.is_set():
                    raise SafetyStop("right C emergency stop is latched")
                self._assert_no_base_publisher()
                self._publish_zero()
                time.sleep(1.0 / 15.0)
            return
        super()._wait_base_zero(stage)

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

    def _start_collector_locked(self, skill: str) -> None:
        if self.attempt_id is None:
            raise RuntimeError("S3 attempt has no id")
        if self.collector_process is not None:
            raise RuntimeError("a collector process is already running")
        if skill != "S3":
            raise RuntimeError("S3 collection can only start the S3 collector")
        suffix = f"{self.attempt_id}_s3"
        log_dir = Path(str(self.config["collection"]["log_directory"])).expanduser()
        log_dir.mkdir(parents=True, exist_ok=True)
        self.collector_log_path = log_dir / f"collect_{suffix}.log"
        self.collector_log_handle = self.collector_log_path.open(
            "a", encoding="utf-8", buffering=1
        )
        command = self._collector_command(skill, suffix)
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
        self.current_skill = skill
        timeout = float(self.config["timing"]["collector_start_timeout_sec"])
        deadline = time.monotonic() + timeout
        episode_path = None
        episode_manifest: dict[str, Any] | None = None
        while time.monotonic() < deadline:
            if self.emergency_event.is_set():
                raise SafetyStop("emergency stop while collector was starting")
            code = self.collector_process.poll()
            if code is not None:
                raise RuntimeError(f"{skill} collector exited during startup with code {code}")
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
            raise RuntimeError(f"{skill} collector did not become ready within {timeout}s")
        self.collector_episode_path = episode_path
        self.collector_started_at = time.monotonic()
        self.collector_recording_start_ns = int(episode_manifest["recording_start_ns"])
        self.attempt_record["episodes"][skill] = {
            "path": str(episode_path),
            "prompt_version": str(episode_manifest.get("prompt_version")),
            "status": "recording",
            "recording_start_ns": self.collector_recording_start_ns,
        }
        self._write_attempt()
        self.event_log.write("collector_started", skill=skill, episode=str(episode_path))


def execute(config_path: Path) -> int:
    config, priors, collector = load_all(config_path)
    os.environ["ROS_DOMAIN_ID"] = str(config["ros"]["domain_id"])
    os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
    priors_path = Path(str(config["base_priors"]))
    prior_streams = {
        stage: load_base_prior_stream(
            stage,
            priors["priors"][stage],
            config_dir=priors_path.parent,
            limits=priors["safety_limits"],
            require_calibrated=True,
            sample_rate_hz=float(priors["sample_rate_hz"]),
        )
        for stage in STAGES
    }
    lease = FileLease(Path(str(config["safety"]["base_command_lock"])).expanduser())
    takeover = ManagedServiceTakeover(config)
    lease.acquire()
    runtime = None
    try:
        runtime = S3CollectionRuntime(config, priors, collector, prior_streams, takeover)
        print(
            "S3 collection ready: right A starts S3 and prints [S3 READY], right A "
            "again stops S3; independently, right B runs B2 forward once in any "
            "normal state and then remains locked; left C toggles O6 action 3, "
            "right C is emergency.",
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
    config, _priors, _collector = load_all(config_path)
    if not args.execute:
        print(json.dumps(dry_run(config_path), indent=2, ensure_ascii=False))
        return 0
    acknowledgement = str(config["safety"]["motion_acknowledgement"])
    if args.acknowledge_motion != acknowledgement:
        raise SystemExit(
            f"real execution blocked: pass --acknowledge-motion {acknowledgement}"
        )
    try:
        return execute(config_path)
    except SafetyStop as exc:
        raise SystemExit(f"real execution blocked: {exc}") from None


if __name__ == "__main__":
    raise SystemExit(main())
