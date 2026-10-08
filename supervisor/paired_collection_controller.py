from __future__ import annotations

import argparse
import contextlib
import datetime as dt
import fcntl
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

import numpy as np
import yaml


COLLECTOR_ROOT = Path(__file__).resolve().parents[1]
if str(COLLECTOR_ROOT) not in sys.path:
    sys.path.insert(0, str(COLLECTOR_ROOT))

from supervisor.core import (
    SafetyStop,
    load_base_prior_stream,
    load_yaml,
)
from supervisor.paired_collection_core import (
    PairAction,
    PairState,
    PairStateMachine,
    validate_config,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Button-driven continuous B0-S1(with B1)-S2 data collection."
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=COLLECTOR_ROOT / "config" / "paired_collection.yaml",
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
        raise ValueError("robot_id differs across paired, prior, and collector configs")
    if int(config["timing"]["hz"]) != 15 or int(priors["sample_rate_hz"]) != 15:
        raise ValueError("paired collection and priors must both run at 15 Hz")
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


def load_embedded_hand_controller(config: dict[str, Any]) -> tuple[Any, dict[str, Any]]:
    embedded = config["hand_controller"]
    module_path = Path(str(embedded["module"])).expanduser()
    hand_config_path = Path(str(embedded["config"])).expanduser()
    if not module_path.is_file() or not hand_config_path.is_file():
        raise RuntimeError(
            f"embedded hand controller files are missing: module={module_path}, "
            f"config={hand_config_path}"
        )
    spec = importlib.util.spec_from_file_location("svt_embedded_o6_button_controller", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load embedded hand controller from {module_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    hand_config = module.load_config(hand_config_path)
    buttons = hand_config.get("o6_button_controller", {})
    left = buttons.get("left", {})
    right = buttons.get("right", {})
    collection = buttons.get("collection", {})
    expected_index = int(embedded["left_c_button_index"])
    expected_action = str(embedded["left_c_action_key"])
    if buttons.get("enabled") is not True:
        raise RuntimeError("embedded hand button controller is disabled")
    if int(left.get("button_index", -1)) != expected_index:
        raise RuntimeError(f"embedded left C must use buttons[{expected_index}]")
    if str(left.get("action_key", "")) != expected_action:
        raise RuntimeError(f"embedded left C must use O6 action {expected_action}")
    action = hand_config.get("o6_hotkeys", {}).get("actions", {}).get(expected_action, {})
    if action.get("enabled") is not True:
        raise RuntimeError(f"embedded O6 action {expected_action} is missing or disabled")
    if right.get("button_enabled") is not False:
        raise RuntimeError("embedded right C hand toggle must be disabled for paired emergency")
    if collection.get("enabled") is not False:
        raise RuntimeError("embedded generic collection must be disabled in paired mode")
    if str(buttons.get("topic")) != str(config["joy_topic"]):
        raise RuntimeError("paired and embedded hand controllers must use the same Joy topic")
    return module, hand_config


def dry_run(config_path: Path) -> dict[str, Any]:
    config, priors, _collector = load_all(config_path)
    machine = PairStateMachine()
    trace = [machine.state.value]
    machine.press("right_a")
    trace.append(machine.state.value)
    machine.base_complete("B0")
    machine.collector_started("S1")
    trace.append(machine.state.value)
    machine.press("right_a")
    trace.append(machine.state.value)
    machine.press("right_b")
    trace.append(machine.state.value)
    machine.base_complete("B1")
    machine.switch_to_s2()
    trace.append(machine.state.value)
    machine.press("right_b")
    machine.collector_stopped("S2")
    trace.append(machine.state.value)
    prior_status = {}
    for stage in ("B0", "B1"):
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
            "right_a": (
                "run B0 -> wait for the terminal [S1 READY] notice -> start S1; "
                "press again to mark grasp complete"
            ),
            "right_b": (
                "run B1 while the single collector keeps recording -> mark S2 start; "
                "press again to stop the continuous collector"
            ),
            "right_c": "latched emergency stop",
            "right_d": "reserved",
        },
        "embedded_hand_controller": {
            "standalone_process_required": False,
            "left_c": "O6 action 4",
            "right_c_hand_toggle": "disabled; paired emergency has priority",
        },
        "base_publisher_takeover": {
            "startup": "F710 remains active; paired publisher is silent",
            "acquire": "first right A",
            "release": "after second right B completes S2",
            "repeat_without_restart": True,
        },
        "state_trace": trace,
        "continuous_episode": (
            "one camera/ROS/hand collector spans S1, concurrent B1, and S2; "
            "paired_segments.json records the S1/S2 frame ranges"
        ),
        "priors": prior_status,
        "motion_published": False,
        "ready_for_real_execution": all(
            value["calibrated"] and value["active_stream"] for value in prior_status.values()
        ),
    }


class FileLease:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.handle: Any | None = None

    def acquire(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.handle = self.path.open("a+", encoding="utf-8")
        try:
            fcntl.flock(self.handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            self.handle.close()
            self.handle = None
            raise RuntimeError(f"base command lease is already held: {self.path}") from exc
        self.handle.seek(0)
        self.handle.truncate()
        self.handle.write(f"pid={os.getpid()} owner=paired_collection\n")
        self.handle.flush()

    def close(self) -> None:
        if self.handle is not None:
            fcntl.flock(self.handle.fileno(), fcntl.LOCK_UN)
            self.handle.close()
            self.handle = None


class ManagedServiceTakeover:
    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config["base_publisher_takeover"]
        self.managed: list[tuple[dict[str, Any], bool]] = []
        self.lock = threading.RLock()

    def _service_active(self, unit: str) -> bool:
        result = subprocess.run(
            [str(self.config["systemctl"]), "is-active", "--quiet", unit],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=False,
        )
        return result.returncode == 0

    def _run_service_action(self, action: str, unit: str, timeout: float) -> None:
        command = [
            str(self.config["sudo"]),
            "-n",
            str(self.config["systemctl"]),
            action,
            unit,
        ]
        result = subprocess.run(
            command,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            check=False,
        )
        if result.returncode != 0:
            detail = result.stdout.strip() or f"exit code {result.returncode}"
            raise RuntimeError(f"cannot {action} {unit}: {detail}")

    def _wait_service(self, unit: str, *, active: bool, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._service_active(unit) is active:
                return
            time.sleep(0.1)
        state = "active" if active else "inactive"
        raise RuntimeError(f"{unit} did not become {state} within {timeout:g}s")

    def _f710_enabled(self, service: dict[str, Any], target: bool | None) -> bool:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import Bool

        context = Context()
        rclpy.init(args=None, context=context)
        node = rclpy.create_node(
            f"paired_f710_takeover_{os.getpid()}", context=context
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        observed: bool | None = None

        def status_callback(message: Any) -> None:
            nonlocal observed
            observed = bool(message.data)

        node.create_subscription(Bool, service["status_topic"], status_callback, 10)
        publisher = (
            node.create_publisher(Bool, service["enable_topic"], 10)
            if target is not None
            else None
        )
        deadline = time.monotonic() + float(self.config["status_timeout_sec"])
        try:
            while time.monotonic() < deadline:
                if publisher is not None:
                    message = Bool()
                    message.data = bool(target)
                    publisher.publish(message)
                executor.spin_once(timeout_sec=0.1)
                if observed is not None and (target is None or observed is target):
                    return observed
        finally:
            executor.remove_node(node)
            executor.shutdown(timeout_sec=1.0)
            node.destroy_node()
            rclpy.shutdown(context=context)
        expected = "any status" if target is None else str(target).lower()
        raise RuntimeError(
            f"{service['unit']} did not report {service['status_topic']}={expected}"
        )

    def _wait_publisher_absent(self, service: dict[str, Any]) -> None:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor

        context = Context()
        rclpy.init(args=None, context=context)
        node = rclpy.create_node(
            f"paired_publisher_release_{os.getpid()}", context=context
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        deadline = time.monotonic() + float(self.config["release_timeout_sec"])
        try:
            while time.monotonic() < deadline:
                names = {
                    info.node_name
                    for info in node.get_publishers_info_by_topic(
                        str(self.config["command_topic"])
                    )
                }
                if str(service["publisher_node"]) not in names:
                    return
                executor.spin_once(timeout_sec=0.1)
        finally:
            executor.remove_node(node)
            executor.shutdown(timeout_sec=1.0)
            node.destroy_node()
            rclpy.shutdown(context=context)
        raise RuntimeError(
            f"publisher {service['publisher_node']} did not release "
            f"{self.config['command_topic']}"
        )

    def acquire(self) -> None:
        with self.lock:
            self._acquire_locked()

    def _acquire_locked(self) -> None:
        for service in self.config["managed_services"]:
            unit = str(service["unit"])
            if not self._service_active(unit):
                print(f"Managed base publisher already inactive: {unit}")
                continue
            initial_enabled = self._f710_enabled(service, None)
            self.managed.append((service, initial_enabled))
            self._f710_enabled(service, False)
            self._run_service_action(
                "stop", unit, float(self.config["service_stop_timeout_sec"])
            )
            self._wait_service(
                unit,
                active=False,
                timeout=float(self.config["service_stop_timeout_sec"]),
            )
            self._wait_publisher_absent(service)
            print(f"Managed base publisher stopped: {unit}")

    def restore(self, *, force_enable: bool = False) -> None:
        with self.lock:
            self._restore_locked(force_enable=force_enable)

    def _restore_locked(self, *, force_enable: bool = False) -> None:
        errors: list[str] = []
        for service, initial_enabled in reversed(self.managed):
            unit = str(service["unit"])
            enabled = True if force_enable else initial_enabled
            try:
                if not self._service_active(unit):
                    self._run_service_action(
                        "start", unit, float(self.config["service_start_timeout_sec"])
                    )
                    self._wait_service(
                        unit,
                        active=True,
                        timeout=float(self.config["service_start_timeout_sec"]),
                    )
                self._f710_enabled(service, enabled)
                print(
                    f"Managed base publisher restored: {unit}, "
                    f"enabled={str(enabled).lower()}"
                )
            except Exception as exc:  # noqa: BLE001
                errors.append(f"{unit}: {exc}")
        if not errors:
            self.managed.clear()
            return
        raise RuntimeError("failed to restore base publisher service: " + "; ".join(errors))


class EventLog:
    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.handle = path.open("a", encoding="utf-8", buffering=1)
        self.lock = threading.Lock()

    def write(self, event: str, **values: Any) -> None:
        row = {"timestamp_ns": time.time_ns(), "event": event, **values}
        with self.lock:
            self.handle.write(json.dumps(row, ensure_ascii=False, default=str) + "\n")

    def close(self) -> None:
        with self.lock:
            self.handle.close()


class PairedCollectionRuntime:
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
        if not isinstance(skills, dict) or not all(
            isinstance(skills.get(skill), dict) for skill in ("S1", "S2")
        ):
            raise RuntimeError("paired collection requires S1 and S2 skill definitions")
        self.skills: dict[str, dict[str, Any]] = skills
        self.machine = PairStateMachine()
        self.machine_lock = threading.Lock()
        self.shutdown_event = threading.Event()
        self.emergency_event = threading.Event()
        self.base_control_active = threading.Event()
        self.actions: queue.Queue[tuple[str, PairState] | str] = queue.Queue()
        self.feedback_lock = threading.Lock()
        self.base_feedback: tuple[float, float, float] | None = None
        self.base_received_at = 0.0
        self.arm_position: np.ndarray | None = None
        self.arm_effort: np.ndarray | None = None
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
        self.hand_node: Any | None = None
        self.hand_lease: Any | None = None
        self.executor: Any | None = None
        self.managed_base_publisher_gids: set[bytes] = set()
        self.attempt_id: str | None = None
        self.attempt_path: Path | None = None
        self.attempt_record: dict[str, Any] = {}
        self.event_log = EventLog(Path(str(config["event_log"])).expanduser())

        os.environ["ROS_DOMAIN_ID"] = str(config["ros"]["domain_id"])
        os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = rclpy.create_node("svt_paired_collection_controller")
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
            hand_module, hand_config = load_embedded_hand_controller(config)
            lock_file = hand_config["o6_button_controller"].get(
                "lock_file", "/tmp/svt_dual_hand_command.lock"
            )
            self.hand_lease = hand_module.HandCommandLease(
                lock_file,
                mode="paired_collection",
                owner="paired_collection_controller",
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
        self.worker = threading.Thread(target=self._worker_loop, name="paired-collection", daemon=True)
        self.worker.start()
        self.event_log.write(
            "runtime_ready",
            state=self.machine.state.value,
            embedded_hand_controller=True,
            left_c_action="4",
        )

    @property
    def state(self) -> PairState:
        with self.machine_lock:
            return self.machine.state

    @staticmethod
    def _publisher_gid(info: Any) -> bytes:
        return bytes(info.endpoint_gid)

    def _remember_managed_base_publishers(
        self, *, before_own_publisher: bool = False
    ) -> None:
        topic = self.config["ros"]["base_command_topic"]
        own_name = self.node.get_name()
        allowed = {
            str(service["publisher_node"])
            for service in self.config["base_publisher_takeover"]["managed_services"]
        }
        if not before_own_publisher:
            allowed.add(own_name)
        infos = self.node.get_publishers_info_by_topic(topic)
        conflicts = [info.node_name for info in infos if info.node_name not in allowed]
        if conflicts and self.config["safety"].get("require_exclusive_base_publisher", True):
            raise RuntimeError(
                f"{topic} has unmanaged publishers {conflicts}; paired collection cannot take over"
            )
        managed_names = {
            str(service["publisher_node"])
            for service in self.config["base_publisher_takeover"]["managed_services"]
        }
        self.managed_base_publisher_gids.update(
            self._publisher_gid(info) for info in infos if info.node_name in managed_names
        )

    def _assert_no_base_publisher(self) -> None:
        topic = self.config["ros"]["base_command_topic"]
        own_name = self.node.get_name()
        infos = self.node.get_publishers_info_by_topic(topic)
        conflicts = [
            info.node_name
            for info in infos
            if info.node_name != own_name
            and self._publisher_gid(info) not in self.managed_base_publisher_gids
        ]
        if conflicts and self.config["safety"].get("require_exclusive_base_publisher", True):
            raise RuntimeError(
                f"{topic} has unmanaged publishers {conflicts}; paired collection cannot take over"
            )

    def _wait_for_exclusive_base_publisher(self) -> None:
        """Wait for this runtime's ROS graph cache to observe the F710 shutdown."""
        timeout = float(self.config["base_publisher_takeover"]["release_timeout_sec"])
        deadline = time.monotonic() + timeout
        last_error: RuntimeError | None = None
        while time.monotonic() < deadline:
            try:
                self._assert_no_base_publisher()
                return
            except RuntimeError as exc:
                last_error = exc
                time.sleep(0.1)
        if last_error is not None:
            raise last_error
        raise RuntimeError("timed out waiting for exclusive base publisher ownership")

    def _publish_twist(self, vx: float, vy: float, wz: float) -> None:
        if not self.base_control_active.is_set():
            return
        if self.emergency_event.is_set() and any(value != 0.0 for value in (vx, vy, wz)):
            return
        message = self.Twist()
        message.linear.x = float(vx)
        message.linear.y = float(vy)
        message.angular.z = float(wz)
        self.base_publisher.publish(message)

    def _publish_zero(self) -> None:
        self._publish_twist(0.0, 0.0, 0.0)

    def _zero_timer(self) -> None:
        if self.base_control_active.is_set() and self.state not in {
            PairState.RUNNING_B0,
            PairState.RUNNING_B1,
        }:
            self._publish_zero()

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
        print("Paired base control acquired; F710 is temporarily disabled.")

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
        print("Pair complete; F710 restored. Reposition the base, then press right A again.")

    def _base_callback(self, message: Any) -> None:
        with self.feedback_lock:
            self.base_feedback = (
                float(message.linear.x),
                float(message.linear.y),
                float(message.angular.z),
            )
            self.base_received_at = time.monotonic()

    def _joint_callback(self, message: Any) -> None:
        indices = {str(name): index for index, name in enumerate(message.name)}
        names = self.config["ros"]["joint_names"]
        if any(name not in indices for name in names):
            return
        position = np.asarray([message.position[indices[name]] for name in names], dtype=float)
        effort = np.asarray(
            [message.effort[indices[name]] if indices[name] < len(message.effort) else 0.0 for name in names],
            dtype=float,
        )
        if position.shape != (14,) or not np.isfinite(position).all():
            return
        with self.feedback_lock:
            self.arm_position = position
            self.arm_effort = effort
            self.arm_received_at = time.monotonic()

    def _pressed(self, value: int) -> bool:
        return value == 0 if self.config["buttons"].get("active_low", False) else value != 0

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
        rises = []
        for name, index in values.items():
            if self._pressed(buttons[index]) and not self._pressed(self.previous_buttons[index]):
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

    def _emergency_stop(self, source: str) -> None:
        self.emergency_event.set()
        with self.machine_lock:
            action = self.machine.press("right_c")
        self._publish_zero()
        self.event_log.write("emergency_stop", source=source, action=str(action))
        self.actions.put("emergency")

    def _recording_too_short(self, button: str) -> bool:
        state = self.state
        expected = (state == PairState.RECORDING_S1 and button == "right_a") or (
            state == PairState.RECORDING_S2 and button == "right_b"
        )
        if not expected:
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
            f"Button ignored: {state.value} has recorded {elapsed:.1f}s; "
            f"wait until {minimum:.1f}s.",
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
                    observed_state = PairState.ABORTED
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
                if action == PairAction.RUN_B0:
                    self._acquire_base_control()
                    self._begin_attempt()
                    self._run_base_and_start_skill("B0", "S1")
                elif action == PairAction.MARK_S1_GRASP_COMPLETE:
                    marker_ns = time.time_ns()
                    self.attempt_record.setdefault("phase_markers", {})[
                        "s1_grasp_complete_ns"
                    ] = marker_ns
                    self._update_attempt("awaiting_b1_while_recording_s1")
                    self.event_log.write(
                        "s1_grasp_complete",
                        episode=str(self.collector_episode_path),
                        marker_ns=marker_ns,
                    )
                    print(
                        "S1 grasp marked; recording continues. Press right B to run B1.",
                        flush=True,
                    )
                elif action == PairAction.RUN_B1:
                    self._run_b1_inside_continuous_episode_and_switch_to_s2()
                elif action == PairAction.STOP_S2:
                    s2_complete_ns = time.time_ns()
                    self.attempt_record.setdefault("phase_markers", {})[
                        "s2_complete_ns"
                    ] = s2_complete_ns
                    source_episode = self.collector_episode_path
                    self._write_attempt()
                    self._stop_collector()
                    self._finalize_continuous_segments(source_episode, s2_complete_ns)
                    with self.machine_lock:
                        self.machine.collector_stopped("S2")
                    self._finish_attempt("unreviewed")
                    self._release_base_control()
                    with self.machine_lock:
                        self.machine.reset_completed_pair()
                    self.attempt_id = None
                    self.attempt_path = None
                    self.attempt_record = {}
                    self.event_log.write("pair_reset", state=self.state.value)
            except Exception as exc:  # noqa: BLE001
                detail = f"{type(exc).__name__}: {exc}"
                self.event_log.write("safety_stop", error=detail)
                print(f"SAFETY STOP: {detail}", flush=True)
                self.emergency_event.set()
                with self.machine_lock:
                    self.machine.abort()
                self._publish_zero()
                self._stop_collector(emergency=True)
                self._finish_attempt("aborted", error=f"{type(exc).__name__}: {exc}")

    def _begin_attempt(self) -> None:
        stamp = dt.datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
        self.attempt_id = f"pair_{stamp}_{time.time_ns() % 1_000_000:06d}"
        directory = Path(str(self.config["collection"]["attempt_directory"])).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        self.attempt_path = directory / f"{self.attempt_id}.json"
        self.attempt_record = {
            "schema_version": 1,
            "workflow": "continuous_s1_b1_s2_single_collector_v3",
            "parent_attempt_id": self.attempt_id,
            "created_at": dt.datetime.now().astimezone().isoformat(),
            "button_sequence": ["right_a", "right_a", "right_b", "right_b"],
            "button_indices": {"right_a": 11, "right_b": 12, "right_c_emergency": 13},
            "priors": {
                stage: {
                    "version": self.priors["priors"][stage]["version"],
                    "stream": self.priors["priors"][stage]["stream"],
                    "sha256": self.priors["priors"][stage]["sha256"],
                }
                for stage in ("B0", "B1")
            },
            "episodes": {},
            "phase_markers": {},
            "continuous_capture": {
                "single_collector": True,
                "camera_restart_between_s1_s2": False,
                "segments_file": "paired_segments.json",
            },
            "status": "running",
        }
        self._write_attempt()

    def _write_attempt(self) -> None:
        if self.attempt_path is None:
            return
        temporary = self.attempt_path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(self.attempt_record, indent=2, ensure_ascii=False), encoding="utf-8")
        temporary.replace(self.attempt_path)

    def _update_attempt(self, status: str) -> None:
        if not self.attempt_record:
            return
        self.attempt_record["status"] = status
        self.attempt_record["updated_at"] = dt.datetime.now().astimezone().isoformat()
        self._write_attempt()

    def _finish_attempt(self, outcome: str, *, error: str | None = None) -> None:
        if not self.attempt_record:
            return
        self.attempt_record["status"] = "complete" if outcome == "unreviewed" else outcome
        self.attempt_record["outcome"] = outcome
        self.attempt_record["ended_at"] = dt.datetime.now().astimezone().isoformat()
        if error:
            self.attempt_record["error"] = error
        self._write_attempt()

    def _feedback_snapshot(
        self, stage: str
    ) -> tuple[tuple[float, float, float], np.ndarray | None, np.ndarray | None]:
        with self.feedback_lock:
            base = self.base_feedback
            base_age = time.monotonic() - self.base_received_at
            arm = None if self.arm_position is None else self.arm_position.copy()
            effort = None if self.arm_effort is None else self.arm_effort.copy()
            arm_age = time.monotonic() - self.arm_received_at
        if base is None or base_age > float(self.config["timing"]["base_feedback_timeout_sec"]):
            raise SafetyStop(f"base feedback is unavailable or stale ({base_age:.3f}s)")
        if stage == "B1" and (
            arm is None
            or effort is None
            or arm_age > float(self.config["timing"]["joint_feedback_timeout_sec"])
        ):
            raise SafetyStop(f"arm feedback is unavailable or stale ({arm_age:.3f}s)")
        return base, arm, effort

    def _check_base_stage_safety(
        self,
        stage: str,
        *,
        check_publisher: bool = False,
    ) -> None:
        if self.emergency_event.is_set():
            raise SafetyStop("right C emergency stop is latched")
        if check_publisher:
            self._assert_no_base_publisher()
        self._feedback_snapshot(stage)

    def _run_prior(self, stage: str) -> None:
        self._assert_no_base_publisher()
        self._feedback_snapshot(stage)
        samples = self.prior_streams[stage]
        started = time.monotonic()
        published_count = 0
        integrated_x = 0.0
        integrated_y = 0.0
        integrated_yaw = 0.0
        previous_sample = None
        self.event_log.write("base_stage_start", stage=stage, sample_count=len(samples))
        try:
            for sample in samples:
                self._check_base_stage_safety(stage, check_publisher=True)
                deadline = started + sample.timestamp_sec
                while time.monotonic() < deadline:
                    self._check_base_stage_safety(stage)
                    time.sleep(min(0.01, max(0.0, deadline - time.monotonic())))
                self._check_base_stage_safety(stage)
                if previous_sample is not None:
                    delta = sample.timestamp_sec - previous_sample.timestamp_sec
                    integrated_x += previous_sample.vx * delta
                    integrated_y += previous_sample.vy * delta
                    integrated_yaw += previous_sample.wz * delta
                self._publish_twist(sample.vx, sample.vy, sample.wz)
                previous_sample = sample
                published_count += 1
            self._publish_zero()
            self._wait_base_zero(stage)
            delay_deadline = time.monotonic() + float(
                self.config["timing"]["post_base_stable_delay_sec"]
            )
            while time.monotonic() < delay_deadline:
                self._check_base_stage_safety(stage, check_publisher=True)
                self._publish_zero()
                time.sleep(1.0 / 15.0)
        except Exception:
            self._publish_zero()
            self.event_log.write(
                "base_stage_aborted",
                stage=stage,
                published_sample_count=published_count,
                sample_count=len(samples),
                elapsed_sec=time.monotonic() - started,
                integrated_command={
                    "x_m": integrated_x,
                    "y_m": integrated_y,
                    "yaw_rad": integrated_yaw,
                },
            )
            raise
        self.event_log.write(
            "base_stage_complete",
            stage=stage,
            published_sample_count=published_count,
            elapsed_sec=time.monotonic() - started,
            integrated_command={
                "x_m": integrated_x,
                "y_m": integrated_y,
                "yaw_rad": integrated_yaw,
            },
        )

    def _wait_base_zero(self, stage: str) -> None:
        stable_since: float | None = None
        deadline = time.monotonic() + 5.0
        tolerance = float(self.config["timing"]["base_feedback_zero_tolerance"])
        required = float(self.config["timing"]["base_zero_stable_sec"])
        while time.monotonic() < deadline:
            self._check_base_stage_safety(stage, check_publisher=True)
            base, _arm, _effort = self._feedback_snapshot(stage)
            self._publish_zero()
            if max(abs(value) for value in base) <= tolerance:
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= required:
                    return
            else:
                stable_since = None
            time.sleep(1.0 / 15.0)
        raise SafetyStop(f"{stage} base feedback did not remain zero for {required}s")

    def _run_base_and_start_skill(self, stage: str, skill: str) -> None:
        self._run_prior(stage)
        with self.machine_lock:
            self.machine.base_complete(stage)
        is_s1_after_b0 = stage == "B0" and skill == "S1"
        if is_s1_after_b0:
            b0_complete_ns = time.time_ns()
            self.attempt_record.setdefault("phase_markers", {})[
                "b0_complete_ns"
            ] = b0_complete_ns
            self._update_attempt("b0_complete_waiting_for_s1_collector")
            self.event_log.write(
                "b0_complete_waiting_for_s1_collector",
                marker_ns=b0_complete_ns,
            )
            print(
                "\n"
                "========================================================================\n"
                "[B0 COMPLETE] B0 已结束，底盘已停稳。\n"
                "[WAIT] 正在启动并检查三相机、ROS 和双手数据。\n"
                "请保持机械臂不动；看到 [S1 READY] 前不要开始抬臂。\n"
                "========================================================================",
                flush=True,
            )
        self._start_collector(skill)
        with self.machine_lock:
            self.machine.collector_started(skill)
        if is_s1_after_b0:
            ready_ns = time.time_ns()
            self.attempt_record.setdefault("phase_markers", {})[
                "s1_operator_ready_ns"
            ] = ready_ns
        self._update_attempt(f"recording_{skill.lower()}")
        if is_s1_after_b0:
            self.event_log.write(
                "s1_operator_ready",
                marker_ns=ready_ns,
                recording_start_ns=self.collector_recording_start_ns,
                episode=str(self.collector_episode_path),
            )
            print(
                "\n"
                "========================================================================\n"
                "[S1 READY] 采集器已就绪，现在可以开始采集 S1。\n"
                "请开始抬起左臂并抓取柜门把手；抓稳后按右 A 标记。\n"
                "========================================================================",
                flush=True,
            )

    def _run_b1_inside_continuous_episode_and_switch_to_s2(self) -> None:
        if self.collector_process is None or self.current_skill != "S1":
            raise RuntimeError("B1 requires the continuous S1 collector to be running")
        started_ns = time.time_ns()
        self.attempt_record.setdefault("phase_markers", {})["b1_start_ns"] = started_ns
        self._update_attempt("recording_s1_during_b1")
        self.event_log.write(
            "s1_concurrent_base_start",
            skill="S1",
            stage="B1",
            episode=str(self.collector_episode_path),
            marker_ns=started_ns,
        )
        self._run_prior("B1")
        with self.machine_lock:
            self.machine.base_complete("B1")
        completed_ns = time.time_ns()
        self.attempt_record.setdefault("phase_markers", {})[
            "b1_complete_ns"
        ] = completed_ns
        self._write_attempt()
        self.event_log.write(
            "s1_concurrent_base_complete",
            skill="S1",
            stage="B1",
            episode=str(self.collector_episode_path),
            marker_ns=completed_ns,
        )
        source_episode = self.collector_episode_path
        if source_episode is None or self.collector_recording_start_ns is None:
            raise RuntimeError("continuous collector has no source episode or start timestamp")
        s2_skill = self.skills["S2"]
        markers = self.attempt_record.setdefault("phase_markers", {})
        markers["s2_start_ns"] = completed_ns
        self.attempt_record["source_episode"] = str(source_episode)
        self.attempt_record["episodes"]["S1"].update(
            {
                "status": "segment_complete",
                "segment_start_ns": self.collector_recording_start_ns,
                "segment_end_ns": completed_ns,
                "end_exclusive": True,
            }
        )
        self.attempt_record["episodes"]["S2"] = {
            "path": str(source_episode),
            "source_episode": str(source_episode),
            "prompt_version": str(s2_skill["prompt_version"]),
            "status": "recording",
            "segment_start_ns": completed_ns,
            "start_inclusive": True,
        }
        self.collector_started_at = time.monotonic()
        with self.machine_lock:
            self.machine.switch_to_s2()
        self._update_attempt("recording_s2")
        self.event_log.write(
            "continuous_skill_transition",
            source_episode=str(source_episode),
            from_skill="S1",
            to_skill="S2",
            marker_ns=completed_ns,
            collector_pid=self.collector_process.pid,
            camera_restart=False,
        )
        print(
            "B1 complete; S2 segment started in the same collector with cameras still "
            "running. Perform release/back-of-hand push, then press right B to stop.",
            flush=True,
        )

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
        if skill == "S1":
            command.append("--allow-base-motion")
        if collection.get("no_offline_align", True):
            command.append("--no-offline-align")
        return command

    def _find_episode(self, episode_name: str) -> Path | None:
        root = Path(str(self.config["collection"]["output_root"])).expanduser()
        if not root.is_dir():
            return None
        today = dt.datetime.now().astimezone().strftime("%Y-%m-%d")
        day = root / today
        if not day.is_dir():
            return None
        matches = sorted(day.glob(f"*_{episode_name}*"), key=lambda path: path.stat().st_mtime)
        return matches[-1] if matches else None

    def _start_collector(self, skill: str) -> None:
        with self.collector_lock:
            self._start_collector_locked(skill)

    def _start_collector_locked(self, skill: str) -> None:
        if self.attempt_id is None:
            raise RuntimeError("paired attempt has no id")
        if self.collector_process is not None:
            raise RuntimeError("a collector process is already running")
        if skill != "S1":
            raise RuntimeError("paired collection starts one S1-configured continuous collector")
        suffix = f"{self.attempt_id}_continuous_s1_b1_s2"
        log_dir = Path(str(self.config["collection"]["log_directory"])).expanduser()
        log_dir.mkdir(parents=True, exist_ok=True)
        self.collector_log_path = log_dir / f"collect_{suffix}.log"
        self.collector_log_handle = self.collector_log_path.open("a", encoding="utf-8", buffering=1)
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
        self.attempt_record["source_episode"] = str(episode_path)
        self.attempt_record["continuous_capture"]["collector_pid"] = self.collector_process.pid
        self.attempt_record["episodes"][skill] = {
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
        self.event_log.write("collector_started", skill=skill, episode=str(episode_path))

    def _stop_collector(self, *, emergency: bool = False) -> None:
        with self.collector_lock:
            self._stop_collector_locked(emergency=emergency)

    def _stop_collector_locked(self, *, emergency: bool = False) -> None:
        process = self.collector_process
        if process is None:
            return
        skill = self.current_skill or "unknown"
        if process.poll() is None:
            with contextlib.suppress(ProcessLookupError):
                # Let the collector notify and join its camera children cleanly.
                # Signalling the process group interrupts camera.close() and can
                # leave the ZED unavailable for the next episode.
                os.kill(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=float(self.config["timing"]["collector_stop_timeout_sec"]))
            except subprocess.TimeoutExpired:
                with contextlib.suppress(ProcessLookupError):
                    os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    with contextlib.suppress(ProcessLookupError):
                        os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=2)
        code = process.returncode
        if self.collector_log_handle is not None:
            self.collector_log_handle.close()
            self.collector_log_handle = None
        episode = self.collector_episode_path
        if episode is not None and (episode / "manifest.json").is_file():
            manifest = json.loads((episode / "manifest.json").read_text(encoding="utf-8"))
            status = str(manifest.get("status"))
        else:
            status = "missing"
        if skill in self.attempt_record.get("episodes", {}):
            self.attempt_record["episodes"][skill].update(
                {"status": status, "collector_exit_code": code}
            )
            self._write_attempt()
        self.event_log.write(
            "collector_stopped", skill=skill, episode=str(episode), status=status, exit_code=code
        )
        self.collector_process = None
        self.collector_log_path = None
        self.collector_episode_path = None
        self.collector_started_at = 0.0
        self.collector_recording_start_ns = None
        self.current_skill = None
        if not emergency and (code not in (0, 130, -signal.SIGINT) or status != "complete"):
            raise RuntimeError(f"{skill} collector stopped with code={code}, status={status}")

    @staticmethod
    def _atomic_write_json(path: Path, value: dict[str, Any]) -> None:
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding="utf-8")
        temporary.replace(path)

    def _finalize_continuous_segments(
        self, source_episode: Path | None, s2_complete_ns: int
    ) -> None:
        if source_episode is None:
            raise RuntimeError("continuous source episode is missing")
        manifest_path = source_episode / "manifest.json"
        frames_path = source_episode / "frames.jsonl"
        if not manifest_path.is_file() or not frames_path.is_file():
            raise RuntimeError(f"continuous source episode is incomplete: {source_episode}")
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest.get("status") != "complete":
            raise RuntimeError("continuous source manifest is not complete")
        markers = self.attempt_record.get("phase_markers")
        if not isinstance(markers, dict):
            raise RuntimeError("continuous attempt has no phase markers")
        s2_start_ns = int(markers["s2_start_ns"])
        recording_start_ns = int(manifest["recording_start_ns"])
        recording_end_ns = int(manifest["recording_end_ns"])
        if not recording_start_ns < s2_start_ns < recording_end_ns:
            raise RuntimeError(
                "S2 boundary is outside the continuous recording: "
                f"{recording_start_ns} < {s2_start_ns} < {recording_end_ns}"
            )

        ranges: dict[str, dict[str, Any]] = {
            "S1": {
                "start_ns": recording_start_ns,
                "end_ns": s2_start_ns,
                "start_inclusive": True,
                "end_inclusive": False,
                "frame_indices": [],
            },
            "S2": {
                "start_ns": s2_start_ns,
                "end_ns": recording_end_ns,
                "start_inclusive": True,
                "end_inclusive": True,
                "frame_indices": [],
            },
        }
        with frames_path.open("r", encoding="utf-8") as stream:
            for line_number, line in enumerate(stream, start=1):
                if not line.strip():
                    continue
                try:
                    frame = json.loads(line)
                    frame_index = int(frame["frame_index"])
                    timestamp_ns = int(frame["timestamp_ns"])
                except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
                    raise RuntimeError(
                        f"invalid continuous frame at {frames_path}:{line_number}: {exc}"
                    ) from exc
                skill = "S1" if timestamp_ns < s2_start_ns else "S2"
                ranges[skill]["frame_indices"].append(frame_index)

        minimum_frames = int(
            round(
                float(self.config["timing"]["minimum_skill_recording_sec"])
                * float(self.config["timing"]["hz"])
            )
        )
        segments: list[dict[str, Any]] = []
        for skill_id in ("S1", "S2"):
            indices = ranges[skill_id].pop("frame_indices")
            if len(indices) < minimum_frames:
                raise RuntimeError(
                    f"continuous {skill_id} segment has {len(indices)} frames; "
                    f"minimum is {minimum_frames}"
                )
            skill = self.skills[skill_id]
            segments.append(
                {
                    "skill_id": skill_id,
                    "task": str(skill["prompt"]),
                    "prompt_version": str(skill["prompt_version"]),
                    **ranges[skill_id],
                    "source_start_frame": indices[0],
                    "source_end_frame": indices[-1],
                    "frame_count": len(indices),
                    "base_motion": {
                        "allowed": skill_id == "S1",
                        "concurrent_prior": "B1" if skill_id == "S1" else None,
                        "included_in_training_action": False,
                    },
                }
            )

        segments_document = {
            "schema_version": 1,
            "workflow": "continuous_s1_b1_s2_single_collector_v3",
            "parent_attempt_id": self.attempt_id,
            "source_episode": str(source_episode),
            "source_frames": "frames.jsonl",
            "camera_pipeline_continuous": True,
            "camera_restart_count_between_segments": 0,
            "phase_markers": dict(markers),
            "segments": segments,
        }
        segments_path = source_episode / "paired_segments.json"
        self._atomic_write_json(segments_path, segments_document)

        manifest["training_eligible_as_atomic_episode"] = False
        manifest["paired_continuous_source"] = {
            "workflow": segments_document["workflow"],
            "segments_file": segments_path.name,
            "camera_pipeline_continuous": True,
            "camera_restart_count_between_segments": 0,
        }
        self._atomic_write_json(manifest_path, manifest)

        self.attempt_record["source_episode"] = str(source_episode)
        self.attempt_record["segments_file"] = str(segments_path)
        for segment in segments:
            skill_id = str(segment["skill_id"])
            self.attempt_record["episodes"][skill_id].update(
                {
                    "path": str(source_episode),
                    "source_episode": str(source_episode),
                    "status": "segment_complete",
                    "segment_start_ns": segment["start_ns"],
                    "segment_end_ns": segment["end_ns"],
                    "source_start_frame": segment["source_start_frame"],
                    "source_end_frame": segment["source_end_frame"],
                    "frame_count": segment["frame_count"],
                    "segments_file": str(segments_path),
                }
            )
        self._write_attempt()
        self.event_log.write(
            "continuous_segments_finalized",
            source_episode=str(source_episode),
            segments_file=str(segments_path),
            s1_frame_count=segments[0]["frame_count"],
            s2_frame_count=segments[1]["frame_count"],
            s2_complete_marker_ns=s2_complete_ns,
            camera_restart_count=0,
        )

    def spin(self) -> None:
        self.executor.spin()

    def close(self) -> None:
        self.shutdown_event.set()
        self.emergency_event.set()
        self._stop_collector(emergency=True)
        count = int(self.config["safety"]["emergency_zero_publish_count"])
        for _ in range(count):
            self._publish_zero()
            time.sleep(1.0 / 15.0)
        worker_timeout = (
            float(self.config["base_publisher_takeover"]["status_timeout_sec"]) * 2
            + float(self.config["base_publisher_takeover"]["service_stop_timeout_sec"])
            + float(self.config["base_publisher_takeover"]["release_timeout_sec"])
            + 5.0
        )
        self.worker.join(timeout=worker_timeout)
        if self.executor is not None:
            self.executor.shutdown(timeout_sec=2.0)
            self.executor.remove_node(self.node)
            if self.hand_node is not None:
                self.executor.remove_node(self.hand_node)
        if self.hand_node is not None:
            self.hand_node.close()
            self.hand_node.destroy_node()
            self.hand_node = None
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()
        if self.hand_lease is not None:
            self.hand_lease.close()
            self.hand_lease = None
        self.event_log.close()


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
        for stage in ("B0", "B1")
    }
    lease = FileLease(Path(str(config["safety"]["base_command_lock"])).expanduser())
    takeover = ManagedServiceTakeover(config)
    lease.acquire()
    runtime = None
    try:
        runtime = PairedCollectionRuntime(
            config, priors, collector, prior_streams, takeover
        )
        print(
            "Paired collection ready with F710 active while idle: left C controls "
            "O6 action 4; right A runs B0, then wait for [S1 READY] before moving "
            "the arm; press right A again to mark grasp; right B runs B1 and then "
            "stops S2; right C is emergency."
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
        raise SystemExit(f"real execution blocked: pass --acknowledge-motion {acknowledgement}")
    try:
        return execute(config_path)
    except SafetyStop as exc:
        raise SystemExit(f"real execution blocked: {exc}") from None


if __name__ == "__main__":
    raise SystemExit(main())
