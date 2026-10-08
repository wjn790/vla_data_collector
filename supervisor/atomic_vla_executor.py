#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import queue
import select
import signal
import subprocess
import sys
import threading
import time
from contextlib import ExitStack
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

import numpy as np
import yaml


COLLECTOR_ROOT = Path(__file__).resolve().parents[1]
if str(COLLECTOR_ROOT) not in sys.path:
    sys.path.insert(0, str(COLLECTOR_ROOT))

from supervisor.core import ArmCommandLimiter, SafetyStop, load_yaml, validate_action_chunk


MOTION_ACKNOWLEDGEMENT = "SVT_ATOMIC_VLA_V1"


class EventLog:
    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.handle = path.open("a", encoding="utf-8", buffering=1)
        self.lock = threading.Lock()

    def write(self, event: str, **values: Any) -> None:
        row = {"timestamp_ns": time.time_ns(), "event": event, **values}
        with self.lock:
            self.handle.write(
                json.dumps(
                    row,
                    ensure_ascii=False,
                    default=lambda value: value.tolist()
                    if isinstance(value, np.ndarray)
                    else str(value),
                )
                + "\n"
            )

    def close(self) -> None:
        with self.lock:
            self.handle.close()


@dataclass(frozen=True)
class ProcessRecord:
    pid: int
    argv: tuple[str, ...]

    @property
    def command(self) -> str:
        return " ".join(self.argv)


def process_matches(record: ProcessRecord, fragments: list[str]) -> bool:
    return all(str(fragment) in record.command for fragment in fragments)


class RuntimeTakeover:
    """Temporarily releases arm/base command publishers and restores prior state."""

    def __init__(self, config: dict[str, Any], event_log: EventLog) -> None:
        self.config = config["runtime_takeover"]
        self.event_log = event_log
        self.exo_command: tuple[str, ...] | None = None
        self.exo_stop_attempted = False
        self.f710_was_active = False
        self.f710_stopped = False
        self.motion_started = False

    @staticmethod
    def _processes() -> list[ProcessRecord]:
        records: list[ProcessRecord] = []
        for entry in Path("/proc").iterdir():
            if not entry.name.isdigit():
                continue
            try:
                raw = (entry / "cmdline").read_bytes()
                argv = tuple(
                    part.decode("utf-8", "replace")
                    for part in raw.split(b"\0")
                    if part
                )
            except (FileNotFoundError, PermissionError, ProcessLookupError):
                continue
            if argv:
                records.append(ProcessRecord(int(entry.name), argv))
        return records

    def _find(self, key: str) -> list[ProcessRecord]:
        fragments = [str(value) for value in self.config[key]]
        return [record for record in self._processes() if process_matches(record, fragments)]

    @staticmethod
    def _alive(pid: int) -> bool:
        try:
            state = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8").split()[2]
        except (FileNotFoundError, ProcessLookupError):
            return False
        return state != "Z"

    def _wait_absent(self, keys: tuple[str, ...], timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if not any(self._find(key) for key in keys):
                return True
            time.sleep(0.1)
        return not any(self._find(key) for key in keys)

    def _wait_present(self, key: str, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._find(key):
                return True
            time.sleep(0.1)
        return bool(self._find(key))

    def _service_active(self) -> bool:
        result = subprocess.run(
            [str(self.config["systemctl"]), "is-active", "--quiet", str(self.config["f710_service"])],
            check=False,
        )
        return result.returncode == 0

    def _sudo(self, *arguments: str) -> None:
        command = [str(self.config["sudo"]), "-n", *arguments]
        result = subprocess.run(
            command,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        if result.returncode != 0:
            detail = (result.stdout or "").strip()
            raise RuntimeError(
                f"command failed ({' '.join(command)}): {detail or result.returncode}"
            )

    def _stop_exoskeleton_bridge(self) -> None:
        launches = self._find("exoskeleton_launch_match")
        nodes = self._find("exoskeleton_node_match")
        if not launches and not nodes:
            self.event_log.write("takeover_exoskeleton_already_stopped")
            return
        if len(launches) > 1 or (not launches and len(nodes) > 1):
            raise RuntimeError(
                "cannot take over exoskeleton bridge with multiple matching processes"
            )
        owner = launches[0] if launches else nodes[0]
        self.exo_command = owner.argv
        self.exo_stop_attempted = True
        os.kill(owner.pid, signal.SIGINT)
        timeout = float(self.config["process_stop_timeout_sec"])
        if not self._wait_absent(
            ("exoskeleton_launch_match", "exoskeleton_node_match"), timeout
        ):
            for record in self._find("exoskeleton_launch_match") + self._find(
                "exoskeleton_node_match"
            ):
                if self._alive(record.pid):
                    os.kill(record.pid, signal.SIGTERM)
            if not self._wait_absent(
                ("exoskeleton_launch_match", "exoskeleton_node_match"), 3.0
            ):
                raise RuntimeError("exoskeleton bridge did not stop")
        self.event_log.write(
            "takeover_exoskeleton_stopped",
            pid=owner.pid,
            command=owner.command,
        )
        print("Stopped exoskeleton_bridge_node for VLA execution.")

    def _stop_f710(self) -> None:
        self.f710_was_active = self._service_active()
        if not self.f710_was_active:
            self.event_log.write("takeover_f710_already_stopped")
            return
        self._sudo(str(self.config["systemctl"]), "stop", str(self.config["f710_service"]))
        deadline = time.monotonic() + float(self.config["service_timeout_sec"])
        while time.monotonic() < deadline and self._service_active():
            time.sleep(0.1)
        if self._service_active():
            raise RuntimeError(f"{self.config['f710_service']} did not stop")
        self.f710_stopped = True
        self.event_log.write("takeover_f710_stopped", service=self.config["f710_service"])
        print(f"Stopped {self.config['f710_service']} for VLA execution.")

    def acquire(self) -> None:
        try:
            self._stop_f710()
            self._stop_exoskeleton_bridge()
        except Exception:
            self.restore(require_operator_confirmation=False)
            raise
        self.event_log.write("runtime_takeover_acquired")
        print(
            "Arm/base teleoperation publishers are released. "
            "Press Ctrl+C or Enter to stop VLA execution."
        )

    def mark_motion_started(self) -> None:
        self.motion_started = True

    def _confirm_safe_exoskeleton_restore(self) -> None:
        if not self.motion_started:
            return
        if not sys.stdin.isatty():
            raise RuntimeError(
                "cannot safely restore exoskeleton bridge without an interactive terminal"
            )
        print(
            "\nVLA control has ended. Before restoring arm teleoperation:\n"
            "  1. Push BOTH exoskeleton Switches down.\n"
            "  2. Align both exoskeleton arms with the robot's current arm pose.\n"
            "  3. Type RESTORE. Keep both Switches down until alignment is complete."
        )
        if input("Type RESTORE to restart exoskeleton_bridge_node: ").strip() != "RESTORE":
            raise RuntimeError(
                "exoskeleton bridge remains stopped because RESTORE was not confirmed"
            )

    def _restore_exoskeleton_bridge(self, require_confirmation: bool) -> None:
        if not self.exo_stop_attempted or self.exo_command is None:
            return
        if self._find("exoskeleton_launch_match") or self._find("exoskeleton_node_match"):
            self.event_log.write("restore_exoskeleton_already_running")
            return
        if require_confirmation:
            self._confirm_safe_exoskeleton_restore()
        log_path = Path(str(self.config["exoskeleton_restore_log"])).expanduser()
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with log_path.open("ab", buffering=0) as log:
            process = subprocess.Popen(
                list(self.exo_command),
                stdin=subprocess.DEVNULL,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                close_fds=True,
            )
        if not self._wait_present(
            "exoskeleton_node_match", float(self.config["process_start_timeout_sec"])
        ):
            raise RuntimeError(
                f"exoskeleton bridge restart failed; see {log_path} (pid {process.pid})"
            )
        self.exo_stop_attempted = False
        self.event_log.write(
            "restore_exoskeleton_started",
            pid=process.pid,
            log=str(log_path),
        )
        print("Restored exoskeleton_bridge_node. Keep both Switches down until aligned.")

    def _restore_f710(self) -> None:
        if not self.f710_stopped:
            return
        self._sudo(str(self.config["systemctl"]), "start", str(self.config["f710_service"]))
        deadline = time.monotonic() + float(self.config["service_timeout_sec"])
        while time.monotonic() < deadline and not self._service_active():
            time.sleep(0.1)
        if not self._service_active():
            raise RuntimeError(f"{self.config['f710_service']} did not restart")
        self.f710_stopped = False
        self.event_log.write("restore_f710_started", service=self.config["f710_service"])
        print(f"Restored {self.config['f710_service']}.")

    def restore(self, *, require_operator_confirmation: bool = True) -> None:
        errors: list[str] = []
        try:
            self._restore_exoskeleton_bridge(require_operator_confirmation)
        except KeyboardInterrupt:
            errors.append(
                "exoskeleton: restore confirmation interrupted; bridge remains stopped"
            )
        except Exception as exc:  # noqa: BLE001
            errors.append(f"exoskeleton: {exc}")
        try:
            self._restore_f710()
        except Exception as exc:  # noqa: BLE001
            errors.append(f"F710: {exc}")
        if errors:
            message = "; ".join(errors)
            self.event_log.write("runtime_restore_failed", error=message)
            raise RuntimeError(message)
        self.event_log.write("runtime_takeover_restored")

    def __enter__(self) -> "RuntimeTakeover":
        self.acquire()
        return self

    def __exit__(self, exc_type: Any, _value: Any, _traceback: Any) -> None:
        try:
            self.restore()
        except Exception as restore_error:
            if exc_type is None:
                raise
            self.event_log.write(
                "runtime_restore_failed_during_exception",
                error=f"{type(restore_error).__name__}: {restore_error}",
            )
            print(f"Runtime restore also failed: {restore_error}", file=sys.stderr)


def resolve_path(config_path: Path, value: Any) -> Path:
    path = Path(str(value)).expanduser()
    return path if path.is_absolute() else (config_path.parent / path).resolve()


def load_all(config_path: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    config = load_yaml(config_path)
    collector_path = resolve_path(config_path, config["collector_config"])
    collector = load_yaml(collector_path)
    if config["robot_id"] != collector["robot_id"]:
        raise ValueError("robot_id differs between atomic executor and collector config")
    config["collector_config"] = str(collector_path)

    control = config["control"]
    if int(control["hz"]) != 15:
        raise ValueError("atomic executor must run at 15 Hz")
    if int(control["chunk_size"]) != 50:
        raise ValueError("atomic executor requires policy chunk_size=50")
    replan_frames = int(control["replan_frames"])
    max_chunk_frames = int(control["max_chunk_frames"])
    if not 1 <= replan_frames <= max_chunk_frames:
        raise ValueError("replan_frames must be between 1 and max_chunk_frames")
    if not 20 <= max_chunk_frames <= 25:
        raise ValueError("max_chunk_frames must remain in the safety range 20..25")
    if float(control["inference_timeout_sec"]) <= 0:
        raise ValueError("inference_timeout_sec must be positive")
    if not 0 < float(control["policy_response_max_age_sec"]) <= float(
        control["inference_timeout_sec"]
    ):
        raise ValueError(
            "policy_response_max_age_sec must be positive and no greater than "
            "inference_timeout_sec"
        )
    if float(control["policy_response_max_age_sec"]) > max_chunk_frames / float(
        control["hz"]
    ):
        raise ValueError(
            "policy_response_max_age_sec cannot exceed the executable chunk horizon"
        )
    if int(config["emergency_stop"]["button_index"]) != 13:
        raise ValueError("right-C emergency stop must remain buttons[13]")

    expected_actions = set(config["policy"]["expected_action_keys"])
    if expected_actions != {"action.arm.position", "action.hand.position"}:
        raise ValueError("policy expected_action_keys must be the SVT arm and hand keys")
    expected_model = str(config["policy"]["expected_model_path"]).strip()
    if not expected_model.startswith("/"):
        raise ValueError("policy expected_model_path must be an absolute path")
    prompt = str(config["task_prompt"]).strip()
    if not prompt:
        raise ValueError("task_prompt cannot be empty")
    takeover = config.get("runtime_takeover")
    required_takeover_keys = {
        "sudo",
        "systemctl",
        "f710_service",
        "exoskeleton_launch_match",
        "exoskeleton_node_match",
        "process_stop_timeout_sec",
        "process_start_timeout_sec",
        "service_timeout_sec",
        "exoskeleton_restore_log",
    }
    if not isinstance(takeover, dict) or not required_takeover_keys <= set(takeover):
        raise ValueError("runtime_takeover config is incomplete")
    return config, collector


def validate_policy_metadata(metadata: dict[str, Any], config: dict[str, Any]) -> None:
    policy = config["policy"]
    expected_model = str(policy["expected_model_path"])
    if str(metadata.get("model_path")) != expected_model:
        raise RuntimeError(
            "policy model mismatch: "
            f"expected {expected_model}, got {metadata.get('model_path')}"
        )
    if int(metadata.get("chunk_size", -1)) != int(config["control"]["chunk_size"]):
        raise RuntimeError(f"policy chunk metadata is incompatible: {metadata}")
    if set(metadata.get("action_keys", ())) != set(policy["expected_action_keys"]):
        raise RuntimeError(f"policy action metadata is incompatible: {metadata}")


@dataclass(frozen=True)
class PolicyRequest:
    request_id: int
    kind: str
    payload: dict[str, Any]
    timeout_sec: float
    submitted_at: float
    motion_overlapped: bool


@dataclass(frozen=True)
class PolicyResult:
    request: PolicyRequest
    response: dict[str, Any] | None
    elapsed_sec: float
    completed_at: float
    error: str | None


class AsyncPolicyWorker:
    """Owns the WebSocket in one thread so inference never blocks control."""

    def __init__(
        self,
        host: str,
        port: int,
        *,
        connect_timeout_sec: float,
        client_factory: Callable[..., Any] | None = None,
    ) -> None:
        if client_factory is None:
            from supervisor.policy_client import PolicyClient

            client_factory = PolicyClient
        self.host = str(host)
        self.port = int(port)
        self.connect_timeout_sec = float(connect_timeout_sec)
        self.client_factory = client_factory
        self.requests: queue.Queue[PolicyRequest | None] = queue.Queue(maxsize=1)
        self.results: queue.Queue[PolicyResult] = queue.Queue()
        self.ready = threading.Event()
        self.stop_event = threading.Event()
        self.metadata: dict[str, Any] | None = None
        self.startup_error: str | None = None
        self.next_request_id = 1
        self.thread = threading.Thread(
            target=self._run,
            name="svt-atomic-policy",
            daemon=True,
        )

    def start(self) -> None:
        self.thread.start()
        if not self.ready.wait(self.connect_timeout_sec + 1.0):
            raise RuntimeError("policy client startup timed out")
        if self.startup_error is not None:
            raise RuntimeError(f"policy client startup failed: {self.startup_error}")

    def _run(self) -> None:
        client: Any | None = None
        try:
            client = self.client_factory(
                self.host,
                self.port,
                connect_timeout_sec=self.connect_timeout_sec,
            )
            self.metadata = dict(client.metadata)
        except Exception as exc:  # noqa: BLE001
            self.startup_error = f"{type(exc).__name__}: {exc}"
            self.ready.set()
            return
        self.ready.set()
        try:
            while not self.stop_event.is_set():
                try:
                    request = self.requests.get(timeout=0.1)
                except queue.Empty:
                    continue
                if request is None:
                    return
                try:
                    response, elapsed = client.infer(request.payload, request.timeout_sec)
                    result = PolicyResult(
                        request=request,
                        response=response,
                        elapsed_sec=float(elapsed),
                        completed_at=time.monotonic(),
                        error=None,
                    )
                except Exception as exc:  # noqa: BLE001
                    result = PolicyResult(
                        request=request,
                        response=None,
                        elapsed_sec=time.monotonic() - request.submitted_at,
                        completed_at=time.monotonic(),
                        error=f"{type(exc).__name__}: {exc}",
                    )
                self.results.put(result)
        finally:
            if client is not None:
                client.close()

    def submit(
        self,
        kind: str,
        payload: dict[str, Any],
        timeout_sec: float,
        *,
        motion_overlapped: bool,
    ) -> int:
        request_id = self.next_request_id
        self.next_request_id += 1
        request = PolicyRequest(
            request_id=request_id,
            kind=str(kind),
            payload=payload,
            timeout_sec=float(timeout_sec),
            submitted_at=time.monotonic(),
            motion_overlapped=bool(motion_overlapped),
        )
        try:
            self.requests.put_nowait(request)
        except queue.Full as exc:
            raise RuntimeError("policy already has a queued request") from exc
        return request_id

    def wait_result(self, request_id: int, timeout_sec: float) -> PolicyResult:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            try:
                result = self.results.get(timeout=min(0.1, deadline - time.monotonic()))
            except queue.Empty:
                continue
            if result.request.request_id != request_id:
                raise RuntimeError(
                    f"unexpected policy result {result.request.request_id}, expected {request_id}"
                )
            return result
        raise RuntimeError(f"policy request {request_id} did not complete in time")

    def poll(self) -> PolicyResult | None:
        try:
            return self.results.get_nowait()
        except queue.Empty:
            return None

    def close(self) -> None:
        self.stop_event.set()
        try:
            self.requests.put_nowait(None)
        except queue.Full:
            pass
        if self.thread.is_alive():
            self.thread.join(timeout=self.connect_timeout_sec + 1.0)


@dataclass
class ChunkCursor:
    arm: np.ndarray
    hand: np.ndarray
    start_index: int
    max_frames: int
    executed: int = 0

    def next_action(self) -> tuple[np.ndarray, np.ndarray] | None:
        index = self.start_index + self.executed
        if self.executed >= self.max_frames or index >= self.arm.shape[0]:
            return None
        self.executed += 1
        return self.arm[index], self.hand[index]


def delayed_chunk_start(
    elapsed_sec: float,
    hz: float,
    chunk_size: int,
    max_chunk_frames: int,
    motion_overlapped: bool,
) -> int:
    if not motion_overlapped:
        return 0
    estimated = max(0, int(round(float(elapsed_sec) * float(hz))))
    return min(estimated, int(chunk_size) - int(max_chunk_frames))


class AtomicRuntime:
    def __init__(
        self,
        config: dict[str, Any],
        collector: dict[str, Any],
        event_log: EventLog,
    ) -> None:
        self.config = config
        self.collector = collector
        self.event_log = event_log
        self.control = config["control"]
        self.period = 1.0 / float(self.control["hz"])
        self.arm_limiter = ArmCommandLimiter(config["arm_limits"])
        self.arm_velocity = np.zeros(14, dtype=float)
        self.last_arm_command = np.zeros(14, dtype=float)
        self.last_hand_command = np.zeros(12, dtype=float)
        self.emergency_event = threading.Event()
        self.emergency_source: str | None = None
        self.previous_buttons: list[int] | None = None
        self.last_emergency_at = 0.0
        self.ros: Any | None = None
        self.hands: Any | None = None
        self.cameras: Any | None = None
        self.policy: AsyncPolicyWorker | None = None
        self.joy_subscription: Any | None = None
        self.ready_for_motion = False

    def open(self) -> None:
        from supervisor.camera_adapter import LiveCameraAdapter
        from supervisor.hand_adapter import HandHardwareAdapter
        from supervisor.ros_adapter import RosRobotAdapter

        # Camera workers use multiprocessing; start them before any local threads.
        self.cameras = LiveCameraAdapter(Path(str(self.config["collector_config"])))
        policy = self.config["policy"]
        self.policy = AsyncPolicyWorker(
            str(policy["host"]),
            int(policy["port"]),
            connect_timeout_sec=float(policy["connect_timeout_sec"]),
        )
        self.policy.start()
        assert self.policy.metadata is not None
        validate_policy_metadata(self.policy.metadata, self.config)
        reset_id = self.policy.submit(
            "reset",
            {"reset": True, "robo_name": str(policy["robot_name"])},
            float(self.control["inference_timeout_sec"]),
            motion_overlapped=False,
        )
        reset = self.policy.wait_result(
            reset_id,
            float(self.control["inference_timeout_sec"]) + 1.0,
        )
        if reset.error is not None:
            raise RuntimeError(f"policy reset failed: {reset.error}")

        self.ros = RosRobotAdapter(self.config["ros"])
        self.ros.wait_ready()
        self.hands = HandHardwareAdapter(self.config["hands"], self.collector)
        self.last_arm_command = self.ros.arm_state()[0]
        self.last_hand_command = self.hands.latched_action()
        from sensor_msgs.msg import Joy

        self.joy_subscription = self.ros.node.create_subscription(
            Joy,
            str(self.config["emergency_stop"]["joy_topic"]),
            self._joy_callback,
            10,
        )
        self.ready_for_motion = True
        self.event_log.write(
            "runtime_ready",
            policy_metadata=self.policy.metadata,
            reset_round_trip_ms=reset.elapsed_sec * 1000.0,
            task=self.config["task_prompt"],
        )

    def _pressed(self, value: int) -> bool:
        active_low = bool(self.config["emergency_stop"].get("active_low", False))
        return value == 0 if active_low else value != 0

    def _joy_callback(self, message: Any) -> None:
        buttons = [int(value) for value in message.buttons]
        index = int(self.config["emergency_stop"]["button_index"])
        if len(buttons) <= index:
            return
        pressed = self._pressed(buttons[index])
        previous_pressed = (
            False
            if self.previous_buttons is None or len(self.previous_buttons) <= index
            else self._pressed(self.previous_buttons[index])
        )
        self.previous_buttons = buttons
        now = time.monotonic()
        debounce = float(self.config["emergency_stop"]["debounce_sec"])
        if pressed and not previous_pressed and now - self.last_emergency_at >= debounce:
            self.last_emergency_at = now
            self.emergency_source = "right_c"
            self.emergency_event.set()
            self.event_log.write("emergency_stop_latched", source="right_c")

    def observation(self) -> dict[str, Any]:
        assert self.ros is not None and self.hands is not None and self.cameras is not None
        images = self.cameras.observation_images()
        arm = self.ros.arm_state()[0].astype(np.float32)
        hand = self.hands.state().astype(np.float32)
        images.update(
            {
                "observation.state.arm.position": arm,
                "observation.state.hand.position": hand,
                "task": str(self.config["task_prompt"]),
            }
        )
        return images

    def hold(self) -> None:
        assert self.ros is not None and self.hands is not None
        self.ros.publish_base_zero()
        self.ros.publish_arms(self.last_arm_command)
        self.hands.publish(self.last_hand_command)

    def final_hold(self) -> None:
        count = int(self.config["control"]["final_hold_frames"])
        for _ in range(count):
            self.hold()
            time.sleep(self.period)

    def emergency_hold(self, reason: str) -> None:
        self.event_log.write("latched_safety_hold", reason=reason)
        print(f"Latched safety hold: {reason}. Press Ctrl-C after the robot is safe.")
        try:
            while True:
                self.hold()
                time.sleep(self.period)
        except KeyboardInterrupt:
            self.event_log.write("latched_hold_operator_exit", reason=reason)

    def _check_feedback(self) -> None:
        assert self.ros is not None and self.hands is not None
        self.ros.arm_state(float(self.control["arm_feedback_max_age_sec"]))
        self.hands.state(float(self.control["hand_feedback_max_age_sec"]))

    def _operator_finished(self) -> bool:
        if not sys.stdin.isatty():
            return False
        readable, _, _ = select.select([sys.stdin], [], [], 0)
        if not readable:
            return False
        sys.stdin.readline()
        return True

    def run(self, max_duration_sec: float) -> None:
        assert self.ros is not None and self.hands is not None and self.policy is not None
        hz = float(self.control["hz"])
        chunk_size = int(self.control["chunk_size"])
        replan_frames = int(self.control["replan_frames"])
        max_chunk_frames = int(self.control["max_chunk_frames"])
        inference_timeout = float(self.control["inference_timeout_sec"])
        cursor: ChunkCursor | None = None
        pending: ChunkCursor | None = None
        inflight_id: int | None = None
        discard_through_id = 0
        last_ownership_check = 0.0
        last_watchdog_reason: str | None = None
        started = time.monotonic()
        next_tick = started

        def invalidate(reason: str) -> None:
            nonlocal cursor, pending, discard_through_id, last_watchdog_reason
            cursor = None
            pending = None
            if inflight_id is not None:
                discard_through_id = max(discard_through_id, inflight_id)
            if reason != last_watchdog_reason:
                self.event_log.write("watchdog_hold", reason=reason)
                print(f"Watchdog hold: {reason}")
                last_watchdog_reason = reason

        def submit_observation(motion_overlapped: bool) -> int:
            observation = self.observation()
            request_id = self.policy.submit(
                "inference",
                observation,
                inference_timeout,
                motion_overlapped=motion_overlapped,
            )
            self.event_log.write(
                "policy_request",
                request_id=request_id,
                motion_overlapped=motion_overlapped,
            )
            return request_id

        try:
            inflight_id = submit_observation(False)
        except Exception as exc:  # noqa: BLE001
            invalidate(f"initial observation failed: {type(exc).__name__}: {exc}")

        print(
            "Atomic VLA execution started. Press Ctrl+C or Enter to stop."
        )
        while time.monotonic() - started < max_duration_sec:
            next_tick += self.period
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            now = time.monotonic()
            lag_ms = max(0.0, (now - next_tick) * 1000.0)
            if lag_ms > float(self.control["loop_lag_warn_ms"]):
                self.event_log.write("control_loop_lag", lag_ms=lag_ms)
                if lag_ms > self.period * 3000.0:
                    next_tick = now

            if self.emergency_event.is_set():
                raise SafetyStop(f"emergency stop from {self.emergency_source or 'unknown'}")
            if self._operator_finished():
                self.event_log.write("normal_stop", source="operator_enter")
                return

            try:
                if now - last_ownership_check >= 1.0:
                    self.ros.assert_command_ownership()
                    last_ownership_check = now
                self._check_feedback()
                last_watchdog_reason = None
            except Exception as exc:  # noqa: BLE001
                invalidate(f"feedback/ownership unavailable: {type(exc).__name__}: {exc}")
                self.hold()
                continue

            result = self.policy.poll()
            if result is not None:
                if inflight_id != result.request.request_id:
                    raise SafetyStop(
                        f"policy result order mismatch: got {result.request.request_id}, "
                        f"expected {inflight_id}"
                    )
                inflight_id = None
                self.event_log.write(
                    "policy_response",
                    request_id=result.request.request_id,
                    round_trip_ms=result.elapsed_sec * 1000.0,
                    server_timing=None
                    if result.response is None
                    else result.response.get("server_timing"),
                    error=result.error,
                )
                if result.error is not None:
                    invalidate(f"policy inference failed: {result.error}")
                    raise SafetyStop(f"policy inference failed: {result.error}")
                if result.elapsed_sec > float(self.control["policy_response_max_age_sec"]):
                    invalidate(
                        "policy response stale: "
                        f"{result.elapsed_sec:.3f}s > "
                        f"{float(self.control['policy_response_max_age_sec']):.3f}s"
                    )
                    self.event_log.write(
                        "policy_response_discarded",
                        request_id=result.request.request_id,
                        reason="response exceeded policy_response_max_age_sec",
                    )
                if result.request.request_id <= discard_through_id:
                    self.event_log.write(
                        "policy_response_discarded",
                        request_id=result.request.request_id,
                        reason="observation invalidated while inference was running",
                    )
                elif result.elapsed_sec <= float(self.control["policy_response_max_age_sec"]):
                    assert result.response is not None
                    arm, hand = validate_action_chunk(result.response, chunk_size)
                    start_index = delayed_chunk_start(
                        result.elapsed_sec,
                        hz,
                        chunk_size,
                        max_chunk_frames,
                        result.request.motion_overlapped,
                    )
                    pending = ChunkCursor(arm, hand, start_index, max_chunk_frames)
                    self.event_log.write(
                        "policy_chunk_ready",
                        request_id=result.request.request_id,
                        start_index=start_index,
                    )

            if pending is not None and (cursor is None or cursor.executed >= replan_frames):
                replaced_after = None if cursor is None else cursor.executed
                cursor = pending
                pending = None
                self.event_log.write(
                    "policy_chunk_installed",
                    start_index=cursor.start_index,
                    replaced_after_frames=replaced_after,
                )

            if cursor is None:
                self.hold()
            else:
                action = cursor.next_action()
                if action is None:
                    self.event_log.write(
                        "chunk_exhausted_hold",
                        executed_frames=cursor.executed,
                    )
                    cursor = None
                    self.hold()
                else:
                    target_arm, target_hand = action
                    command, self.arm_velocity, violations = self.arm_limiter.limit(
                        target_arm,
                        self.last_arm_command,
                        self.arm_velocity,
                        self.period,
                    )
                    if violations:
                        self.event_log.write(
                            "arm_limit",
                            violations=violations,
                            target=target_arm,
                            command=command,
                        )
                    self.last_arm_command = command
                    self.last_hand_command = np.asarray(target_hand, dtype=np.float64)
                    self.ros.publish_base_zero()
                    self.ros.publish_arms(command)
                    self.hands.publish(self.last_hand_command)

            if inflight_id is None and pending is None:
                try:
                    inflight_id = submit_observation(cursor is not None)
                except Exception as exc:  # noqa: BLE001
                    invalidate(f"observation failed: {type(exc).__name__}: {exc}")
                    self.hold()

        self.event_log.write("normal_stop", source="max_duration", elapsed_sec=max_duration_sec)

    def close(self) -> None:
        if self.policy is not None:
            self.policy.close()
        if self.cameras is not None:
            self.cameras.close()
        if self.hands is not None:
            self.hands.close()
        if self.ros is not None:
            self.ros.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="SVT single-prompt LingBot VLA executor.")
    parser.add_argument(
        "--config",
        type=Path,
        default=COLLECTOR_ROOT / "config" / "atomic_vla_executor.yaml",
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--acknowledge-motion")
    parser.add_argument("--max-duration", type=float)
    parser.add_argument("--event-log", type=Path)
    return parser.parse_args()


def dry_run(config_path: Path) -> dict[str, Any]:
    config, _collector = load_all(config_path)
    return {
        "mode": "dry_run",
        "robot_id": config["robot_id"],
        "task_prompt": config["task_prompt"],
        "policy": {
            "endpoint": f"ws://{config['policy']['host']}:{config['policy']['port']}",
            "expected_model_path": config["policy"]["expected_model_path"],
            "chunk_size": config["control"]["chunk_size"],
            "replan_frames": config["control"]["replan_frames"],
            "max_chunk_frames": config["control"]["max_chunk_frames"],
        },
        "base_command": "zero_only",
        "optional_emergency_stop": {
            "topic": config["emergency_stop"]["joy_topic"],
            "button_index": config["emergency_stop"]["button_index"],
            "latched": True,
        },
        "operator_stop": {"ctrl_c": True, "enter": True},
        "runtime_takeover": {
            "stops_before_motion": [
                "exoskeleton_bridge_node",
                config["runtime_takeover"]["f710_service"],
            ],
            "restores_original_state_on_exit": True,
            "arm_restore_requires_operator_confirmation": True,
        },
        "motion_published": False,
    }


def execute(args: argparse.Namespace) -> int:
    config_path = args.config.expanduser().resolve()
    config, collector = load_all(config_path)
    max_duration = (
        float(args.max_duration)
        if args.max_duration is not None
        else float(config["control"]["max_duration_sec"])
    )
    if max_duration <= 0:
        raise SystemExit("--max-duration must be positive")
    event_log_path = (
        args.event_log.expanduser().resolve()
        if args.event_log is not None
        else Path(str(config["event_log"])).expanduser().resolve()
    )

    if not Path("/home/svt/glove_control").is_dir():
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "glove_control"))
    sys.path.insert(0, "/home/svt/glove_control")
    from hand_command_arbiter import HandCommandLease

    event_log = EventLog(event_log_path)
    runtime = AtomicRuntime(config, collector, event_log)
    takeover = RuntimeTakeover(config, event_log)
    try:
        with takeover:
            with ExitStack() as stack:
                stack.enter_context(
                    HandCommandLease(
                        config["control"]["command_owner_lock"],
                        mode="atomic_vla",
                        owner="atomic_vla_executor",
                    )
                )
                stack.enter_context(
                    HandCommandLease(
                        config["control"]["hand_owner_lock"],
                        mode="atomic_vla",
                        owner="atomic_vla_executor",
                    )
                )
                try:
                    runtime.open()
                    takeover.mark_motion_started()
                    runtime.run(max_duration)
                    runtime.final_hold()
                except KeyboardInterrupt:
                    event_log.write("normal_stop", source="keyboard_interrupt")
                    if runtime.ros is not None and runtime.hands is not None:
                        runtime.final_hold()
                except Exception as exc:  # noqa: BLE001
                    reason = f"{type(exc).__name__}: {exc}"
                    event_log.write("safety_stop", error=reason)
                    if runtime.ready_for_motion:
                        runtime.emergency_hold(reason)
                    else:
                        print(
                            f"Startup blocked before command interfaces were ready: {reason}",
                            file=sys.stderr,
                        )
                        return 1
                finally:
                    runtime.close()
    finally:
        event_log.close()
    return 0


def main() -> int:
    args = parse_args()
    config_path = args.config.expanduser().resolve()
    if not args.execute:
        print(json.dumps(dry_run(config_path), indent=2, ensure_ascii=False))
        return 0
    if args.acknowledge_motion != MOTION_ACKNOWLEDGEMENT:
        raise SystemExit(
            "real execution blocked: pass "
            f"--execute --acknowledge-motion {MOTION_ACKNOWLEDGEMENT}"
        )
    return execute(args)


if __name__ == "__main__":
    raise SystemExit(main())
