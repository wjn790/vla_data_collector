#!/usr/bin/env python3
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
import select
import signal
import sys
import threading
import time
from contextlib import ExitStack
from pathlib import Path
from typing import Any

import numpy as np
import yaml

COLLECTOR_ROOT = Path(__file__).resolve().parents[1]
if str(COLLECTOR_ROOT) not in sys.path:
    sys.path.insert(0, str(COLLECTOR_ROOT))

from supervisor.core import (
    ArmCommandLimiter,
    EXECUTION_PROFILES,
    FeedbackZeroGate,
    SafetyStop,
    StateMachine,
    enforce_skill_executors,
    load_base_prior_stream,
    load_skills,
    load_yaml,
    validate_action_chunk,
)


class EventLog:
    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.handle = path.open("a", encoding="utf-8", buffering=1)
        self.lock = threading.Lock()

    def write(self, event: str, **values: Any) -> None:
        with self.lock:
            self.handle.write(
                json.dumps(
                    {"timestamp_ns": time.time_ns(), "event": event, **values},
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


def _request_graceful_shutdown(signum: int, _frame: Any) -> None:
    """Turn terminal/session shutdown signals into the normal cleanup path."""
    signal_name = signal.Signals(signum).name
    raise KeyboardInterrupt(f"received {signal_name}")


def install_shutdown_handlers() -> dict[int, Any]:
    """Install handlers that let the F710 takeover cleanup run on exit."""
    previous: dict[int, Any] = {}
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        previous[signum] = signal.getsignal(signum)
        signal.signal(signum, _request_graceful_shutdown)
    return previous


def restore_shutdown_handlers(previous: dict[int, Any]) -> None:
    for signum, handler in previous.items():
        signal.signal(signum, handler)


def ignore_shutdown_signals(previous: dict[int, Any]) -> None:
    """Do not let a second Ctrl-C interrupt F710 restoration in finally."""
    for signum in previous:
        signal.signal(signum, signal.SIG_IGN)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="SVT prior-base / learned-arm long-task supervisor.")
    parser.add_argument(
        "--config",
        type=Path,
        default=COLLECTOR_ROOT / "config" / "supervisor.yaml",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--execute", action="store_true")
    mode.add_argument(
        "--shadow",
        action="store_true",
        help="Read live sensors and query the policy without creating command publishers.",
    )
    parser.add_argument(
        "--shadow-rounds",
        type=int,
        default=10,
        help="Policy requests per learned skill in --shadow mode (default: 10).",
    )
    parser.add_argument(
        "--profile",
        choices=sorted(EXECUTION_PROFILES),
        help="Execution profile. Defaults to execution_profile in the supervisor config.",
    )
    parser.add_argument(
        "--acknowledge-motion",
        help="Real execution requires the exact value SVT_LONG_TASK_V1.",
    )
    parser.add_argument(
        "--event-log",
        type=Path,
        default=COLLECTOR_ROOT / "logs" / "long_task_supervisor.jsonl",
    )
    parser.add_argument(
        "--action-log",
        type=Path,
        help="Per-frame model targets, published commands, and feedback JSONL path.",
    )
    return parser.parse_args()


def load_all(
    config_path: Path,
    requested_profile: str | None = None,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any], tuple[str, ...], Any, str]:
    supervisor = load_yaml(config_path)
    def config_relative(value: Any) -> Path:
        path = Path(str(value)).expanduser()
        return path if path.is_absolute() else (config_path.parent / path).resolve()

    skills_path = config_relative(supervisor["skills"])
    priors_path = config_relative(supervisor["base_priors"])
    collector_path = config_relative(supervisor["collector_config"])
    supervisor["skills"] = str(skills_path)
    supervisor["base_priors"] = str(priors_path)
    supervisor["collector_config"] = str(collector_path)
    full_chain, skills = load_skills(skills_path)
    priors = load_yaml(priors_path)
    collector = load_yaml(collector_path)
    if supervisor["robot_id"] != priors["robot_id"] or supervisor["robot_id"] != collector["robot_id"]:
        raise ValueError("robot_id differs across supervisor, priors, and collector configs")
    control = supervisor["control"]
    if int(control["hz"]) != 15 or int(control["chunk_size"]) != 50:
        raise ValueError("supervisor must use 15 Hz and chunk_size=50")
    if int(control["execute_frames"]) != 15:
        raise ValueError("supervisor must execute exactly fifteen frames per replan")
    prefetch_after_frames = int(control["prefetch_after_frames"])
    if not 1 <= prefetch_after_frames < int(control["execute_frames"]):
        raise ValueError("prefetch_after_frames must be within the executed action block")
    inference_watchdog_ms = int(control["inference_watchdog_ms"])
    if not 500 <= inference_watchdog_ms <= 1200:
        raise ValueError("supervisor inference watchdog must be between 500 and 1200 ms")
    configured_profile = str(supervisor.get("execution_profile", "full"))
    profile = requested_profile or configured_profile
    if profile not in EXECUTION_PROFILES:
        raise ValueError(f"unsupported execution profile: {profile}")
    if requested_profile is not None and requested_profile != configured_profile:
        raise ValueError(
            f"requested profile {requested_profile} conflicts with configured profile {configured_profile}"
        )
    chain = EXECUTION_PROFILES[profile]
    if full_chain != EXECUTION_PROFILES["full"]:
        raise ValueError("skills config does not contain the supported full stage chain")
    return supervisor, priors, collector, chain, skills, profile


def required_prior_ids(chain: tuple[str, ...], skills: dict[str, Any]) -> tuple[str, ...]:
    required: list[str] = []
    for stage in chain:
        if stage.startswith("B"):
            required.append(stage)
        elif stage in skills and skills[stage].concurrent_base_prior is not None:
            required.append(skills[stage].concurrent_base_prior)
    return tuple(dict.fromkeys(required))


def dry_run(config_path: Path, requested_profile: str | None = None) -> dict[str, Any]:
    supervisor, priors, _collector, chain, skills, profile = load_all(config_path, requested_profile)
    prior_results = {}
    priors_path = Path(str(supervisor["base_priors"]))
    for prior_id in required_prior_ids(chain, skills):
        entry = priors["priors"][prior_id]
        samples = load_base_prior_stream(
            prior_id,
            entry,
            config_dir=priors_path.parent,
            limits=priors["safety_limits"],
            require_calibrated=False,
            sample_rate_hz=float(priors["sample_rate_hz"]),
        )
        prior_results[prior_id] = {
            "calibrated": bool(samples),
            "sample_count": len(samples),
            "version": entry.get("version"),
        }
    state_machine = StateMachine(chain)
    transitions = []
    while state_machine.current is not None:
        stage = state_machine.current
        if stage in skills:
            skill = skills[stage]
            arm, hand = enforce_skill_executors(
                skill,
                np.ones(14),
                np.ones(12),
                np.zeros(14),
                np.zeros(12),
            )
            transitions.append(
                {
                    "stage": stage,
                    "prompt": skill.prompt,
                    "moving_arm_dimensions": int(np.count_nonzero(arm)),
                    "moving_hand_dimensions": int(np.count_nonzero(hand)),
                    "concurrent_base_prior": skill.concurrent_base_prior,
                }
            )
        else:
            transitions.append({"stage": stage, "vla_enabled": False})
        state_machine.confirm_success(stage)
    return {
        "mode": "dry_run",
        "profile": profile,
        "robot_id": supervisor["robot_id"],
        "state_chain": list(chain),
        "priors": prior_results,
        "transitions": transitions,
        "motion_published": False,
        "ready_for_real_execution": all(value["calibrated"] for value in prior_results.values()),
    }


class SupervisorRuntime:
    def __init__(
        self,
        supervisor: dict[str, Any],
        priors: dict[str, Any],
        collector: dict[str, Any],
        skills: dict[str, Any],
        event_log: EventLog,
        action_log: EventLog | None = None,
    ) -> None:
        self.supervisor = supervisor
        self.priors = priors
        self.collector = collector
        self.skills = skills
        self.event_log = event_log
        self.action_log = action_log
        self.run_id = f"{time.strftime('%Y%m%d_%H%M%S')}_{os.getpid()}"
        self.executed_action_block_index = 0
        self.executed_action_frame_index = 0
        self.control = supervisor["control"]
        self.period = 1.0 / float(self.control["hz"])
        self.arm_limiter = ArmCommandLimiter(supervisor["arm_limits"])
        self.arm_velocity = np.zeros(14)
        self.last_arm_command = np.zeros(14)
        self.last_hand_command = np.zeros(12)
        self.ros: Any | None = None
        self.hands: Any | None = None
        self.cameras: Any | None = None
        self.policy: Any | None = None

    def open(self, *, read_only: bool = False) -> None:
        from supervisor.camera_adapter import LiveCameraAdapter
        from supervisor.hand_adapter import HandHardwareAdapter
        from supervisor.policy_client import PolicyClient
        from supervisor.ros_adapter import RosRobotAdapter

        self.ros = RosRobotAdapter(self.supervisor["ros"], read_only=read_only)
        self.ros.wait_ready()
        self.hands = HandHardwareAdapter(self.supervisor["hands"], self.collector)
        self.cameras = LiveCameraAdapter(Path(str(self.supervisor["collector_config"])))
        policy = self.supervisor["policy"]
        self.policy = PolicyClient(str(policy["host"]), int(policy["port"]))
        expected_protocol = policy.get("protocol_version")
        if expected_protocol and self.policy.metadata.get("protocol_version") != expected_protocol:
            raise RuntimeError(f"policy protocol is incompatible: {self.policy.metadata}")
        robot_config = str(policy.get("robot_config", "svt_cabinet_b0_s1_b1_s2"))
        metadata_robot = self.policy.metadata.get("robot_config")
        if metadata_robot and metadata_robot != robot_config:
            raise RuntimeError(f"policy robot config is incompatible: {self.policy.metadata}")
        expected_contract_id = policy.get("deployment_contract_id")
        if expected_protocol == "svt-policy-v1" and not expected_contract_id:
            raise RuntimeError(
                "svt-policy-v1 requires a pinned deployment contract; no policy "
                "action was published"
            )
        if expected_contract_id:
            contract_expectations = {
                "deployment_contract_id": expected_contract_id,
                "model_path": policy.get("model_path"),
                "normalization_sha256": policy.get("normalization_sha256"),
                "robot_config_sha256": policy.get("robot_config_sha256"),
                "training_config_sha256": policy.get("training_config_sha256"),
            }
            checkpoint_hash_fields = (
                "checkpoint_index_sha256",
                "checkpoint_sha256",
            )
            configured_checkpoint_hashes = [
                name for name in checkpoint_hash_fields if policy.get(name)
            ]
            if len(configured_checkpoint_hashes) != 1:
                raise RuntimeError(
                    "policy config must pin exactly one checkpoint hash field; "
                    "no policy action was published"
                )
            checkpoint_hash_field = configured_checkpoint_hashes[0]
            contract_expectations[checkpoint_hash_field] = policy[checkpoint_hash_field]
            contract_errors = []
            for key, expected in contract_expectations.items():
                if not isinstance(expected, str) or not expected:
                    contract_errors.append(f"missing local expectation {key}")
                elif self.policy.metadata.get(key) != expected:
                    contract_errors.append(
                        f"{key}: expected {expected!r}, "
                        f"got {self.policy.metadata.get(key)!r}"
                    )
            if contract_errors:
                raise RuntimeError(
                    "policy deployment contract is incompatible; no policy action "
                    "was published: " + "; ".join(contract_errors)
                )
        if int(self.policy.metadata.get("chunk_size", -1)) != int(self.control["chunk_size"]):
            raise RuntimeError(f"policy server chunk metadata is incompatible: {self.policy.metadata}")
        expected_actions = {"action.arm.position", "action.hand.position"}
        if set(self.policy.metadata.get("action_keys", ())) != expected_actions:
            raise RuntimeError(f"policy server action metadata is incompatible: {self.policy.metadata}")
        reset_elapsed = self.policy.reset(robot_config)
        if reset_elapsed * 1000.0 > float(self.control["inference_watchdog_ms"]):
            raise RuntimeError(f"policy reset exceeded watchdog: {reset_elapsed * 1000.0:.1f} ms")
        self.last_arm_command = self.ros.arm_state()[0]
        self.last_hand_command = self.hands.latched_action()
        self.event_log.write(
            "runtime_ready",
            read_only=read_only,
            policy_metadata=self.policy.metadata,
            policy_reset_ms=reset_elapsed * 1000.0,
        )
        if self.action_log is not None:
            self.action_log.write(
                "action_run_start",
                run_id=self.run_id,
                read_only=read_only,
                hz=float(self.control["hz"]),
                execute_frames=int(self.control["execute_frames"]),
                policy_metadata=self.policy.metadata,
            )

    def close(self) -> None:
        if self.policy is not None:
            self.policy.close()
        if self.cameras is not None:
            self.cameras.close()
        if self.hands is not None:
            self.hands.close()
        if self.ros is not None:
            self.ros.close()

    def hold(self) -> None:
        assert self.ros is not None and self.hands is not None
        self.ros.publish_base_zero()
        self.ros.publish_arms(self.last_arm_command)
        self.hands.publish(self.last_hand_command)

    def wait_base_zero(self, timeout_sec: float = 5.0) -> None:
        assert self.ros is not None
        if str(self.control.get("base_feedback_mode", "required")) == "observe":
            deadline = time.monotonic() + float(self.control["base_zero_stable_sec"])
            while time.monotonic() < deadline:
                self.hold()
                time.sleep(self.period)
            self.event_log.write(
                "base_zero_hold_complete",
                mode="observe",
                feedback=self.ros.base_feedback_diagnostic(),
            )
            return
        gate = FeedbackZeroGate(
            float(self.control["base_feedback_zero_tolerance"]),
            float(self.control["base_zero_stable_sec"]),
        )
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self.hold()
            if gate.update(*self.ros.base_state()):
                return
            time.sleep(self.period)
        raise SafetyStop("base feedback did not remain zero for 0.5 seconds")

    def run_base_stage(self, stage: str, samples: list[Any], entry: dict[str, Any]) -> None:
        assert self.ros is not None
        self.ros.assert_command_ownership()
        self.last_arm_command = self.ros.arm_state()[0]
        self.last_hand_command = self.hands.latched_action() if self.hands is not None else self.last_hand_command
        started = time.monotonic()
        for sample in samples:
            deadline = started + sample.timestamp_sec
            while time.monotonic() < deadline:
                self.ros.publish_arms(self.last_arm_command)
                if self.hands is not None:
                    self.hands.publish(self.last_hand_command)
                time.sleep(min(self.period, max(0.0, deadline - time.monotonic())))
            self.ros.publish_base(sample.vx, sample.vy, sample.wz)
        self.ros.publish_base_zero()
        self.wait_base_zero()
        self.event_log.write("base_stage_complete", stage=stage, elapsed_sec=time.monotonic() - started)

    def observation(self, prompt: str) -> dict[str, Any]:
        assert self.ros is not None and self.hands is not None and self.cameras is not None
        arm_state = self.ros.arm_state()[0].astype(np.float32)
        hand_state = self.hands.state().astype(np.float32)
        value: dict[str, Any] = self.cameras.observation_images()
        value.update(
            {
                "observation.state.arm.position": arm_state,
                "observation.state.hand.position": hand_state,
                "task": prompt,
            }
        )
        return value

    def _infer_action_chunk(self, stage: str, prompt: str) -> tuple[np.ndarray, np.ndarray]:
        assert self.policy is not None
        response, elapsed = self.policy.infer(
            self.observation(prompt),
            timeout_sec=float(self.control["inference_watchdog_ms"]) / 1000.0,
        )
        if elapsed * 1000.0 > float(self.control["inference_watchdog_ms"]):
            self.hold()
            raise SafetyStop(f"policy response exceeded watchdog: {elapsed * 1000:.1f} ms")
        arm_chunk, hand_chunk = validate_action_chunk(
            response,
            int(self.control["chunk_size"]),
        )
        self.event_log.write(
            "policy_chunk",
            stage=stage,
            round_trip_ms=elapsed * 1000,
            server_timing=response.get("server_timing"),
        )
        return arm_chunk, hand_chunk

    def warm_policy(self) -> None:
        """Compile and validate one action chunk without publishing any motion."""
        assert self.policy is not None
        stage = next((name for name in EXECUTION_PROFILES["b0_s1_b1_s2"] if name in self.skills), None)
        if stage is None:
            raise RuntimeError("no learned skill is available for policy warmup")
        skill = self.skills[stage]
        timeout_sec = float(self.control.get("policy_warmup_timeout_sec", 20.0))
        print(
            f"[POLICY WARMUP] Requesting {stage} without publishing robot commands; "
            f"timeout={timeout_sec:.1f}s."
        )
        response, elapsed = self.policy.infer(
            self.observation(skill.prompt),
            timeout_sec=timeout_sec,
        )
        validate_action_chunk(response, int(self.control["chunk_size"]))
        self.event_log.write(
            "policy_warmup",
            stage=stage,
            round_trip_ms=elapsed * 1000.0,
            server_timing=response.get("server_timing"),
            published_motion=False,
        )
        print(
            f"[POLICY WARMUP READY] {stage} response in {elapsed * 1000.0:.1f} ms; "
            "no robot command was published."
        )

    def _execute_skill_action(
        self,
        stage: str,
        skill: Any,
        arm_target: np.ndarray,
        hand_target: np.ndarray,
        latched_arm: np.ndarray,
        latched_hand: np.ndarray,
        *,
        publish_base_zero: bool,
        action_block_index: int,
        chunk_frame_index: int,
    ) -> None:
        assert self.ros is not None and self.hands is not None
        raw_arm_target = np.asarray(arm_target, dtype=float).copy()
        raw_hand_target = np.asarray(hand_target, dtype=float).copy()
        feedback_before: np.ndarray | None = None
        hand_feedback_before: np.ndarray | None = None
        feedback_before_error: str | None = None
        hand_feedback_before_error: str | None = None
        try:
            feedback_before = self.ros.arm_state(max_age_sec=0.5)[0]
        except Exception as exc:  # noqa: BLE001 - logging must not alter execution
            feedback_before_error = str(exc)
        try:
            hand_feedback_before = self.hands.state(max_age_sec=0.5)
        except Exception as exc:  # noqa: BLE001 - logging must not alter execution
            hand_feedback_before_error = str(exc)
        target_arm, target_hand = enforce_skill_executors(
            skill,
            raw_arm_target,
            raw_hand_target,
            latched_arm,
            latched_hand,
        )
        bounded_arm_target = np.clip(
            target_arm,
            self.arm_limiter.lower,
            self.arm_limiter.upper,
        )
        command, self.arm_velocity, violations = self.arm_limiter.limit(
            target_arm,
            self.last_arm_command,
            self.arm_velocity,
            self.period,
        )
        if violations:
            self.event_log.write("arm_limit", stage=stage, violations=violations)
        self.last_arm_command = command
        self.last_hand_command = target_hand
        if publish_base_zero:
            self.ros.publish_base_zero()
        self.ros.publish_arms(command)
        self.hands.publish(target_hand)
        published_timestamp_ns = time.time_ns()
        published_hand_command = self.hands.latched_action()
        time.sleep(self.period)
        feedback_after: np.ndarray | None = None
        hand_feedback_after: np.ndarray | None = None
        feedback_after_error: str | None = None
        hand_feedback_after_error: str | None = None
        try:
            feedback_after = self.ros.arm_state(max_age_sec=0.5)[0]
        except Exception as exc:  # noqa: BLE001 - logging must not alter execution
            feedback_after_error = str(exc)
        try:
            hand_feedback_after = self.hands.state(max_age_sec=0.5)
        except Exception as exc:  # noqa: BLE001 - logging must not alter execution
            hand_feedback_after_error = str(exc)
        if self.action_log is not None:
            self.action_log.write(
                "executed_action_frame",
                run_id=self.run_id,
                stage=stage,
                action_block_index=action_block_index,
                chunk_frame_index=chunk_frame_index,
                executed_frame_index=self.executed_action_frame_index,
                published_timestamp_ns=published_timestamp_ns,
                model_arm_target=raw_arm_target,
                effective_arm_target=target_arm,
                bounded_arm_target=bounded_arm_target,
                published_arm_command=command,
                arm_feedback_before=feedback_before,
                arm_feedback_after=feedback_after,
                arm_feedback_before_error=feedback_before_error,
                arm_feedback_after_error=feedback_after_error,
                model_hand_target=raw_hand_target,
                effective_hand_target=target_hand,
                published_hand_command=published_hand_command,
                hand_feedback_before=hand_feedback_before,
                hand_feedback_after=hand_feedback_after,
                hand_feedback_before_error=hand_feedback_before_error,
                hand_feedback_after_error=hand_feedback_after_error,
                arm_limit_violations=violations,
            )
        self.executed_action_frame_index += 1

    @staticmethod
    def _operator_answer() -> str | None:
        readable, _, _ = select.select([sys.stdin], [], [], 0)
        return sys.stdin.readline().strip().lower() if readable else None

    def _start_policy_request(
        self,
        executor: ThreadPoolExecutor,
        stage: str,
        prompt: str,
    ) -> Any:
        return executor.submit(self._infer_action_chunk, stage, prompt)

    def _execute_action_block(
        self,
        executor: ThreadPoolExecutor,
        stage: str,
        skill: Any,
        arm_chunk: np.ndarray,
        hand_chunk: np.ndarray,
        latched_arm: np.ndarray,
        latched_hand: np.ndarray,
        *,
        publish_base_zero: bool,
    ) -> tuple[Any, str | None, bool]:
        """Publish one continuous action block while prefetching its successor."""
        execute_frames = int(self.control["execute_frames"])
        prefetch_after = int(self.control["prefetch_after_frames"])
        action_block_index = self.executed_action_block_index
        self.executed_action_block_index += 1
        next_request: Any | None = None
        for index in range(execute_frames):
            self._execute_skill_action(
                stage,
                skill,
                arm_chunk[index],
                hand_chunk[index],
                latched_arm,
                latched_hand,
                publish_base_zero=publish_base_zero,
                action_block_index=action_block_index,
                chunk_frame_index=index,
            )
            if index + 1 == prefetch_after:
                next_request = self._start_policy_request(executor, stage, skill.prompt)
            answer = self._operator_answer()
            if answer is not None:
                return next_request, answer, False
        if next_request is None:
            next_request = self._start_policy_request(executor, stage, skill.prompt)
        return next_request, None, False

    def run_skill_stage(self, stage: str) -> None:
        assert self.ros is not None and self.hands is not None and self.policy is not None
        self.ros.assert_command_ownership()
        skill = self.skills[stage]
        if skill.concurrent_base_prior is not None:
            raise RuntimeError(f"{stage} must run through the combined-skill path")
        self.wait_base_zero()
        latched_arm = self.ros.arm_state()[0]
        latched_hand = self.hands.state()
        started = time.monotonic()
        print(f"{stage} running. Type 's' + Enter when successful, or 'a' + Enter to abort.")
        with ThreadPoolExecutor(max_workers=1, thread_name_prefix="policy-prefetch") as executor:
            request = self._start_policy_request(executor, stage, skill.prompt)
            while (
                skill.max_duration_sec is None
                or time.monotonic() - started < skill.max_duration_sec
            ):
                self.ros.assert_command_ownership()
                self.ros.publish_base_zero()
                arm_chunk, hand_chunk = request.result()
                request, answer, _stopped = self._execute_action_block(
                    executor,
                    stage,
                    skill,
                    arm_chunk,
                    hand_chunk,
                    latched_arm,
                    latched_hand,
                    publish_base_zero=True,
                )
                if answer == "s":
                    self.event_log.write("skill_operator_success", stage=stage)
                    return
                if answer == "a":
                    raise SafetyStop(f"operator aborted {stage}")
        raise SafetyStop(f"{stage} exceeded max_duration_sec={skill.max_duration_sec}")

    def _run_concurrent_b1_prior(
        self,
        samples: list[Any],
        entry: dict[str, Any],
        stop_event: threading.Event,
    ) -> None:
        assert self.ros is not None
        started = time.monotonic()
        published_count = 0
        self.event_log.write("base_stage_start", stage="B1", sample_count=len(samples))
        try:
            for sample in samples:
                deadline = started + sample.timestamp_sec
                while time.monotonic() < deadline:
                    if stop_event.is_set():
                        raise SafetyStop("B1 stopped because the concurrent S1 policy stopped")
                    time.sleep(min(0.01, max(0.0, deadline - time.monotonic())))
                self.ros.publish_base(sample.vx, sample.vy, sample.wz)
                published_count += 1
            self.ros.publish_base_zero()
            self.wait_base_zero()
        except Exception:
            self.ros.publish_base_zero()
            self.event_log.write(
                "base_stage_aborted",
                stage="B1",
                published_sample_count=published_count,
                sample_count=len(samples),
                elapsed_sec=time.monotonic() - started,
            )
            raise
        self.event_log.write(
            "base_stage_complete",
            stage="B1",
            published_sample_count=published_count,
            elapsed_sec=time.monotonic() - started,
        )

    def run_combined_s1_stage(self, samples: list[Any], entry: dict[str, Any]) -> None:
        assert self.ros is not None and self.hands is not None and self.policy is not None
        stage = "S1"
        skill = self.skills[stage]
        if skill.concurrent_base_prior != "B1":
            raise RuntimeError("S1 must declare concurrent_base_prior B1")
        self.ros.assert_command_ownership()
        self.wait_base_zero()
        latched_arm = self.ros.arm_state()[0]
        latched_hand = self.hands.state()
        print(
            "S1 grasp phase running. Type 's' + Enter after the handle is secure; "
            "B1 will then run while S1 policy control continues."
        )
        with ThreadPoolExecutor(max_workers=1, thread_name_prefix="policy-prefetch") as policy_executor:
            request = self._start_policy_request(policy_executor, stage, skill.prompt)
            while True:
                arm_chunk, hand_chunk = request.result()
                request, answer, _stopped = self._execute_action_block(
                    policy_executor,
                    stage,
                    skill,
                    arm_chunk,
                    hand_chunk,
                    latched_arm,
                    latched_hand,
                    publish_base_zero=True,
                )
                if answer == "s":
                    self.event_log.write("s1_grasp_operator_success")
                    break
                if answer == "a":
                    raise SafetyStop("operator aborted S1 grasp phase")
            if request is None:
                request = self._start_policy_request(policy_executor, stage, skill.prompt)
            initial_arm_chunk, initial_hand_chunk = request.result()
            stop_event = threading.Event()
            with ThreadPoolExecutor(max_workers=1, thread_name_prefix="b1-prior") as executor:
                base_future = executor.submit(
                    self._run_concurrent_b1_prior,
                    samples,
                    entry,
                    stop_event,
                )
                arm_chunk = initial_arm_chunk
                hand_chunk = initial_hand_chunk
                try:
                    while not base_future.done():
                        request, answer, _stopped = self._execute_action_block(
                            policy_executor,
                            stage,
                            skill,
                            arm_chunk,
                            hand_chunk,
                            latched_arm,
                            latched_hand,
                            publish_base_zero=False,
                        )
                        if base_future.done():
                            break
                        if answer == "a":
                            raise SafetyStop("operator aborted S1 during B1")
                        arm_chunk, hand_chunk = request.result()
                    base_future.result()
                finally:
                    active_error = sys.exc_info()[0] is not None
                    stop_event.set()
                    if not base_future.done():
                        self.ros.publish_base_zero()
                    try:
                        base_future.result(timeout=2.0)
                    except Exception:
                        if not active_error:
                            raise
        self.event_log.write("combined_skill_complete", stage="S1", concurrent_prior="B1")

    def manual_hold(self, reason: str) -> None:
        self.event_log.write("manual_recovery_hold", reason=reason)
        print(f"Manual recovery hold: {reason}. Base is zero; arms/hands remain latched. Ctrl-C exits.")
        while True:
            self.hold()
            time.sleep(self.period)


def _shadow_chunk_summary(
    supervisor: dict[str, Any],
    skill: Any,
    arm_chunk: np.ndarray,
    hand_chunk: np.ndarray,
    latched_arm: np.ndarray,
    latched_hand: np.ndarray,
) -> dict[str, Any]:
    """Evaluate a returned chunk without applying a command limiter or publishing it."""
    effective_arm = np.empty_like(arm_chunk, dtype=float)
    effective_hand = np.empty_like(hand_chunk, dtype=float)
    for index in range(arm_chunk.shape[0]):
        effective_arm[index], effective_hand[index] = enforce_skill_executors(
            skill,
            arm_chunk[index],
            hand_chunk[index],
            latched_arm,
            latched_hand,
        )
    arm_limits = supervisor["arm_limits"]
    lower = np.asarray(arm_limits["lower"], dtype=float)
    upper = np.asarray(arm_limits["upper"], dtype=float)
    o6_min = np.asarray(supervisor["hands"]["left_o6_min"], dtype=float)
    o6_max = np.asarray(supervisor["hands"]["left_o6_max"], dtype=float)
    return {
        "arm_min_rad": float(np.min(effective_arm)),
        "arm_max_rad": float(np.max(effective_arm)),
        "arm_hard_limit_values": int(
            np.count_nonzero((effective_arm < lower) | (effective_arm > upper))
        ),
        "left_o6_min": float(np.min(effective_hand[:, :6])),
        "left_o6_max": float(np.max(effective_hand[:, :6])),
        "left_o6_out_of_range_values": int(
            np.count_nonzero(
                (effective_hand[:, :6] < o6_min) | (effective_hand[:, :6] > o6_max)
            )
        ),
    }


def shadow(args: argparse.Namespace) -> int:
    if args.shadow_rounds < 1:
        raise ValueError("--shadow-rounds must be at least 1")
    config_path = args.config.expanduser().resolve()
    supervisor, priors, collector, chain, skills, profile = load_all(config_path, args.profile)
    event_log = EventLog(args.event_log.expanduser().resolve())
    action_log_path = (
        args.action_log.expanduser().resolve()
        if args.action_log is not None
        else Path("/svtrobo_data/inference_logs")
        / f"long_task_actions_{time.strftime('%Y%m%d_%H%M%S')}.jsonl"
    )
    action_log = EventLog(action_log_path)
    print(f"Per-frame action log: {action_log_path}")
    event_log.write("action_log_opened", path=str(action_log_path))
    runtime = SupervisorRuntime(
        supervisor,
        priors,
        collector,
        skills,
        event_log,
        action_log=action_log,
    )
    watchdog_ms = float(supervisor["control"]["inference_watchdog_ms"])
    learned_stages = [stage for stage in chain if stage in skills]
    report: dict[str, Any] = {
        "mode": "shadow",
        "profile": profile,
        "policy_watchdog_ms": watchdog_ms,
        "rounds_per_skill": args.shadow_rounds,
        "motion_published": False,
        "stages": {},
    }
    try:
        # read_only avoids both command publisher creation and close-time zero commands.
        runtime.open(read_only=True)
        for stage in learned_stages:
            skill = skills[stage]
            latched_arm = runtime.ros.arm_state()[0]
            latched_hand = runtime.hands.state()
            timings_ms: list[float] = []
            arm_limit_values = 0
            o6_out_of_range_values = 0
            arm_min_rad = float("inf")
            arm_max_rad = float("-inf")
            o6_min = float("inf")
            o6_max = float("-inf")
            for round_index in range(args.shadow_rounds):
                response, elapsed = runtime.policy.infer(
                    runtime.observation(skill.prompt),
                    timeout_sec=watchdog_ms / 1000.0,
                )
                elapsed_ms = elapsed * 1000.0
                if elapsed_ms > watchdog_ms:
                    raise SafetyStop(
                        f"shadow {stage} response exceeded watchdog: {elapsed_ms:.1f} ms"
                    )
                arm_chunk, hand_chunk = validate_action_chunk(
                    response,
                    int(supervisor["control"]["chunk_size"]),
                )
                summary = _shadow_chunk_summary(
                    supervisor,
                    skill,
                    arm_chunk,
                    hand_chunk,
                    latched_arm,
                    latched_hand,
                )
                timings_ms.append(elapsed_ms)
                arm_limit_values += summary["arm_hard_limit_values"]
                o6_out_of_range_values += summary["left_o6_out_of_range_values"]
                arm_min_rad = min(arm_min_rad, summary["arm_min_rad"])
                arm_max_rad = max(arm_max_rad, summary["arm_max_rad"])
                o6_min = min(o6_min, summary["left_o6_min"])
                o6_max = max(o6_max, summary["left_o6_max"])
                event_log.write(
                    "shadow_policy_chunk",
                    stage=stage,
                    round_index=round_index,
                    round_trip_ms=elapsed_ms,
                    server_timing=response.get("server_timing"),
                    **summary,
                )
            report["stages"][stage] = {
                "requests": len(timings_ms),
                "latency_ms": {
                    "p50": float(np.percentile(timings_ms, 50)),
                    "p95": float(np.percentile(timings_ms, 95)),
                    "max": float(np.max(timings_ms)),
                },
                "arm_range_rad": [arm_min_rad, arm_max_rad],
                "arm_hard_limit_values": arm_limit_values,
                "left_o6_range": [o6_min, o6_max],
                "left_o6_out_of_range_values": o6_out_of_range_values,
            }
        print(json.dumps(report, indent=2, ensure_ascii=False))
        return 0
    finally:
        runtime.close()
        event_log.close()


def execute(args: argparse.Namespace) -> int:
    config_path = args.config.expanduser().resolve()
    supervisor, priors, collector, chain, skills, profile = load_all(config_path, args.profile)
    priors_path = Path(str(supervisor["base_priors"]))
    prior_streams = {
        prior_id: load_base_prior_stream(
            prior_id,
            priors["priors"][prior_id],
            config_dir=priors_path.parent,
            limits=priors["safety_limits"],
            require_calibrated=True,
            sample_rate_hz=float(priors["sample_rate_hz"]),
        )
        for prior_id in required_prior_ids(chain, skills)
    }

    if not Path("/home/svt/glove_control").is_dir():
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "glove_control"))
    sys.path.insert(0, "/home/svt/glove_control")
    from hand_command_arbiter import HandCommandLease

    event_log = EventLog(args.event_log.expanduser().resolve())
    runtime = SupervisorRuntime(supervisor, priors, collector, skills, event_log)
    machine = StateMachine(chain)
    shutdown_handlers = install_shutdown_handlers()
    takeover: Any | None = None
    if supervisor.get("base_publisher_takeover", {}).get("enabled") is True:
        from supervisor.paired_collection_controller import ManagedServiceTakeover

        takeover = ManagedServiceTakeover(supervisor)
    with ExitStack() as stack:
        stack.enter_context(
            HandCommandLease(
                supervisor["control"]["command_owner_lock"],
                mode="long_task_supervisor",
                owner="base_arm_hand_supervisor",
            )
        )
        stack.enter_context(
            HandCommandLease(
                supervisor["control"]["hand_owner_lock"],
                mode="vla_and_latched_priors",
                owner="long_task_supervisor",
            )
        )
        try:
            if takeover is not None:
                takeover.acquire()
                event_log.write("base_publisher_takeover_acquired", profile=profile)
            runtime.open()
            runtime.warm_policy()
            while machine.current is not None:
                stage = machine.current
                answer = input(f"Type RUN {stage} to start, or ABORT: ").strip()
                if answer == "ABORT":
                    raise SafetyStop(f"operator aborted before {stage}")
                if answer != f"RUN {stage}":
                    print("Stage not started.")
                    continue
                event_log.write("stage_start", stage=stage)
                if stage == "S1":
                    runtime.run_combined_s1_stage(
                        prior_streams["B1"], priors["priors"]["B1"]
                    )
                elif stage.startswith("B"):
                    runtime.run_base_stage(stage, prior_streams[stage], priors["priors"][stage])
                else:
                    runtime.run_skill_stage(stage)
                machine.confirm_success(stage)
                event_log.write("stage_complete", stage=stage, next_stage=machine.current)
            runtime.manual_hold("task complete; preserving final arm/hand state")
        except KeyboardInterrupt:
            event_log.write("operator_exit")
        except Exception as exc:
            event_log.write("safety_stop", error=f"{type(exc).__name__}: {exc}")
            if runtime.ros is not None and runtime.hands is not None:
                try:
                    runtime.manual_hold(f"{type(exc).__name__}: {exc}")
                except KeyboardInterrupt:
                    pass
            else:
                print(f"Startup blocked before command interfaces were ready: {exc}", file=sys.stderr)
        finally:
            # A failure while closing a camera, ROS, or hand adapter must never
            # prevent F710 from being restored after this process releases base
            # command ownership.
            # The first SIGINT/SIGTERM/SIGHUP has already selected this cleanup
            # path. Ignore any follow-up signal until the base publisher is
            # restored, so terminal closure cannot interrupt the restore call.
            ignore_shutdown_signals(shutdown_handlers)
            try:
                runtime.close()
            except Exception as exc:  # noqa: BLE001
                event_log.write("runtime_close_failed", error=str(exc))
                print(f"WARNING: runtime cleanup failed: {exc}", file=sys.stderr)
            try:
                if takeover is not None:
                    takeover.restore(force_enable=True)
                    event_log.write(
                        "base_publisher_takeover_restored", f710_enabled=True
                    )
            except Exception as exc:  # noqa: BLE001
                event_log.write("base_publisher_takeover_restore_failed", error=str(exc))
                print(f"WARNING: failed to restore F710 publisher: {exc}", file=sys.stderr)
            finally:
                restore_shutdown_handlers(shutdown_handlers)
                action_log.write("action_run_end", run_id=runtime.run_id)
                action_log.close()
                event_log.close()
    return 0


def main() -> int:
    args = parse_args()
    if args.shadow:
        return shadow(args)
    if not args.execute:
        print(
            json.dumps(
                dry_run(args.config.expanduser().resolve(), args.profile),
                indent=2,
                ensure_ascii=False,
            )
        )
        return 0
    raw_config = load_yaml(args.config.expanduser().resolve())
    acknowledgement = str(raw_config.get("motion_acknowledgement", "SVT_LONG_TASK_V1"))
    if args.acknowledge_motion != acknowledgement:
        raise SystemExit(f"real execution blocked: pass --acknowledge-motion {acknowledgement}")
    return execute(args)


if __name__ == "__main__":
    raise SystemExit(main())
