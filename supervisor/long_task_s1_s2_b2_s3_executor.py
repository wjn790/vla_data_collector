#!/usr/bin/env python3
"""Dedicated B0/S1(+B1)/S2/B2/S3 executor for the three-skill policy."""

from __future__ import annotations

import argparse
from dataclasses import replace
from contextlib import ExitStack
import json
import math
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np


COLLECTOR_ROOT = Path(__file__).resolve().parents[1]
if str(COLLECTOR_ROOT) not in sys.path:
    sys.path.insert(0, str(COLLECTOR_ROOT))

from supervisor import long_task_supervisor as base
from supervisor.core import (
    SafetyStop,
    enforce_skill_executors,
    finite_array,
    load_base_prior_stream,
    load_skills,
    load_yaml,
    validate_action_chunk,
)


PROFILE = "b0_s1_b1_s2_b2_s3"
CHAIN = ("B0", "S1", "S2", "B2", "S3")
POLICY_ROBOT_CONFIGS = {
    "svt_cabinet_s1_s2_s3",
    "svt_cabinet_s1_s2_s3_continuation",
}


class StageMachine:
    def __init__(self) -> None:
        self.index = 0

    @property
    def current(self) -> str | None:
        return None if self.index >= len(CHAIN) else CHAIN[self.index]

    def confirm_success(self, stage: str) -> None:
        if stage != self.current:
            raise RuntimeError(f"cannot confirm {stage}; current stage is {self.current}")
        self.index += 1


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=COLLECTOR_ROOT / "config" / "inference_b0_s1_b1_s2_b2_s3.yaml",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--execute", action="store_true")
    mode.add_argument(
        "--shadow",
        action="store_true",
        help="Query S1/S2/S3 without creating ROS command publishers.",
    )
    parser.add_argument("--shadow-rounds", type=int, default=10)
    parser.add_argument("--acknowledge-motion")
    parser.add_argument(
        "--event-log",
        type=Path,
        default=COLLECTOR_ROOT / "logs" / "long_task_s1_s2_b2_s3.jsonl",
    )
    parser.add_argument("--action-log", type=Path)
    return parser.parse_args(argv)


def _config_relative(config_path: Path, value: Any) -> Path:
    path = Path(str(value)).expanduser()
    return path if path.is_absolute() else (config_path.parent / path).resolve()


def _apply_skill_overrides(
    supervisor: dict[str, Any],
    skills: dict[str, Any],
) -> dict[str, Any]:
    overrides = supervisor.get("skill_overrides", {})
    if not isinstance(overrides, dict) or not set(overrides) <= {"S2", "S3"}:
        raise ValueError("skill_overrides may only contain S2 and S3")
    result = dict(skills)
    for stage, values in overrides.items():
        if not isinstance(values, dict) or set(values) != {"max_duration_sec"}:
            raise ValueError(f"{stage} override must contain only max_duration_sec")
        raw_duration = values["max_duration_sec"]
        if raw_duration is None:
            duration = None
        else:
            duration = float(raw_duration)
            if not math.isfinite(duration) or duration <= 0:
                raise ValueError(f"{stage} max_duration_sec override must be positive or null")
        result[stage] = replace(result[stage], max_duration_sec=duration)
    return result


def load_all(
    config_path: Path,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any], dict[str, Any]]:
    supervisor = load_yaml(config_path)
    if supervisor.get("execution_profile") != PROFILE:
        raise ValueError(f"execution_profile must be {PROFILE}")
    for key in ("skills", "base_priors", "collector_config"):
        supervisor[key] = str(_config_relative(config_path, supervisor[key]))

    full_chain, loaded_skills = load_skills(Path(supervisor["skills"]))
    if tuple(full_chain[:5]) != CHAIN:
        raise ValueError(f"skills chain must start with {CHAIN}")
    skills = _apply_skill_overrides(supervisor, loaded_skills)
    priors = load_yaml(Path(supervisor["base_priors"]))
    collector = load_yaml(Path(supervisor["collector_config"]))
    if not (
        supervisor["robot_id"] == priors["robot_id"] == collector["robot_id"]
    ):
        raise ValueError("robot_id differs across executor, prior, and collector configs")

    control = supervisor.get("control")
    if not isinstance(control, dict):
        raise ValueError("control must be a mapping")
    if int(control["hz"]) != 15 or int(control["chunk_size"]) != 50:
        raise ValueError("executor requires 15 Hz and chunk_size=50")
    execute_frames = int(control["execute_frames"])
    if not 1 <= execute_frames <= int(control["chunk_size"]):
        raise ValueError("execute_frames is outside the policy chunk")
    if control.get("inference_mode") != "synchronous":
        raise ValueError("inference_mode must be synchronous")
    if "prefetch_after_frames" in control:
        raise ValueError("prefetch_after_frames must be omitted in synchronous mode")
    watchdog_ms = int(control["inference_watchdog_ms"])
    if not 500 <= watchdog_ms <= 5000:
        raise ValueError("inference_watchdog_ms must be between 500 and 5000")
    if not 20 <= float(control["policy_warmup_timeout_sec"]) <= 300:
        raise ValueError("policy_warmup_timeout_sec must be between 20 and 300")
    if not 1 <= float(control["stage_reset_timeout_sec"]) <= 30:
        raise ValueError("stage_reset_timeout_sec must be between 1 and 30")

    policy = supervisor.get("policy")
    if not isinstance(policy, dict) or policy.get("robot_config") not in POLICY_ROBOT_CONFIGS:
        raise ValueError(
            f"policy.robot_config must be one of {sorted(POLICY_ROBOT_CONFIGS)}"
        )
    required_contract_fields = {
        "deployment_contract_id",
        "model_path",
        "normalization_sha256",
        "robot_config_sha256",
        "training_config_sha256",
    }
    missing_contract_fields = sorted(required_contract_fields - set(policy))
    if missing_contract_fields:
        raise ValueError(
            f"policy deployment contract fields are missing: {missing_contract_fields}"
        )
    if policy.get("protocol_version") == "svt-pi05-policy-v1":
        if policy.get("policy_type") != "pi05":
            raise ValueError("svt-pi05-policy-v1 requires policy_type: pi05")
        if not str(policy.get("model_path", "")).startswith("/"):
            raise ValueError("PI0.5 model_path must be an absolute WJN path")
    checkpoint_hash_fields = (
        "checkpoint_index_sha256",
        "checkpoint_sha256",
    )
    configured_checkpoint_hashes = [
        name for name in checkpoint_hash_fields if policy.get(name)
    ]
    if len(configured_checkpoint_hashes) != 1:
        raise ValueError(
            "policy must define exactly one of checkpoint_index_sha256 or "
            "checkpoint_sha256"
        )
    for hash_field in (
        "normalization_sha256",
        "robot_config_sha256",
        "training_config_sha256",
        configured_checkpoint_hashes[0],
    ):
        value = policy[hash_field]
        if not isinstance(value, str) or len(value) != 64:
            raise ValueError(f"policy.{hash_field} must be a SHA-256 digest")
    preset = supervisor.get("s3_preset")
    if not isinstance(preset, dict) or preset.get("enabled") is not True:
        raise ValueError("s3_preset must be enabled")
    finite_array(preset["left_arm_position"], (7,), "S3 left-arm preset")
    finite_array(preset["left_hand_position"], (6,), "S3 O6 preset")
    if float(preset["max_velocity_radps"]) <= 0:
        raise ValueError("S3 preset max_velocity_radps must be positive")
    if float(preset["max_hand_velocity_units_per_sec"]) <= 0:
        raise ValueError("S3 preset max_hand_velocity_units_per_sec must be positive")
    if not 0 < float(preset["min_duration_sec"]) <= float(preset["max_duration_sec"]):
        raise ValueError("invalid S3 preset duration bounds")
    return supervisor, priors, collector, skills


def required_prior_ids(skills: dict[str, Any]) -> tuple[str, ...]:
    return base.required_prior_ids(CHAIN, skills)


def smooth_transition_commands(
    start: Any,
    target: Any,
    *,
    hz: float,
    max_velocity_radps: float,
    min_duration_sec: float,
    max_duration_sec: float,
) -> np.ndarray:
    start_array = finite_array(start, (14,), "transition start")
    target_array = finite_array(target, (14,), "transition target")
    if hz <= 0 or max_velocity_radps <= 0:
        raise ValueError("transition rate and velocity must be positive")
    max_delta = float(np.max(np.abs(target_array - start_array)))
    # Quintic smootherstep has a maximum derivative of 1.875.
    duration = max(float(min_duration_sec), 1.875 * max_delta / max_velocity_radps)
    if duration > float(max_duration_sec):
        raise SafetyStop(
            f"S3 preset requires {duration:.2f}s, above max_duration_sec={max_duration_sec}"
        )
    intervals = max(1, int(math.ceil(duration * hz)))
    phase = np.linspace(0.0, 1.0, intervals + 1)
    blend = 10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5
    commands = start_array + blend[:, None] * (target_array - start_array)
    observed_velocity = float(np.max(np.abs(np.diff(commands, axis=0))) * hz)
    if observed_velocity > max_velocity_radps + 1.0e-9:
        raise RuntimeError("generated S3 transition exceeds its velocity bound")
    return commands


def smooth_arm_hand_transition_commands(
    start_arm: Any,
    target_arm: Any,
    start_hand: Any,
    target_hand: Any,
    *,
    hz: float,
    max_arm_velocity_radps: float,
    max_hand_velocity_units_per_sec: float,
    min_duration_sec: float,
    max_duration_sec: float,
) -> tuple[np.ndarray, np.ndarray]:
    start_arm_array = finite_array(start_arm, (14,), "transition arm start")
    target_arm_array = finite_array(target_arm, (14,), "transition arm target")
    start_hand_array = finite_array(start_hand, (12,), "transition hand start")
    target_hand_array = finite_array(target_hand, (12,), "transition hand target")
    if hz <= 0 or max_arm_velocity_radps <= 0 or max_hand_velocity_units_per_sec <= 0:
        raise ValueError("transition rates must be positive")
    arm_delta = float(np.max(np.abs(target_arm_array - start_arm_array)))
    hand_delta = float(np.max(np.abs(target_hand_array - start_hand_array)))
    duration = max(
        float(min_duration_sec),
        1.875 * arm_delta / max_arm_velocity_radps,
        1.875 * hand_delta / max_hand_velocity_units_per_sec,
    )
    if duration > float(max_duration_sec):
        raise SafetyStop(
            f"S3 preset requires {duration:.2f}s, above max_duration_sec={max_duration_sec}"
        )
    intervals = max(1, int(math.ceil(duration * hz)))
    phase = np.linspace(0.0, 1.0, intervals + 1)
    blend = 10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5
    arm_commands = start_arm_array + blend[:, None] * (
        target_arm_array - start_arm_array
    )
    hand_commands = start_hand_array + blend[:, None] * (
        target_hand_array - start_hand_array
    )
    observed_arm_velocity = float(
        np.max(np.abs(np.diff(arm_commands, axis=0))) * hz
    )
    observed_hand_velocity = float(
        np.max(np.abs(np.diff(hand_commands, axis=0))) * hz
    )
    if observed_arm_velocity > max_arm_velocity_radps + 1.0e-9:
        raise RuntimeError("generated S3 transition exceeds its arm velocity bound")
    if observed_hand_velocity > max_hand_velocity_units_per_sec + 1.0e-9:
        raise RuntimeError("generated S3 transition exceeds its hand velocity bound")
    return arm_commands, hand_commands


class ExtendedRuntime(base.SupervisorRuntime):
    def _execute_action_block(
        self,
        *args: Any,
        publish_base_zero: bool,
    ) -> Any:
        """Execute a complete block before capturing the next observation.

        Current supervisors call this method synchronously with six positional
        arguments. Older deployments also pass a policy executor and expect a
        future in the return value. Supporting both contracts keeps this
        dedicated executor compatible without changing the S1-S2 supervisor.
        """
        if len(args) == 6:
            executor = None
            stage, skill, arm_chunk, hand_chunk, latched_arm, latched_hand = args
            legacy_contract = False
        elif len(args) == 7:
            (
                executor,
                stage,
                skill,
                arm_chunk,
                hand_chunk,
                latched_arm,
                latched_hand,
            ) = args
            legacy_contract = True
        else:
            raise TypeError(
                "_execute_action_block expects the synchronous six-argument "
                "contract or the legacy seven-argument contract"
            )

        execute_frames = int(self.control["execute_frames"])
        action_block_index = self.executed_action_block_index
        self.executed_action_block_index += 1
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
            answer = self._operator_answer()
            if answer is not None:
                if legacy_contract:
                    return None, answer, False
                return answer

        # The caller captures the next observation only after this returns.
        # Re-publish the final targets once so they remain explicitly latched
        # while the synchronous policy request is running.
        self.hold()
        self.event_log.write(
            "synchronous_action_block_complete",
            stage=stage,
            completed_action_block_index=action_block_index,
            observation_after_executed_frames=execute_frames,
        )
        if not legacy_contract:
            return None

        next_request = self._start_policy_request(executor, stage, skill.prompt)
        return next_request, None, False

    def transition_to_s3_preset(self) -> None:
        assert self.ros is not None and self.hands is not None
        self.ros.assert_command_ownership()
        self.wait_base_zero()
        preset = self.supervisor["s3_preset"]
        start_arm = self.ros.arm_state(max_age_sec=0.5)[0]
        start_hand = self.hands.state(max_age_sec=0.5)
        target_arm = start_arm.copy()
        target_hand = start_hand.copy()
        target_arm[:7] = finite_array(
            preset["left_arm_position"], (7,), "S3 left-arm preset"
        )
        target_hand[:6] = finite_array(
            preset["left_hand_position"], (6,), "S3 O6 preset"
        )
        if np.any(target_arm < self.arm_limiter.lower) or np.any(
            target_arm > self.arm_limiter.upper
        ):
            raise SafetyStop("S3 preset lies outside configured arm limits")
        o6_min = finite_array(self.supervisor["hands"]["left_o6_min"], (6,), "O6 min")
        o6_max = finite_array(self.supervisor["hands"]["left_o6_max"], (6,), "O6 max")
        if np.any(target_hand[:6] < o6_min) or np.any(target_hand[:6] > o6_max):
            raise SafetyStop("S3 O6 preset lies outside configured hand limits")

        arm_commands, hand_commands = smooth_arm_hand_transition_commands(
            start_arm,
            target_arm,
            start_hand,
            target_hand,
            hz=float(self.control["hz"]),
            max_arm_velocity_radps=float(preset["max_velocity_radps"]),
            max_hand_velocity_units_per_sec=float(
                preset["max_hand_velocity_units_per_sec"]
            ),
            min_duration_sec=float(preset["min_duration_sec"]),
            max_duration_sec=float(preset["max_duration_sec"]),
        )
        print(
            f"[S3 PRESET] Moving the left arm/O6 to the S3 training start state "
            f"over {(len(arm_commands) - 1) * self.period:.2f}s."
        )
        self.event_log.write(
            "s3_preset_start",
            source=preset.get("source"),
            frame_count=len(arm_commands),
            start_arm=start_arm,
            target_arm=target_arm,
            start_hand=start_hand,
            target_hand=target_hand,
        )
        self.last_arm_command = start_arm.copy()
        self.last_hand_command = start_hand.copy()
        self.arm_velocity = np.zeros(14)
        for frame_index, (arm_command, hand_command) in enumerate(
            zip(arm_commands, hand_commands)
        ):
            self.ros.assert_command_ownership()
            self.ros.publish_base_zero()
            self.ros.publish_arms(arm_command)
            self.hands.publish(hand_command)
            self.last_arm_command = arm_command.copy()
            self.last_hand_command = hand_command.copy()
            if self.action_log is not None:
                self.action_log.write(
                    "s3_preset_frame",
                    run_id=self.run_id,
                    frame_index=frame_index,
                    frame_count=len(arm_commands),
                    published_arm_command=arm_command,
                    published_hand_command=self.hands.latched_action(),
                )
            time.sleep(self.period)

        settle_deadline = time.monotonic() + float(preset["settle_sec"])
        while time.monotonic() < settle_deadline:
            self.hold()
            time.sleep(self.period)
        arm_feedback = self.ros.arm_state(max_age_sec=0.5)[0]
        hand_feedback = self.hands.state(max_age_sec=0.5)
        arm_error = float(np.max(np.abs(arm_feedback[:7] - target_arm[:7])))
        hand_error = float(np.max(np.abs(hand_feedback[:6] - target_hand[:6])))
        if arm_error > float(preset["arm_tolerance_rad"]):
            raise SafetyStop(
                f"S3 preset left-arm error {arm_error:.4f} rad exceeds tolerance"
            )
        if hand_error > float(preset["hand_tolerance"]):
            raise SafetyStop(f"S3 preset O6 error {hand_error:.1f} exceeds tolerance")
        self.arm_velocity = np.zeros(14)
        self.event_log.write(
            "s3_preset_complete",
            arm_error_rad=arm_error,
            hand_error=hand_error,
            arm_feedback=arm_feedback,
            hand_feedback=hand_feedback,
        )
        print(
            f"[S3 PRESET READY] left-arm error={arm_error:.4f} rad, "
            f"O6 error={hand_error:.1f}."
        )

    def prepare_s3_policy(self) -> None:
        assert self.policy is not None and self.ros is not None
        self.wait_base_zero()
        reset_timeout = float(self.control["stage_reset_timeout_sec"])
        elapsed = self.policy.reset(
            str(self.supervisor["policy"]["robot_config"]),
            timeout_sec=reset_timeout,
        )
        self.event_log.write("policy_stage_reset", stage="S3", elapsed_ms=elapsed * 1000.0)
        skill = self.skills["S3"]
        warmup_timeout = float(self.control["policy_warmup_timeout_sec"])
        print(
            f"[S3 POLICY WARMUP] Querying S3 without publishing motion; "
            f"timeout={warmup_timeout:.1f}s."
        )
        repeats = int(self.control.get("policy_warmup_repeats", 2))
        if repeats < 1:
            raise ValueError("policy_warmup_repeats must be at least 1")
        for warmup_index in range(repeats):
            response, elapsed = self.policy.infer(
                self.observation(skill.prompt), timeout_sec=warmup_timeout
            )
            validate_action_chunk(response, int(self.control["chunk_size"]))
            self.event_log.write(
                "policy_warmup",
                stage="S3",
                warmup_index=warmup_index,
                round_trip_ms=elapsed * 1000.0,
                server_timing=response.get("server_timing"),
                published_motion=False,
            )
        self.hold()
        print(
            f"[S3 POLICY READY] warmup {repeats}/{repeats} response="
            f"{elapsed * 1000.0:.1f} ms; "
            "no warmup action was published."
        )


def _prior_dry_run(
    supervisor: dict[str, Any],
    priors: dict[str, Any],
    skills: dict[str, Any],
) -> dict[str, Any]:
    results: dict[str, Any] = {}
    priors_path = Path(supervisor["base_priors"])
    for prior_id in required_prior_ids(skills):
        entry = priors["priors"][prior_id]
        try:
            samples = load_base_prior_stream(
                prior_id,
                entry,
                config_dir=priors_path.parent,
                limits=priors["safety_limits"],
                require_calibrated=False,
                sample_rate_hz=float(priors["sample_rate_hz"]),
            )
            error = None
        except FileNotFoundError as exc:
            samples = []
            error = f"stream is only available on SVT: {exc}"
        results[prior_id] = {
            "declared_calibrated": entry.get("calibrated") is True,
            "sample_count": len(samples),
            "error": error,
        }
    return results


def dry_run(config_path: Path) -> dict[str, Any]:
    supervisor, priors, _collector, skills = load_all(config_path)
    prior_results = _prior_dry_run(supervisor, priors, skills)
    return {
        "mode": "dry_run",
        "profile": PROFILE,
        "state_chain": list(CHAIN),
        "policy_robot_config": supervisor["policy"]["robot_config"],
        "priors": prior_results,
        "s3_preset": supervisor["s3_preset"],
        "motion_published": False,
        "ready_for_real_execution": all(
            value["sample_count"] > 0 for value in prior_results.values()
        ),
    }


def _action_log_path(args: argparse.Namespace) -> Path:
    if args.action_log is not None:
        return args.action_log.expanduser().resolve()
    return Path("/svtrobo_data/inference_logs") / (
        f"long_task_s1_s2_b2_s3_actions_{time.strftime('%Y%m%d_%H%M%S')}.jsonl"
    )


def shadow(args: argparse.Namespace) -> int:
    if args.shadow_rounds < 1:
        raise ValueError("--shadow-rounds must be at least 1")
    supervisor, priors, collector, skills = load_all(args.config.expanduser().resolve())
    event_log = base.EventLog(args.event_log.expanduser().resolve())
    action_log_path = _action_log_path(args)
    action_log = base.EventLog(action_log_path)
    runtime = ExtendedRuntime(
        supervisor, priors, collector, skills, event_log, action_log=action_log
    )
    report: dict[str, Any] = {"mode": "shadow", "motion_published": False, "stages": {}}
    try:
        runtime.open(read_only=True)
        watchdog_sec = float(supervisor["control"]["inference_watchdog_ms"]) / 1000.0
        reset_timeout = float(supervisor["control"]["stage_reset_timeout_sec"])
        warmup_timeout = float(supervisor["control"]["policy_warmup_timeout_sec"])
        for stage in ("S1", "S2", "S3"):
            skill = skills[stage]
            latched_arm = runtime.ros.arm_state()[0]
            latched_hand = runtime.hands.state()
            reset_elapsed = runtime.policy.reset(
                str(supervisor["policy"]["robot_config"]), timeout_sec=reset_timeout
            )
            warmup_response, warmup_elapsed = runtime.policy.infer(
                runtime.observation(skill.prompt), timeout_sec=warmup_timeout
            )
            validate_action_chunk(
                warmup_response, int(supervisor["control"]["chunk_size"])
            )
            event_log.write(
                "shadow_policy_warmup",
                stage=stage,
                reset_ms=reset_elapsed * 1000.0,
                round_trip_ms=warmup_elapsed * 1000.0,
                published_motion=False,
            )
            timings: list[float] = []
            hard_limit_values = 0
            o6_out_of_range_values = 0
            for round_index in range(args.shadow_rounds):
                response, elapsed = runtime.policy.infer(
                    runtime.observation(skill.prompt), timeout_sec=watchdog_sec
                )
                arm_chunk, hand_chunk = validate_action_chunk(
                    response, int(supervisor["control"]["chunk_size"])
                )
                summary = base._shadow_chunk_summary(
                    supervisor,
                    skill,
                    arm_chunk,
                    hand_chunk,
                    latched_arm,
                    latched_hand,
                )
                timings.append(elapsed * 1000.0)
                hard_limit_values += int(summary["arm_hard_limit_values"])
                o6_out_of_range_values += int(summary["left_o6_out_of_range_values"])
                event_log.write(
                    "shadow_policy_chunk",
                    stage=stage,
                    round_index=round_index,
                    round_trip_ms=elapsed * 1000.0,
                    **summary,
                )
            report["stages"][stage] = {
                "requests": len(timings),
                "latency_ms": {
                    "p50": float(np.percentile(timings, 50)),
                    "p95": float(np.percentile(timings, 95)),
                    "max": float(np.max(timings)),
                },
                "arm_hard_limit_values": hard_limit_values,
                "left_o6_out_of_range_values": o6_out_of_range_values,
            }
        print(json.dumps(report, indent=2, ensure_ascii=False))
        return 0
    finally:
        runtime.close()
        action_log.close()
        event_log.close()


def execute(args: argparse.Namespace) -> int:
    config_path = args.config.expanduser().resolve()
    supervisor, priors, collector, skills = load_all(config_path)
    priors_path = Path(supervisor["base_priors"])
    prior_streams = {
        prior_id: load_base_prior_stream(
            prior_id,
            priors["priors"][prior_id],
            config_dir=priors_path.parent,
            limits=priors["safety_limits"],
            require_calibrated=True,
            sample_rate_hz=float(priors["sample_rate_hz"]),
        )
        for prior_id in required_prior_ids(skills)
    }

    if not Path("/home/svt/glove_control").is_dir():
        sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "glove_control"))
    sys.path.insert(0, "/home/svt/glove_control")
    from hand_command_arbiter import HandCommandLease

    event_log = base.EventLog(args.event_log.expanduser().resolve())
    action_log_path = _action_log_path(args)
    action_log = base.EventLog(action_log_path)
    print(f"Per-frame action log: {action_log_path}")
    event_log.write("action_log_opened", path=str(action_log_path))
    runtime = ExtendedRuntime(
        supervisor, priors, collector, skills, event_log, action_log=action_log
    )
    machine = StageMachine()
    shutdown_handlers = base.install_shutdown_handlers()
    takeover: Any | None = None
    if supervisor.get("base_publisher_takeover", {}).get("enabled") is True:
        from supervisor.paired_collection_controller import ManagedServiceTakeover

        takeover = ManagedServiceTakeover(supervisor)

    with ExitStack() as stack:
        stack.enter_context(
            HandCommandLease(
                supervisor["control"]["command_owner_lock"],
                mode="long_task_s1_s2_b2_s3",
                owner="base_arm_hand_supervisor",
            )
        )
        stack.enter_context(
            HandCommandLease(
                supervisor["control"]["hand_owner_lock"],
                mode="vla_and_latched_priors",
                owner="long_task_s1_s2_b2_s3",
            )
        )
        try:
            if takeover is not None:
                takeover.acquire()
                event_log.write("base_publisher_takeover_acquired", profile=PROFILE)
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
                elif stage == "B2":
                    runtime.run_base_stage(stage, prior_streams[stage], priors["priors"][stage])
                    print(
                        "[B2 COMPLETE] Base is stopped. Type RUN S3 only after the "
                        "cabinet and arm workspace are clear."
                    )
                elif stage.startswith("B"):
                    runtime.run_base_stage(stage, prior_streams[stage], priors["priors"][stage])
                elif stage == "S3":
                    runtime.transition_to_s3_preset()
                    runtime.prepare_s3_policy()
                    runtime.run_skill_stage(stage)
                else:
                    runtime.run_skill_stage(stage)
                machine.confirm_success(stage)
                event_log.write("stage_complete", stage=stage, next_stage=machine.current)
                if stage == "S2":
                    print(
                        "[S2 COMPLETE] S2 was confirmed by the operator. The next "
                        "explicit command is RUN B2."
                    )
            runtime.manual_hold("S3 complete; preserving the box transport pose")
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
                print(
                    f"Startup blocked before command interfaces were ready: {exc}",
                    file=sys.stderr,
                )
        finally:
            base.ignore_shutdown_signals(shutdown_handlers)
            try:
                runtime.close()
            except Exception as exc:
                event_log.write("runtime_close_failed", error=str(exc))
                print(f"WARNING: runtime cleanup failed: {exc}", file=sys.stderr)
            try:
                if takeover is not None:
                    takeover.restore(force_enable=True)
                    event_log.write("base_publisher_takeover_restored", f710_enabled=True)
            except Exception as exc:
                event_log.write("base_publisher_takeover_restore_failed", error=str(exc))
                print(f"WARNING: failed to restore F710 publisher: {exc}", file=sys.stderr)
            finally:
                base.restore_shutdown_handlers(shutdown_handlers)
                action_log.write("action_run_end", run_id=runtime.run_id)
                action_log.close()
                event_log.close()
    return 0


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    config_path = args.config.expanduser().resolve()
    if args.shadow:
        return shadow(args)
    if not args.execute:
        print(json.dumps(dry_run(config_path), indent=2, ensure_ascii=False))
        return 0
    config = load_yaml(config_path)
    acknowledgement = str(config["motion_acknowledgement"])
    if args.acknowledge_motion != acknowledgement:
        raise SystemExit(
            f"real execution blocked: pass --acknowledge-motion {acknowledgement}"
        )
    return execute(args)


if __name__ == "__main__":
    raise SystemExit(main())
