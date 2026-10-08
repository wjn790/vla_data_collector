from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np
import yaml


EXPECTED_CHAIN = ("B0", "S1", "S2", "B2", "S3", "B3", "S4", "S5", "S6")
EXECUTION_PROFILES = {
    "full": EXPECTED_CHAIN,
    # B1 is intentionally embedded in the concurrent S1 path. It is not an
    # independent VLA state and therefore does not appear in this sequence.
    "b0_s1_b1_s2": EXPECTED_CHAIN[:3],
}


class SafetyStop(RuntimeError):
    pass


def load_yaml(path: Path) -> dict[str, Any]:
    value = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"expected YAML mapping: {path}")
    if int(value.get("schema_version", 0)) != 1:
        raise ValueError(f"schema_version must be 1: {path}")
    return value


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def finite_array(value: Any, shape: tuple[int, ...], label: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f"{label} must be finite with shape {shape}, got {result.shape}")
    return result


@dataclass(frozen=True)
class Skill:
    skill_id: str
    prompt: str
    prompt_version: str
    allowed_executors: frozenset[str]
    max_duration_sec: float | None
    completion: dict[str, Any]
    concurrent_base_prior: str | None


def load_skills(path: Path) -> tuple[tuple[str, ...], dict[str, Skill]]:
    value = load_yaml(path)
    chain = tuple(str(stage) for stage in value.get("state_chain", ()))
    if chain != EXPECTED_CHAIN:
        raise ValueError(f"state_chain must be exactly {EXPECTED_CHAIN}, got {chain}")
    raw_skills = value.get("skills")
    if not isinstance(raw_skills, dict) or set(raw_skills) != {f"S{i}" for i in range(1, 7)}:
        raise ValueError("skills config must define exactly S1 through S6")
    skills: dict[str, Skill] = {}
    prompts: set[str] = set()
    valid_executors = {
        "left_arm",
        "right_arm",
        "left_hand",
        "right_hand",
        "right_hand_index",
    }
    for skill_id, raw in raw_skills.items():
        if not isinstance(raw, dict):
            raise ValueError(f"{skill_id} must be a mapping")
        prompt = str(raw.get("prompt", "")).strip()
        if not prompt or prompt in prompts:
            raise ValueError(f"{skill_id} prompt is empty or duplicated")
        prompts.add(prompt)
        allowed = frozenset(str(item) for item in raw.get("allowed_executors", ()))
        if not allowed or not allowed <= valid_executors:
            raise ValueError(f"{skill_id} has invalid allowed_executors: {sorted(allowed)}")
        raw_max_duration = raw.get("max_duration_sec", 0)
        if raw_max_duration is None:
            if skill_id != "S1":
                raise ValueError(
                    f"only S1 may set max_duration_sec to null; got {skill_id}"
                )
            max_duration: float | None = None
        else:
            max_duration = float(raw_max_duration)
            if not math.isfinite(max_duration) or max_duration <= 0:
                raise ValueError(f"{skill_id} max_duration_sec must be positive")
        completion = raw.get("completion")
        if not isinstance(completion, dict) or completion.get("mode") != "operator_confirm":
            raise ValueError(f"{skill_id} must use operator_confirm in the first deployment")
        skills[skill_id] = Skill(
            skill_id=skill_id,
            prompt=prompt,
            prompt_version=str(raw.get("prompt_version", "")).strip(),
            allowed_executors=allowed,
            max_duration_sec=max_duration,
            completion=dict(completion),
            concurrent_base_prior=(
                str(raw["concurrent_base_prior"])
                if raw.get("concurrent_base_prior") is not None
                else None
            ),
        )
    if skills["S1"].concurrent_base_prior != "B1":
        raise ValueError("S1 must declare concurrent_base_prior: B1")
    if any(
        skill.concurrent_base_prior is not None
        for skill_id, skill in skills.items()
        if skill_id != "S1"
    ):
        raise ValueError("only S1 may declare a concurrent base prior")
    return chain, skills


@dataclass(frozen=True)
class TwistSample:
    timestamp_sec: float
    vx: float
    vy: float
    wz: float

    @property
    def is_zero(self) -> bool:
        return self.vx == 0.0 and self.vy == 0.0 and self.wz == 0.0


def load_base_prior_stream(
    prior_id: str,
    entry: dict[str, Any],
    *,
    config_dir: Path,
    limits: dict[str, Any],
    require_calibrated: bool,
    sample_rate_hz: float = 15.0,
) -> list[TwistSample]:
    calibrated = entry.get("calibrated") is True
    stream_value = entry.get("stream")
    checksum = str(entry.get("sha256") or "").strip()
    if not calibrated or not stream_value or not checksum:
        if require_calibrated:
            raise SafetyStop(f"{prior_id} is not calibrated with a checksummed stream")
        return []
    stream_path = Path(str(stream_value)).expanduser()
    if not stream_path.is_absolute():
        stream_path = (config_dir / stream_path).resolve()
    if not stream_path.is_file():
        raise FileNotFoundError(stream_path)
    actual_checksum = sha256_file(stream_path)
    if actual_checksum != checksum:
        raise SafetyStop(
            f"{prior_id} checksum mismatch: expected {checksum}, got {actual_checksum}"
        )

    allowed_keys = {"timestamp_sec", "vx", "vy", "wz"}
    samples: list[TwistSample] = []
    with stream_path.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            if not line.strip():
                continue
            row = json.loads(line)
            if not isinstance(row, dict) or set(row) != allowed_keys:
                raise ValueError(
                    f"{stream_path}:{line_number} must contain only {sorted(allowed_keys)}"
                )
            sample = TwistSample(
                timestamp_sec=float(row["timestamp_sec"]),
                vx=float(row["vx"]),
                vy=float(row["vy"]),
                wz=float(row["wz"]),
            )
            if not all(
                math.isfinite(value)
                for value in (sample.timestamp_sec, sample.vx, sample.vy, sample.wz)
            ):
                raise ValueError(f"{stream_path}:{line_number} contains a non-finite value")
            if samples and sample.timestamp_sec <= samples[-1].timestamp_sec:
                raise ValueError(f"{stream_path}:{line_number} timestamp is not increasing")
            samples.append(sample)
    if len(samples) < 2 or samples[0].timestamp_sec != 0.0:
        raise ValueError(f"{prior_id} must contain at least two samples and start at timestamp 0")
    expected_period = 1.0 / float(sample_rate_hz)
    for previous, current in zip(samples, samples[1:]):
        actual_period = current.timestamp_sec - previous.timestamp_sec
        if abs(actual_period - expected_period) > 0.005:
            raise SafetyStop(
                f"{prior_id} cadence {actual_period:.6f}s is not {sample_rate_hz:g} Hz"
            )
    max_duration = float(entry["max_duration_sec"])
    if samples[-1].timestamp_sec > max_duration:
        raise SafetyStop(f"{prior_id} exceeds max_duration_sec={max_duration}")
    bounds = (
        ("vx", float(limits["max_abs_vx_mps"])),
        ("vy", float(limits["max_abs_vy_mps"])),
        ("wz", float(limits["max_abs_wz_radps"])),
    )
    for sample in samples:
        for name, maximum in bounds:
            if abs(getattr(sample, name)) > maximum:
                raise SafetyStop(
                    f"{prior_id} {name}={getattr(sample, name)} exceeds configured {maximum}"
                )
    start_zero_sec = float(entry.get("start_zero_sec", 0))
    end_zero_sec = float(entry.get("end_zero_sec", 0))
    if any(not sample.is_zero for sample in samples if sample.timestamp_sec <= start_zero_sec):
        raise SafetyStop(f"{prior_id} does not maintain its required leading zero segment")
    final_time = samples[-1].timestamp_sec
    if any(
        not sample.is_zero
        for sample in samples
        if sample.timestamp_sec >= final_time - end_zero_sec
    ):
        raise SafetyStop(f"{prior_id} does not maintain its required trailing zero segment")
    if prior_id == "B1":
        if entry.get("max_retreat_distance_m") is None:
            raise SafetyStop("B1 requires max_retreat_distance_m")
        commanded_distance = sum(
            math.hypot(previous.vx, previous.vy)
            * (current.timestamp_sec - previous.timestamp_sec)
            for previous, current in zip(samples, samples[1:])
        )
        if commanded_distance > float(entry["max_retreat_distance_m"]):
            raise SafetyStop(
                f"B1 commanded distance {commanded_distance:.3f} m exceeds "
                f"max_retreat_distance_m={entry['max_retreat_distance_m']}"
            )
    return samples


class StateMachine:
    def __init__(self, chain: Iterable[str] = EXPECTED_CHAIN) -> None:
        self.chain = tuple(chain)
        if self.chain not in EXECUTION_PROFILES.values():
            raise ValueError(f"unsupported state chain: {self.chain}")
        self.index = 0
        self.aborted = False
        self.abort_reason: str | None = None

    @property
    def current(self) -> str | None:
        return None if self.index >= len(self.chain) else self.chain[self.index]

    @property
    def complete(self) -> bool:
        return self.current is None and not self.aborted

    def confirm_success(self, stage: str) -> None:
        if self.aborted or stage != self.current:
            raise RuntimeError(f"cannot confirm {stage}; current state is {self.current}")
        self.index += 1

    def abort(self, reason: str) -> None:
        self.aborted = True
        self.abort_reason = str(reason)


class ArmCommandLimiter:
    def __init__(self, config: dict[str, Any]) -> None:
        lower = finite_array(config["lower"], (14,), "arm lower limits")
        upper = finite_array(config["upper"], (14,), "arm upper limits")
        margin = float(config.get("position_margin_rad", 0))
        self.lower = lower + margin
        self.upper = upper - margin
        if np.any(self.lower >= self.upper):
            raise ValueError("arm position margin collapses a joint range")
        self.max_velocity = float(config["max_velocity_radps"])
        raw_acceleration = config.get("max_acceleration_radps2")
        self.max_acceleration = (
            None if raw_acceleration is None else float(raw_acceleration)
        )
        if self.max_velocity <= 0:
            raise ValueError("arm velocity limit must be positive")
        if self.max_acceleration is not None and self.max_acceleration <= 0:
            raise ValueError("arm acceleration limit must be positive when enabled")

    def limit(
        self,
        target: Any,
        previous_command: Any,
        previous_velocity: Any,
        dt: float,
    ) -> tuple[np.ndarray, np.ndarray, list[str]]:
        target_array = finite_array(target, (14,), "arm target")
        previous = finite_array(previous_command, (14,), "previous arm command")
        velocity = finite_array(previous_velocity, (14,), "previous arm velocity")
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be positive")
        clipped_target = np.clip(target_array, self.lower, self.upper)
        violations = [
            f"joint_{index}:position_clipped"
            for index in np.flatnonzero(clipped_target != target_array)
        ]
        desired_velocity = np.clip(
            (clipped_target - previous) / dt,
            -self.max_velocity,
            self.max_velocity,
        )
        if self.max_acceleration is None:
            next_velocity = desired_velocity
        else:
            max_velocity_delta = self.max_acceleration * dt
            next_velocity = velocity + np.clip(
                desired_velocity - velocity,
                -max_velocity_delta,
                max_velocity_delta,
            )
        command = np.clip(previous + next_velocity * dt, self.lower, self.upper)
        return command, next_velocity, violations


def enforce_skill_executors(
    skill: Skill,
    arm_target: Any,
    hand_target: Any,
    latched_arm: Any,
    latched_hand: Any,
) -> tuple[np.ndarray, np.ndarray]:
    arm = finite_array(arm_target, (14,), "arm action").copy()
    hand = finite_array(hand_target, (12,), "hand action").copy()
    arm_latch = finite_array(latched_arm, (14,), "latched arm")
    hand_latch = finite_array(latched_hand, (12,), "latched hand")
    if "left_arm" not in skill.allowed_executors:
        arm[:7] = arm_latch[:7]
    if "right_arm" not in skill.allowed_executors:
        arm[7:] = arm_latch[7:]
    if "left_hand" not in skill.allowed_executors:
        hand[:6] = hand_latch[:6]
    if "right_hand" not in skill.allowed_executors:
        hand[6:] = hand_latch[6:]
    if "right_hand_index" in skill.allowed_executors:
        hand[6:] = hand_latch[6:]
        hand[8] = finite_array(hand_target, (12,), "hand action")[8]
    return arm, hand


def validate_action_chunk(response: dict[str, Any], chunk_size: int = 50) -> tuple[np.ndarray, np.ndarray]:
    if not isinstance(response, dict):
        raise ValueError("policy response must be a mapping")
    arm = finite_array(response.get("action.arm.position"), (chunk_size, 14), "arm chunk")
    hand = finite_array(response.get("action.hand.position"), (chunk_size, 12), "hand chunk")
    return arm, hand


class FeedbackZeroGate:
    def __init__(self, tolerance: float, stable_sec: float) -> None:
        self.tolerance = float(tolerance)
        self.stable_sec = float(stable_sec)
        self.zero_since: float | None = None

    def update(self, vx: float, vy: float, wz: float, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else float(now)
        if max(abs(vx), abs(vy), abs(wz)) <= self.tolerance:
            if self.zero_since is None:
                self.zero_since = now
            return now - self.zero_since >= self.stable_sec
        self.zero_since = None
        return False
