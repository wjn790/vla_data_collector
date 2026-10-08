from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from supervisor.core import (
    ArmCommandLimiter,
    EXECUTION_PROFILES,
    FeedbackZeroGate,
    SafetyStop,
    StateMachine,
    enforce_skill_executors,
    load_base_prior_stream,
    load_skills,
    validate_action_chunk,
)
from supervisor.long_task_supervisor import (
    EventLog,
    SupervisorRuntime,
    _shadow_chunk_summary,
    load_all,
    required_prior_ids,
)
from supervisor.policy_client import Packer, PolicyClient, unpackb


ROOT = Path(__file__).resolve().parents[1]


def test_skill_chain_and_s5_executor_mask() -> None:
    chain, skills = load_skills(ROOT / "config" / "skills.yaml")
    assert chain == ("B0", "S1", "S2", "B2", "S3", "B3", "S4", "S5", "S6")
    assert skills["S1"].prompt == (
        "grasp the cabinet door handle with the left hand, then hold it and adjust "
        "the left arm while the base retreats to pull the door open"
    )
    assert skills["S1"].prompt_version == "v2"
    assert skills["S1"].concurrent_base_prior == "B1"
    assert skills["S2"].prompt == (
        "release the cabinet door handle, use the back of the left hand to push the "
        "cabinet door slightly farther open, then move the left arm clear"
    )
    assert skills["S2"].prompt_version == "v3"
    assert skills["S2"].completion == {
        "mode": "operator_confirm",
        "requires_handle_release": True,
        "required_contact_surface": "back_of_left_hand",
        "requires_additional_door_opening": True,
        "requires_arm_clear": True,
    }
    arm, hand = enforce_skill_executors(
        skills["S5"],
        np.ones(14),
        np.arange(12),
        np.zeros(14),
        np.zeros(12),
    )
    assert np.all(arm[:7] == 0)
    assert np.all(arm[7:] == 1)
    assert np.count_nonzero(hand) == 1
    assert hand[8] == 8


def test_state_machine_requires_in_order_confirmation() -> None:
    machine = StateMachine()
    with pytest.raises(RuntimeError):
        machine.confirm_success("S1")
    for stage in machine.chain:
        assert machine.current == stage
        machine.confirm_success(stage)
    assert machine.complete


def test_b0_s1_b1_s2_profile_is_a_supported_prefix() -> None:
    chain = EXECUTION_PROFILES["b0_s1_b1_s2"]
    machine = StateMachine(chain)
    assert machine.current == "B0"
    for stage in chain:
        machine.confirm_success(stage)
    assert machine.complete


def test_b0_s1_b1_s2_loads_only_b0_and_concurrent_b1() -> None:
    _chain, skills = load_skills(ROOT / "config" / "skills.yaml")
    assert required_prior_ids(EXECUTION_PROFILES["b0_s1_b1_s2"], skills) == ("B0", "B1")


def test_supervisor_accepts_configured_policy_watchdog() -> None:
    config, _priors, _collector, _chain, _skills, _profile = load_all(
        ROOT / "config" / "inference_b0_s1_b1_s2.yaml"
    )
    assert 500 <= config["control"]["inference_watchdog_ms"] <= 5000


def test_policy_reset_is_sent_before_first_observation() -> None:
    class FakeConnection:
        def __init__(self) -> None:
            self.sent: list[bytes] = []

        def send(self, value: bytes) -> None:
            self.sent.append(value)

        def recv(self, timeout: float) -> bytes:
            assert timeout == 2.0
            return Packer().pack({"action": None})

    client = PolicyClient.__new__(PolicyClient)
    client._connection = FakeConnection()
    client._packer = Packer()
    elapsed = client.reset("svt_cabinet_b0_s1_b1_s2", timeout_sec=2.0)

    assert elapsed >= 0
    assert unpackb(client._connection.sent[0]) == {
        "reset": True,
        "robo_name": "svt_cabinet_b0_s1_b1_s2",
    }


def test_sync_policy_client_disables_keepalive_pings(monkeypatch: pytest.MonkeyPatch) -> None:
    captured: dict[str, object] = {}

    class FakeConnection:
        def recv(self, timeout: float) -> bytes:
            return Packer().pack({"protocol_version": "svt-policy-v1"})

    def fake_connect(uri: str, **kwargs: object) -> FakeConnection:
        captured["uri"] = uri
        captured.update(kwargs)
        return FakeConnection()

    import websockets.sync.client

    monkeypatch.setattr(websockets.sync.client, "connect", fake_connect)
    client = PolicyClient("10.0.0.18", 8006)

    assert client.metadata["protocol_version"] == "svt-policy-v1"
    assert captured["uri"] == "ws://10.0.0.18:8006"
    # Keepalive pings are disabled so a long first-request compile on the
    # server (which blocks its event loop) cannot be killed by the client's
    # 20 s ping timeout. Older websockets releases fall back to no options.
    assert captured.get("ping_interval") is None
    assert captured.get("ping_timeout") is None


def test_arm_limiter_applies_position_velocity_and_acceleration() -> None:
    limiter = ArmCommandLimiter(
        {
            "lower": [-1.0] * 14,
            "upper": [1.0] * 14,
            "position_margin_rad": 0.1,
            "max_velocity_radps": 0.5,
            "max_acceleration_radps2": 1.0,
        }
    )
    command, velocity, violations = limiter.limit(
        np.full(14, 5.0),
        np.zeros(14),
        np.zeros(14),
        0.1,
    )
    np.testing.assert_allclose(velocity, 0.1)
    np.testing.assert_allclose(command, 0.01)
    assert len(violations) == 14


def test_arm_limiter_supports_disabled_acceleration_limit() -> None:
    limiter = ArmCommandLimiter(
        {
            "lower": [-1.0] * 14,
            "upper": [1.0] * 14,
            "position_margin_rad": 0.1,
            "max_velocity_radps": 0.5,
            "max_acceleration_radps2": None,
        }
    )
    command, velocity, violations = limiter.limit(
        np.full(14, 5.0),
        np.zeros(14),
        np.zeros(14),
        0.1,
    )
    np.testing.assert_allclose(velocity, 0.5)
    np.testing.assert_allclose(command, 0.05)
    assert len(violations) == 14


def test_executed_action_frame_records_raw_command_and_feedback(tmp_path: Path) -> None:
    config, priors, collector, _chain, skills, _profile = load_all(
        ROOT / "config" / "inference_b0_s1_b1_s2.yaml"
    )
    event_path = tmp_path / "events.jsonl"
    action_path = tmp_path / "actions.jsonl"
    event_log = EventLog(event_path)
    action_log = EventLog(action_path)
    runtime = SupervisorRuntime(
        config,
        priors,
        collector,
        skills,
        event_log,
        action_log=action_log,
    )
    initial_arm = (runtime.arm_limiter.lower + runtime.arm_limiter.upper) / 2.0

    class FakeRos:
        def __init__(self) -> None:
            self.published: list[np.ndarray] = []

        def arm_state(self, max_age_sec: float = 0.2) -> tuple[np.ndarray, np.ndarray]:
            return initial_arm.copy(), np.zeros(14)

        def publish_base_zero(self) -> None:
            return None

        def publish_arms(self, command: np.ndarray) -> None:
            self.published.append(np.asarray(command).copy())

    class FakeHands:
        def __init__(self) -> None:
            self.command = np.zeros(12)

        def state(self, max_age_sec: float = 0.25) -> np.ndarray:
            return self.command.copy()

        def publish(self, command: np.ndarray) -> None:
            self.command = np.asarray(command).copy()

        def latched_action(self) -> np.ndarray:
            return self.command.copy()

    runtime.ros = FakeRos()
    runtime.hands = FakeHands()
    runtime.last_arm_command = initial_arm.copy()
    runtime.s1_hand_filter.enabled = False
    raw_arm_target = initial_arm.copy()
    raw_arm_target[3] = -1.0
    raw_hand_target = np.arange(12, dtype=float)
    runtime._execute_skill_action(
        "S1",
        skills["S1"],
        raw_arm_target,
        raw_hand_target,
        initial_arm,
        np.zeros(12),
        publish_base_zero=True,
        action_block_index=2,
        chunk_frame_index=4,
    )
    action_log.close()
    event_log.close()

    records = [json.loads(line) for line in action_path.read_text().splitlines()]
    assert len(records) == 1
    record = records[0]
    assert record["event"] == "executed_action_frame"
    assert record["action_block_index"] == 2
    assert record["chunk_frame_index"] == 4
    assert record["model_arm_target"][3] == -1.0
    assert record["bounded_arm_target"][3] == pytest.approx(0.05)
    np.testing.assert_allclose(record["published_arm_command"], runtime.ros.published[0])
    np.testing.assert_allclose(record["arm_feedback_before"], initial_arm)
    np.testing.assert_allclose(record["arm_feedback_after"], initial_arm)
    assert record["arm_limit_violations"] == ["joint_3:position_clipped"]


def test_s1_hand_phase_filter_debounces_coupled_grasp_and_latches_closed(
    tmp_path: Path,
) -> None:
    config, priors, collector, _chain, skills, _profile = load_all(
        ROOT / "config" / "inference_b0_s1_b1_s2.yaml"
    )
    event_log = EventLog(tmp_path / "events.jsonl")
    runtime = SupervisorRuntime(config, priors, collector, skills, event_log)
    open_hand = np.array([240.0] * 6 + [0.0] * 6)
    three_closed = np.array([240.0] * 3 + [0.0] * 3 + [0.0] * 6)
    grasped = np.array([100.0] * 3 + [0.0] * 3 + [0.0] * 6)
    runtime.last_arm_command = np.zeros(14)
    runtime.last_hand_command = open_hand.copy()

    for _ in range(int(config["control"]["s1_three_finger_confirm_frames"])):
        arm, hand = runtime._apply_s1_hand_phase_filter(
            "S1", np.ones(14), three_closed
        )
        runtime.last_arm_command = arm
        runtime.last_hand_command = hand
    assert runtime.s1_hand_filter.phase == "three_closed"

    for _ in range(int(config["control"]["s1_grasp_confirm_frames"]) - 1):
        arm, hand = runtime._apply_s1_hand_phase_filter(
            "S1", np.full(14, 2.0), grasped
        )
        np.testing.assert_allclose(arm[:7], runtime.last_arm_command[:7])
        np.testing.assert_allclose(hand[:6], runtime.last_hand_command[:6])
        runtime.last_arm_command = arm
        runtime.last_hand_command = hand

    arm, hand = runtime._apply_s1_hand_phase_filter(
        "S1", np.full(14, 2.0), grasped
    )
    np.testing.assert_allclose(arm, 2.0)
    np.testing.assert_allclose(hand, grasped)
    runtime.last_arm_command = arm
    runtime.last_hand_command = hand
    assert runtime.s1_hand_filter.phase == "grasped"

    adjusted_arm, latched_hand = runtime._apply_s1_hand_phase_filter(
        "S1", np.full(14, 3.0), open_hand
    )
    np.testing.assert_allclose(adjusted_arm, 3.0)
    np.testing.assert_allclose(latched_hand, grasped)
    event_log.close()

    events = [
        json.loads(line)
        for line in tmp_path.joinpath("events.jsonl").read_text().splitlines()
    ]
    transitions = [
        event["transition"]
        for event in events
        if event["event"] == "s1_hand_phase_transition"
    ]
    assert transitions == ["approach->three_closed", "three_closed->grasped"]


def test_preset_move_commands_arms_and_hand(tmp_path: Path) -> None:
    config, priors, collector, _chain, skills, _profile = load_all(
        ROOT / "config" / "inference_b0_s1_b1_s2.yaml"
    )
    event_log = EventLog(tmp_path / "events.jsonl")
    runtime = SupervisorRuntime(config, priors, collector, skills, event_log)

    class FakeRos:
        def __init__(self) -> None:
            self.arm = np.zeros(14)
            self.published: list[np.ndarray] = []

        def assert_command_ownership(self) -> None:
            return None

        def arm_state(self, max_age_sec: float = 0.2) -> tuple[np.ndarray, np.ndarray]:
            return self.arm.copy(), np.zeros(14)

        def publish_base_zero(self) -> None:
            return None

        def publish_arms(self, command: np.ndarray) -> None:
            self.published.append(np.asarray(command).copy())
            self.arm = np.asarray(command).copy()

    class FakeHands:
        def __init__(self) -> None:
            self.cmd = np.zeros(12)

        def publish(self, command: np.ndarray) -> None:
            self.cmd = np.asarray(command).copy()

    runtime.ros = FakeRos()
    runtime.hands = FakeHands()
    preset = {
        "arm_position": [0.0] * 14,
        "hand_position": [254.0] * 6 + [0.0] * 6,
        "tolerance_rad": 0.05,
        "timeout_sec": 5.0,
    }
    runtime._move_to_preset(preset, "S1")
    assert float(np.max(np.abs(runtime.ros.arm))) <= 0.05
    np.testing.assert_allclose(runtime.hands.cmd[:6], 254.0)
    event_log.close()

    events = [
        json.loads(line)
        for line in tmp_path.joinpath("events.jsonl").read_text().splitlines()
    ]
    assert any(event["event"] == "preset_move_complete" for event in events)


def test_preset_move_fails_fast_when_arm_feedback_stalls(tmp_path: Path) -> None:
    config, priors, collector, _chain, skills, _profile = load_all(
        ROOT / "config" / "inference_b0_s1_b1_s2.yaml"
    )
    event_log = EventLog(tmp_path / "events.jsonl")
    runtime = SupervisorRuntime(config, priors, collector, skills, event_log)

    class FrozenRos:
        def assert_command_ownership(self) -> None:
            return None

        def arm_state(self, max_age_sec: float = 0.2) -> tuple[np.ndarray, np.ndarray]:
            return np.zeros(14), np.zeros(14)

        def publish_base_zero(self) -> None:
            return None

        def publish_arms(self, command: np.ndarray) -> None:
            return None

    class FakeHands:
        def publish(self, command: np.ndarray) -> None:
            return None

    runtime.ros = FrozenRos()
    runtime.hands = FakeHands()
    preset = {
        "arm_position": [0.5] * 14,
        "hand_position": [254.0] * 6 + [0.0] * 6,
        "tolerance_rad": 0.05,
        "timeout_sec": 25.0,
        "stall_sec": 0.1,
    }
    with pytest.raises(SafetyStop, match="feedback stalled"):
        runtime._move_to_preset(preset, "S1")
    event_log.close()


def write_prior(path: Path, *, extra_key: bool = False) -> str:
    rows = []
    for index in range(19):
        timestamp = index / 15
        moving = 8 <= index <= 9
        row = {
            "timestamp_sec": timestamp,
            "vx": -0.05 if moving else 0.0,
            "vy": 0.0,
            "wz": 0.0,
        }
        if extra_key and index == 0:
            row["arm"] = [1]
        rows.append(row)
    path.write_text("\n".join(json.dumps(row) for row in rows) + "\n", encoding="utf-8")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_base_prior_accepts_only_checksummed_twist_stream(tmp_path: Path) -> None:
    stream = tmp_path / "B1.jsonl"
    checksum = write_prior(stream)
    samples = load_base_prior_stream(
        "B1",
        {
            "calibrated": True,
            "stream": stream.name,
            "sha256": checksum,
            "max_duration_sec": 2,
            "start_zero_sec": 0.5,
            "end_zero_sec": 0.5,
            "max_retreat_distance_m": 0.1,
        },
        config_dir=tmp_path,
        limits={
            "max_abs_vx_mps": 0.15,
            "max_abs_vy_mps": 0.15,
            "max_abs_wz_radps": 0.2,
        },
        require_calibrated=True,
    )
    assert len(samples) == 19


def test_base_prior_rejects_non_twist_fields(tmp_path: Path) -> None:
    stream = tmp_path / "bad.jsonl"
    checksum = write_prior(stream, extra_key=True)
    with pytest.raises(ValueError, match="only"):
        load_base_prior_stream(
            "B0",
            {
                "calibrated": True,
                "stream": stream.name,
                "sha256": checksum,
                "max_duration_sec": 2,
                "start_zero_sec": 0.5,
                "end_zero_sec": 0.5,
            },
            config_dir=tmp_path,
            limits={
                "max_abs_vx_mps": 0.15,
                "max_abs_vy_mps": 0.15,
                "max_abs_wz_radps": 0.2,
            },
            require_calibrated=True,
        )


def test_action_chunk_shape_and_zero_gate() -> None:
    arm, hand = validate_action_chunk(
        {
            "action.arm.position": np.zeros((50, 14)),
            "action.hand.position": np.zeros((50, 12)),
        }
    )
    assert arm.shape == (50, 14) and hand.shape == (50, 12)
    gate = FeedbackZeroGate(0.01, 0.5)
    assert not gate.update(0, 0, 0, now=1.0)
    assert gate.update(0, 0, 0, now=1.5)
    assert not gate.update(0.1, 0, 0, now=1.6)


def test_shadow_chunk_summary_masks_non_executing_dimensions() -> None:
    _chain, skills = load_skills(ROOT / "config" / "skills.yaml")
    supervisor = {
        "arm_limits": {"lower": [-1.0] * 14, "upper": [1.0] * 14},
        "hands": {"left_o6_min": [0] * 6, "left_o6_max": [255] * 6},
    }
    arm_chunk = np.full((50, 14), 9.0)
    hand_chunk = np.full((50, 12), 300.0)
    summary = _shadow_chunk_summary(
        supervisor,
        skills["S1"],
        arm_chunk,
        hand_chunk,
        np.zeros(14),
        np.zeros(12),
    )
    assert summary["arm_hard_limit_values"] == 7 * 50
    assert summary["left_o6_out_of_range_values"] == 6 * 50
