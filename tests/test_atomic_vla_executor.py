from __future__ import annotations

import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest

from supervisor.atomic_vla_executor import (
    AtomicRuntime,
    AsyncPolicyWorker,
    ChunkCursor,
    ProcessRecord,
    RuntimeTakeover,
    delayed_chunk_start,
    dry_run,
    load_all,
    process_matches,
    validate_policy_metadata,
)
from supervisor.hand_adapter import prepare_linkerhand_sdk_paths
from supervisor.ros_adapter import RosRobotAdapter, publisher_conflicts


ROOT = Path(__file__).resolve().parents[1]


def test_default_mode_is_non_motion_and_fixed_for_svt() -> None:
    result = dry_run(ROOT / "config" / "atomic_vla_executor.yaml")
    assert result["motion_published"] is False
    assert result["base_command"] == "zero_only"
    assert result["optional_emergency_stop"] == {
        "topic": "/exo/gamepad_keys",
        "button_index": 13,
        "latched": True,
    }
    assert result["operator_stop"] == {"ctrl_c": True, "enter": True}
    assert result["policy"]["chunk_size"] == 50
    assert result["policy"]["replan_frames"] == 20
    assert result["policy"]["max_chunk_frames"] == 25
    assert result["runtime_takeover"] == {
        "stops_before_motion": [
            "exoskeleton_bridge_node",
            "svtrobo-f710.service",
        ],
        "restores_original_state_on_exit": True,
        "arm_restore_requires_operator_confirmation": True,
    }


def test_policy_metadata_requires_exact_checkpoint() -> None:
    config, _ = load_all(ROOT / "config" / "atomic_vla_executor.yaml")
    metadata = {
        "model_path": config["policy"]["expected_model_path"],
        "chunk_size": 50,
        "action_keys": ["action.arm.position", "action.hand.position"],
    }
    validate_policy_metadata(metadata, config)
    metadata["model_path"] = "/wrong/checkpoint"
    with pytest.raises(RuntimeError, match="model mismatch"):
        validate_policy_metadata(metadata, config)


def test_delayed_chunk_uses_predicted_time_but_keeps_25_actions() -> None:
    assert delayed_chunk_start(1.3, 15, 50, 25, False) == 0
    assert delayed_chunk_start(1.3, 15, 50, 25, True) == 20
    assert delayed_chunk_start(9.0, 15, 50, 25, True) == 25
    cursor = ChunkCursor(
        arm=np.arange(50 * 14).reshape(50, 14),
        hand=np.arange(50 * 12).reshape(50, 12),
        start_index=20,
        max_frames=25,
    )
    actions = []
    while (action := cursor.next_action()) is not None:
        actions.append(action)
    assert len(actions) == 25
    np.testing.assert_array_equal(actions[0][0], cursor.arm[20])
    np.testing.assert_array_equal(actions[-1][0], cursor.arm[44])


class FakePolicyClient:
    def __init__(self, _host: str, _port: int, *, connect_timeout_sec: float) -> None:
        assert connect_timeout_sec > 0
        self.metadata = {
            "model_path": "/fake",
            "chunk_size": 50,
            "action_keys": ["action.arm.position", "action.hand.position"],
        }
        self.closed = False

    def infer(self, payload: dict[str, Any], _timeout_sec: float) -> tuple[dict[str, Any], float]:
        time.sleep(0.01)
        if payload.get("reset"):
            return {"action": None}, 0.01
        return {
            "action.arm.position": np.zeros((50, 14)),
            "action.hand.position": np.zeros((50, 12)),
        }, 0.01

    def close(self) -> None:
        self.closed = True


def test_policy_requests_run_on_background_worker() -> None:
    worker = AsyncPolicyWorker(
        "fake",
        1,
        connect_timeout_sec=1,
        client_factory=FakePolicyClient,
    )
    worker.start()
    reset_id = worker.submit(
        "reset",
        {"reset": True, "robo_name": "svt"},
        1,
        motion_overlapped=False,
    )
    reset = worker.wait_result(reset_id, 1)
    assert reset.error is None
    request_id = worker.submit(
        "inference",
        {"task": "test"},
        1,
        motion_overlapped=True,
    )
    result = worker.wait_result(request_id, 1)
    worker.close()
    assert result.error is None
    assert result.request.motion_overlapped is True
    assert result.response is not None
    assert result.response["action.arm.position"].shape == (50, 14)


def test_command_conflict_filter_keeps_other_publishers_only() -> None:
    class FakeNode:
        def get_publishers_info_by_topic(self, topic: str) -> list[Any]:
            if topic == "/arms":
                return [
                    SimpleNamespace(node_name="atomic"),
                    SimpleNamespace(node_name="teleop"),
                ]
            return []

    topics = ("/arms", "/base")
    assert publisher_conflicts(FakeNode(), topics) == {
        "/arms": ["atomic", "teleop"]
    }
    assert publisher_conflicts(FakeNode(), topics, own_node_name="atomic") == {
        "/arms": ["teleop"]
    }


def test_process_matching_requires_every_configured_fragment() -> None:
    record = ProcessRecord(
        123,
        (
            "/usr/bin/python3",
            "/opt/ros/humble/bin/ros2",
            "launch",
            "qnbot_teleoperator",
            "exoskeleton_bridge.launch.py",
            "gripper_scaling_factor:=0.05",
        ),
    )
    assert process_matches(
        record,
        ["/opt/ros/humble/bin/ros2", "qnbot_teleoperator", "exoskeleton_bridge.launch.py"],
    )
    assert not process_matches(record, ["exoskeleton_bridge.launch.py", "wrong_package"])


def test_f710_sudo_command_is_non_interactive(monkeypatch: pytest.MonkeyPatch) -> None:
    config, _ = load_all(ROOT / "config" / "atomic_vla_executor.yaml")
    commands: list[list[str]] = []

    def fake_run(command: list[str], **kwargs: Any) -> SimpleNamespace:
        commands.append(command)
        return SimpleNamespace(returncode=0, stdout="")

    monkeypatch.setattr("supervisor.atomic_vla_executor.subprocess.run", fake_run)
    takeover = RuntimeTakeover(config, SimpleNamespace(write=lambda *args, **kwargs: None))
    takeover._sudo(
        config["runtime_takeover"]["systemctl"],
        "stop",
        config["runtime_takeover"]["f710_service"],
    )

    assert commands == [[
        "/usr/bin/sudo",
        "-n",
        "/usr/bin/systemctl",
        "stop",
        "svtrobo-f710.service",
    ]]


def test_linkerhand_core_package_precedes_collector_module(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    glove_root = Path("/home/svt/glove_control")
    supervisor_root = ROOT / "supervisor"
    monkeypatch.setattr(
        sys,
        "path",
        [str(supervisor_root), str(glove_root), *sys.path],
    )

    prepare_linkerhand_sdk_paths(glove_root)

    assert sys.path[:3] == [
        str(glove_root / "linkerhand_sdk" / "LinkerHand"),
        str(glove_root / "linkerhand_sdk"),
        str(glove_root),
    ]


def test_feedback_check_does_not_require_gamepad_messages() -> None:
    config, collector = load_all(ROOT / "config" / "atomic_vla_executor.yaml")
    runtime = AtomicRuntime(
        config,
        collector,
        SimpleNamespace(write=lambda *args, **kwargs: None),
    )
    runtime.ros = SimpleNamespace(arm_state=lambda *_args: np.zeros(14))
    runtime.hands = SimpleNamespace(state=lambda *_args: np.zeros(12))

    runtime._check_feedback()


def test_wuji_multithread_access_is_enabled_after_controller_creation(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    prepare_linkerhand_sdk_paths(Path("/home/svt/glove_control"))
    import mixed_glove_teleop as mixed

    calls: list[str] = []

    class FakeO6:
        def __init__(self, **_kwargs: Any) -> None:
            self.hand = SimpleNamespace(x01=0)

        def set_speed(self, _values: list[int]) -> None:
            pass

        def set_torque(self, _values: list[int]) -> None:
            pass

        def get_state(self) -> list[int]:
            return [0] * 6

    class FakeController:
        pass

    class FakeHand:
        def __init__(self, *, serial_number: str) -> None:
            assert serial_number == "test-wuji"

        def read_joint_lower_limit(self) -> np.ndarray:
            return np.zeros((5, 4))

        def read_joint_upper_limit(self) -> np.ndarray:
            return np.ones((5, 4))

        def read_joint_actual_position(self) -> np.ndarray:
            return np.zeros((5, 4))

        def write_joint_enabled(self, enabled: bool) -> None:
            assert enabled is True

        def realtime_controller(self, **_kwargs: Any) -> FakeController:
            calls.append("controller_created")
            return FakeController()

        def disable_thread_safe_check(self) -> None:
            calls.append("thread_check_disabled")

    monkeypatch.setattr(mixed.dual, "LinkerHandApi", FakeO6)
    monkeypatch.setattr(mixed.wujihandpy, "Hand", FakeHand)
    monkeypatch.setattr(mixed.time, "sleep", lambda _seconds: None)
    hardware = mixed.MixedHardware(
        SimpleNamespace(
            o6_can="can1",
            hand_serial="test-wuji",
            hardware_limit_margin_rad=0.02,
        ),
        {"o6": {}},
    )

    hardware.open()

    assert calls == ["controller_created", "thread_check_disabled"]


def test_ros_startup_requires_arm_feedback_but_not_unused_base_feedback() -> None:
    adapter = RosRobotAdapter.__new__(RosRobotAdapter)
    adapter.lock = threading.Lock()
    adapter.joint_position = np.zeros(14)
    adapter.base_received_at = 0.0

    adapter.wait_ready(timeout_sec=0.01)

    adapter.joint_position = None
    with pytest.raises(RuntimeError, match="joint feedback did not become ready"):
        adapter.wait_ready(timeout_sec=0.01)
