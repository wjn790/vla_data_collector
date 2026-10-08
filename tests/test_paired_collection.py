from __future__ import annotations

import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import numpy as np
import yaml

from supervisor.core import SafetyStop
from supervisor.paired_collection_controller import PairedCollectionRuntime
from supervisor.paired_collection_core import PairAction, PairState, PairStateMachine, validate_config


ROOT = Path(__file__).resolve().parents[1]


def test_button_mapping_and_config_are_fixed() -> None:
    config = yaml.safe_load((ROOT / "config" / "paired_collection.yaml").read_text())
    mapping = validate_config(config)
    assert (mapping.right_a, mapping.right_b, mapping.right_c, mapping.right_d) == (
        11,
        12,
        13,
        14,
    )
    assert config["hand_controller"]["left_c_button_index"] == 7
    assert config["hand_controller"]["left_c_action_key"] == "4"
    takeover = config["base_publisher_takeover"]
    assert takeover["enabled"] is True
    assert takeover["scope"] == "first_right_a_through_s2_completion"
    assert takeover["managed_services"] == [
        {
            "unit": "svtrobo-f710.service",
            "publisher_node": "my_controller_node",
            "enable_topic": "/f710/enable",
            "status_topic": "/f710/status",
        }
    ]


def test_hand_controller_releases_paired_buttons() -> None:
    candidates = (
        ROOT / "dual_hand_config.json",
        Path("/home/svt/glove_control/wuji_bridge/dual_hand_config.json"),
    )
    config_path = next((path for path in candidates if path.is_file()), None)
    assert config_path is not None
    config = json.loads(config_path.read_text())
    button_config = config["o6_button_controller"]
    assert button_config["left"]["button_index"] == 7
    assert button_config["left"]["action_key"] == "4"
    action = config["o6_hotkeys"]["actions"]["4"]
    staged = action["staged_activation"]
    assert staged["enabled"] is True
    assert staged["delay_sec"] == 3.0
    assert staged["first_stage_positions"] == {
        "middle_flex": 0,
        "ring_flex": 0,
        "pinky_flex": 0,
    }
    assert button_config["right"]["button_enabled"] is False
    assert button_config["collection"]["enabled"] is False


def test_a_a_b_b_pair_sequence() -> None:
    machine = PairStateMachine()
    assert machine.press("right_a") == PairAction.RUN_B0
    machine.base_complete("B0")
    machine.collector_started("S1")
    assert machine.state == PairState.RECORDING_S1
    assert machine.press("right_a") == PairAction.MARK_S1_GRASP_COMPLETE
    assert machine.state == PairState.AWAITING_B1
    assert machine.press("right_b") == PairAction.RUN_B1
    machine.base_complete("B1")
    assert machine.state == PairState.SWITCHING_TO_S2
    machine.switch_to_s2()
    assert machine.state == PairState.RECORDING_S2
    assert machine.press("right_b") == PairAction.STOP_S2
    machine.collector_stopped("S2")
    assert machine.state == PairState.COMPLETE
    machine.reset_completed_pair()
    assert machine.state == PairState.IDLE
    assert machine.press("right_a") == PairAction.RUN_B0


def test_b0_wait_notice_precedes_s1_ready_notice(capsys, monkeypatch) -> None:
    class EventLog:
        def __init__(self) -> None:
            self.events: list[tuple[str, dict[str, object]]] = []

        def write(self, event: str, **values: object) -> None:
            self.events.append((event, values))

    runtime = object.__new__(PairedCollectionRuntime)
    runtime.machine = PairStateMachine()
    runtime.machine.press("right_a")
    runtime.machine_lock = threading.Lock()
    runtime.attempt_record = {"phase_markers": {}}
    runtime.event_log = EventLog()
    runtime.collector_recording_start_ns = None
    runtime.collector_episode_path = None
    runtime._run_prior = lambda stage: None

    def start_collector(skill: str) -> None:
        assert skill == "S1"
        runtime.collector_recording_start_ns = 150
        runtime.collector_episode_path = Path("/tmp/continuous_pair")

    runtime._start_collector = start_collector
    runtime._update_attempt = lambda status: runtime.attempt_record.update(status=status)
    timestamps = iter((100, 200))
    monkeypatch.setattr(
        "supervisor.paired_collection_controller.time.time_ns",
        lambda: next(timestamps),
    )

    runtime._run_base_and_start_skill("B0", "S1")

    output = capsys.readouterr().out
    assert output.index("[B0 COMPLETE]") < output.index("[WAIT]") < output.index("[S1 READY]")
    assert "看到 [S1 READY] 前不要开始抬臂" in output
    assert "现在可以开始采集 S1" in output
    assert runtime.machine.state == PairState.RECORDING_S1
    assert runtime.attempt_record["phase_markers"] == {
        "b0_complete_ns": 100,
        "s1_operator_ready_ns": 200,
    }
    assert runtime.attempt_record["status"] == "recording_s1"
    assert [event for event, _values in runtime.event_log.events] == [
        "b0_complete_waiting_for_s1_collector",
        "s1_operator_ready",
    ]


def test_b1_switches_segments_without_restarting_the_collector() -> None:
    class EventLog:
        def __init__(self) -> None:
            self.events: list[tuple[str, dict[str, object]]] = []

        def write(self, event: str, **values: object) -> None:
            self.events.append((event, values))

    runtime = object.__new__(PairedCollectionRuntime)
    process = SimpleNamespace(pid=4242)
    runtime.collector_process = process
    runtime.current_skill = "S1"
    runtime.collector_episode_path = Path("/tmp/continuous_pair")
    runtime.collector_recording_start_ns = 100
    runtime.collector_started_at = 0.0
    runtime.skills = {"S2": {"prompt_version": "v3"}}
    runtime.machine = PairStateMachine()
    runtime.machine.press("right_a")
    runtime.machine.base_complete("B0")
    runtime.machine.collector_started("S1")
    runtime.machine.press("right_a")
    runtime.machine.press("right_b")
    runtime.machine_lock = threading.Lock()
    runtime.attempt_record = {
        "episodes": {"S1": {"status": "recording"}},
        "phase_markers": {},
    }
    runtime.event_log = EventLog()
    runtime._run_prior = lambda stage: None
    runtime._write_attempt = lambda: None
    runtime._update_attempt = lambda status: runtime.attempt_record.update(status=status)

    def must_not_restart(*_args, **_kwargs) -> None:
        raise AssertionError("B1-to-S2 transition must not stop or start the collector")

    runtime._stop_collector = must_not_restart
    runtime._start_collector = must_not_restart

    runtime._run_b1_inside_continuous_episode_and_switch_to_s2()

    assert runtime.collector_process is process
    assert runtime.machine.state == PairState.RECORDING_S2
    assert runtime.attempt_record["status"] == "recording_s2"
    assert runtime.attempt_record["episodes"]["S1"]["status"] == "segment_complete"
    assert runtime.attempt_record["episodes"]["S2"]["status"] == "recording"
    assert runtime.attempt_record["episodes"]["S2"]["path"] == "/tmp/continuous_pair"
    transition = [event for event in runtime.event_log.events if event[0] == "continuous_skill_transition"]
    assert len(transition) == 1
    assert transition[0][1]["collector_pid"] == 4242
    assert transition[0][1]["camera_restart"] is False


def test_continuous_source_writes_explicit_s1_s2_frame_ranges(tmp_path: Path) -> None:
    source = tmp_path / "continuous_pair"
    source.mkdir()
    (source / "manifest.json").write_text(
        json.dumps(
            {
                "status": "complete",
                "recording_start_ns": 100,
                "recording_end_ns": 900,
            }
        )
    )
    with (source / "frames.jsonl").open("w", encoding="utf-8") as stream:
        for index, timestamp_ns in enumerate(range(100, 901, 100)):
            stream.write(json.dumps({"frame_index": index, "timestamp_ns": timestamp_ns}) + "\n")

    runtime = object.__new__(PairedCollectionRuntime)
    runtime.attempt_id = "pair_test"
    runtime.attempt_path = None
    runtime.attempt_record = {
        "phase_markers": {"s2_start_ns": 500, "s2_complete_ns": 850},
        "episodes": {"S1": {}, "S2": {}},
    }
    runtime.config = {"timing": {"minimum_skill_recording_sec": 0.2, "hz": 10}}
    runtime.skills = {
        "S1": {"prompt": "s1 task", "prompt_version": "v2"},
        "S2": {"prompt": "s2 task", "prompt_version": "v3"},
    }
    runtime.event_log = SimpleNamespace(write=lambda *_args, **_kwargs: None)

    runtime._finalize_continuous_segments(source, 850)

    segments = json.loads((source / "paired_segments.json").read_text())
    assert segments["camera_pipeline_continuous"] is True
    assert segments["camera_restart_count_between_segments"] == 0
    assert segments["segments"] == [
        {
            "skill_id": "S1",
            "task": "s1 task",
            "prompt_version": "v2",
            "start_ns": 100,
            "end_ns": 500,
            "start_inclusive": True,
            "end_inclusive": False,
            "source_start_frame": 0,
            "source_end_frame": 3,
            "frame_count": 4,
            "base_motion": {
                "allowed": True,
                "concurrent_prior": "B1",
                "included_in_training_action": False,
            },
        },
        {
            "skill_id": "S2",
            "task": "s2 task",
            "prompt_version": "v3",
            "start_ns": 500,
            "end_ns": 900,
            "start_inclusive": True,
            "end_inclusive": True,
            "source_start_frame": 4,
            "source_end_frame": 8,
            "frame_count": 5,
            "base_motion": {
                "allowed": False,
                "concurrent_prior": None,
                "included_in_training_action": False,
            },
        },
    ]
    manifest = json.loads((source / "manifest.json").read_text())
    assert manifest["training_eligible_as_atomic_episode"] is False
    assert manifest["paired_continuous_source"]["segments_file"] == "paired_segments.json"


def test_pair_reset_requires_normal_completion() -> None:
    machine = PairStateMachine()
    try:
        machine.reset_completed_pair()
    except RuntimeError as exc:
        assert "cannot reset pair from idle" in str(exc)
    else:
        raise AssertionError("idle pair reset must fail")


def test_invalid_buttons_do_not_skip_stages() -> None:
    machine = PairStateMachine()
    assert machine.press("right_b") is None
    assert machine.state == PairState.IDLE
    assert machine.press("right_a") == PairAction.RUN_B0
    assert machine.press("right_a") is None
    assert machine.press("right_b") is None
    assert machine.state == PairState.RUNNING_B0


def test_right_c_is_latched_emergency_from_every_state() -> None:
    for prepare in (False, True):
        machine = PairStateMachine()
        if prepare:
            machine.press("right_a")
            machine.base_complete("B0")
            machine.collector_started("S1")
        assert machine.press("right_c") == PairAction.EMERGENCY_STOP
        assert machine.state == PairState.ABORTED
        assert machine.press("right_a") is None
        assert machine.press("right_b") is None


def test_base_takeover_waits_for_runtime_graph_to_drop_f710() -> None:
    runtime = object.__new__(PairedCollectionRuntime)
    runtime.config = {"base_publisher_takeover": {"release_timeout_sec": 1.0}}
    attempts = 0

    def assert_exclusive() -> None:
        nonlocal attempts
        attempts += 1
        if attempts < 3:
            raise RuntimeError("stale my_controller_node endpoint")

    runtime._assert_no_base_publisher = assert_exclusive
    runtime._wait_for_exclusive_base_publisher()
    assert attempts == 3


def test_base_takeover_ignores_only_recorded_stale_endpoint_gid() -> None:
    managed = SimpleNamespace(node_name="my_controller_node", endpoint_gid=b"managed")
    own = SimpleNamespace(
        node_name="svt_paired_collection_controller", endpoint_gid=b"paired"
    )

    class FakeNode:
        infos = [managed, own]

        @staticmethod
        def get_name() -> str:
            return "svt_paired_collection_controller"

        def get_publishers_info_by_topic(self, _topic: str):
            return list(self.infos)

    runtime = object.__new__(PairedCollectionRuntime)
    runtime.node = FakeNode()
    runtime.managed_base_publisher_gids = set()
    runtime.config = {
        "ros": {"base_command_topic": "/svtrobot_cmd"},
        "base_publisher_takeover": {
            "managed_services": [{"publisher_node": "my_controller_node"}]
        },
        "safety": {"require_exclusive_base_publisher": True},
    }

    runtime._remember_managed_base_publishers()
    runtime._assert_no_base_publisher()

    runtime.node.infos.append(
        SimpleNamespace(node_name="my_controller_node", endpoint_gid=b"new-publisher")
    )
    try:
        runtime._assert_no_base_publisher()
    except RuntimeError as exc:
        assert "my_controller_node" in str(exc)
    else:
        raise AssertionError("a new publisher GID must not be treated as stale")


def test_b1_allows_left_arm_motion_and_arbitrary_joint_effort() -> None:
    runtime = object.__new__(PairedCollectionRuntime)
    runtime.emergency_event = threading.Event()
    initial = np.zeros(14, dtype=float)
    arm = np.zeros(14, dtype=float)
    effort = np.full(14, 100.0, dtype=float)
    arm[7:] = 1.0
    runtime._feedback_snapshot = lambda _stage: ((0.0, 0.0, 0.0), arm, effort)

    runtime._check_base_stage_safety("B0")
    runtime._check_base_stage_safety("B1")

    arm[0] = 0.5
    runtime._check_base_stage_safety("B0")
    runtime._check_base_stage_safety("B1")


def test_arm_feedback_freshness_is_ignored_for_b0_and_half_second_for_b1() -> None:
    runtime = object.__new__(PairedCollectionRuntime)
    runtime.feedback_lock = threading.Lock()
    runtime.base_feedback = (0.0, 0.0, 0.0)
    runtime.base_received_at = time.monotonic()
    runtime.arm_position = None
    runtime.arm_effort = None
    runtime.arm_received_at = 0.0
    runtime.config = {
        "timing": {
            "base_feedback_timeout_sec": 0.5,
            "joint_feedback_timeout_sec": 0.5,
        }
    }

    runtime._feedback_snapshot("B0")

    runtime.arm_position = np.zeros(14, dtype=float)
    runtime.arm_effort = np.zeros(14, dtype=float)
    runtime.arm_received_at = time.monotonic() - 0.3
    runtime._feedback_snapshot("B1")

    runtime.arm_received_at = time.monotonic() - 0.6
    try:
        runtime._feedback_snapshot("B1")
    except SafetyStop as exc:
        assert "arm feedback is unavailable or stale" in str(exc)
    else:
        raise AssertionError("B1 must reject arm feedback older than 0.5 seconds")
