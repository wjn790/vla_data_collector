from __future__ import annotations

import sys
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from supervisor.s3_collection_core import (
    S3CollectionAction,
    S3CollectionState,
    S3CollectionStateMachine,
    validate_config,
)


def config() -> dict:
    return yaml.safe_load((ROOT / "config" / "s3_collection.yaml").read_text())


def test_button_mapping_and_action3() -> None:
    value = config()
    mapping = validate_config(value)
    assert (mapping.right_a, mapping.right_b, mapping.right_c, mapping.right_d) == (
        11,
        12,
        13,
        14,
    )
    assert value["hand_controller"]["left_c_action_key"] == "3"


def test_complete_s3_with_one_b2_flow() -> None:
    machine = S3CollectionStateMachine()
    assert machine.press("right_a") == S3CollectionAction.START_S3
    machine.collector_started("S3")
    assert machine.state == S3CollectionState.RECORDING_S3
    assert machine.press("right_b") == S3CollectionAction.RUN_B2
    assert machine.state == S3CollectionState.RECORDING_S3
    machine.base_complete("B2")
    assert machine.state == S3CollectionState.RECORDING_S3
    assert machine.press("right_a") == S3CollectionAction.STOP_S3
    machine.collector_stopped("S3")
    assert machine.state == S3CollectionState.COMPLETE
    assert machine.press("right_b") == S3CollectionAction.B2_ALREADY_EXECUTED


def test_out_of_order_buttons_are_ignored() -> None:
    machine = S3CollectionStateMachine()
    assert machine.press("right_b") == S3CollectionAction.RUN_B2
    machine.base_complete("B2")
    assert machine.press("right_d") is None
    assert machine.state == S3CollectionState.IDLE
    machine.press("right_a")
    assert machine.press("right_a") is None
    assert machine.press("right_b") == S3CollectionAction.B2_ALREADY_EXECUTED


def test_b2_is_latched_before_replay_completes() -> None:
    machine = S3CollectionStateMachine()
    assert machine.press("right_b") == S3CollectionAction.RUN_B2
    assert machine.press("right_b") == S3CollectionAction.B2_ALREADY_EXECUTED
    machine.base_complete("B2")
    assert machine.press("right_b") == S3CollectionAction.B2_ALREADY_EXECUTED


def test_b2_can_run_after_s3_stops() -> None:
    machine = S3CollectionStateMachine()
    machine.press("right_a")
    machine.collector_started("S3")
    machine.press("right_a")
    machine.collector_stopped("S3")
    assert machine.state == S3CollectionState.COMPLETE
    assert machine.press("right_b") == S3CollectionAction.RUN_B2
    machine.base_complete("B2")
    assert machine.state == S3CollectionState.COMPLETE


@pytest.mark.parametrize("state", list(S3CollectionState))
def test_right_c_is_emergency_from_every_state(state: S3CollectionState) -> None:
    machine = S3CollectionStateMachine()
    machine.state = state
    assert machine.press("right_c") == S3CollectionAction.EMERGENCY_STOP
    assert machine.state == S3CollectionState.ABORTED
