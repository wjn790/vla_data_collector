from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any


class S3CollectionState(str, Enum):
    IDLE = "idle"
    STARTING_S3 = "starting_s3"
    RECORDING_S3 = "recording_s3"
    STOPPING_S3 = "stopping_s3"
    COMPLETE = "complete"
    ABORTED = "aborted"


class S3CollectionAction(str, Enum):
    START_S3 = "start_s3"
    RUN_B2 = "run_b2"
    STOP_S3 = "stop_s3"
    B2_ALREADY_EXECUTED = "b2_already_executed"
    EMERGENCY_STOP = "emergency_stop"


@dataclass(frozen=True)
class ButtonMapping:
    right_a: int
    right_b: int
    right_c: int
    right_d: int


def validate_config(config: dict[str, Any]) -> ButtonMapping:
    buttons = config.get("buttons")
    if not isinstance(buttons, dict):
        raise ValueError("S3 collection config must define buttons")
    mapping = ButtonMapping(
        right_a=int(buttons["right_a"]),
        right_b=int(buttons["right_b"]),
        right_c=int(buttons["right_c_emergency"]),
        right_d=int(buttons["right_d_reserved"]),
    )
    values = (mapping.right_a, mapping.right_b, mapping.right_c, mapping.right_d)
    if values != (11, 12, 13, 14):
        raise ValueError("S3 collection requires right A/B/C/D indices 11/12/13/14")

    hand = config.get("hand_controller")
    if not isinstance(hand, dict) or hand.get("enabled") is not True:
        raise ValueError("S3 collection must enable its embedded hand controller")
    if int(hand.get("left_c_button_index", -1)) != 7:
        raise ValueError("S3 collection must use left C buttons[7]")
    if str(hand.get("left_c_action_key", "")) != "3":
        raise ValueError("S3 collection must map left C to O6 action 3")

    takeover = config.get("base_publisher_takeover")
    if not isinstance(takeover, dict) or takeover.get("enabled") is not True:
        raise ValueError("S3 collection must enable managed base publisher takeover")
    if str(takeover.get("scope")) != "right_b_b2_only":
        raise ValueError("S3 collection takeover must be limited to the one-shot B2")

    timing = config.get("timing")
    if not isinstance(timing, dict):
        raise ValueError("S3 collection config must define timing")
    for key in (
        "button_debounce_sec",
        "post_base_stable_delay_sec",
        "minimum_skill_recording_sec",
        "collector_start_timeout_sec",
        "collector_stop_timeout_sec",
        "base_zero_stable_sec",
        "base_feedback_timeout_sec",
    ):
        if float(timing.get(key, 0)) <= 0:
            raise ValueError(f"timing.{key} must be positive")
    return mapping


class S3CollectionStateMachine:
    def __init__(self) -> None:
        self.state = S3CollectionState.IDLE
        self.b2_executed_once = False

    def press(self, button: str) -> S3CollectionAction | None:
        if button == "right_c":
            self.state = S3CollectionState.ABORTED
            return S3CollectionAction.EMERGENCY_STOP
        if button == "right_b":
            if self.b2_executed_once:
                return S3CollectionAction.B2_ALREADY_EXECUTED
            if self.state == S3CollectionState.ABORTED:
                return None
            self.b2_executed_once = True
            return S3CollectionAction.RUN_B2
        if self.state in {S3CollectionState.COMPLETE, S3CollectionState.ABORTED}:
            return None
        if button == "right_a":
            if self.state == S3CollectionState.IDLE:
                self.state = S3CollectionState.STARTING_S3
                return S3CollectionAction.START_S3
            if self.state == S3CollectionState.RECORDING_S3:
                self.state = S3CollectionState.STOPPING_S3
                return S3CollectionAction.STOP_S3
            return None
        return None

    def base_complete(self, stage: str) -> None:
        if stage == "B2" and self.b2_executed_once:
            return
        raise RuntimeError(f"cannot complete {stage} from {self.state.value}")

    def collector_started(self, skill: str) -> None:
        if skill != "S3" or self.state != S3CollectionState.STARTING_S3:
            raise RuntimeError(f"cannot start {skill} collector from {self.state.value}")
        self.state = S3CollectionState.RECORDING_S3

    def collector_stopped(self, skill: str) -> None:
        if skill != "S3" or self.state != S3CollectionState.STOPPING_S3:
            raise RuntimeError(f"cannot stop {skill} collector from {self.state.value}")
        self.state = S3CollectionState.COMPLETE

    def abort(self) -> None:
        self.state = S3CollectionState.ABORTED

    def reset_completed_s3(self) -> None:
        if self.state != S3CollectionState.COMPLETE:
            raise RuntimeError(f"cannot reset S3 from {self.state.value}")
        self.state = S3CollectionState.IDLE
