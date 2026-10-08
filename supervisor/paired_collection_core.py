from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any


class PairState(str, Enum):
    IDLE = "idle"
    RUNNING_B0 = "running_b0"
    STARTING_S1 = "starting_s1"
    RECORDING_S1 = "recording_s1"
    AWAITING_B1 = "awaiting_b1_while_recording_s1"
    RUNNING_B1 = "running_b1"
    SWITCHING_TO_S2 = "switching_to_s2"
    RECORDING_S2 = "recording_s2"
    STOPPING_S2 = "stopping_s2"
    COMPLETE = "complete"
    ABORTED = "aborted"


class PairAction(str, Enum):
    RUN_B0 = "run_b0"
    MARK_S1_GRASP_COMPLETE = "mark_s1_grasp_complete"
    RUN_B1 = "run_b1"
    STOP_S2 = "stop_s2"
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
        raise ValueError("paired collection config must define buttons")
    mapping = ButtonMapping(
        right_a=int(buttons["right_a"]),
        right_b=int(buttons["right_b"]),
        right_c=int(buttons["right_c_emergency"]),
        right_d=int(buttons["right_d_reserved"]),
    )
    values = (mapping.right_a, mapping.right_b, mapping.right_c, mapping.right_d)
    if any(value < 0 for value in values) or len(set(values)) != len(values):
        raise ValueError(f"paired collection button indices must be unique and nonnegative: {values}")
    if values != (11, 12, 13, 14):
        raise ValueError(
            "paired collection must use right A/B/C/D indices 11/12/13/14 for this exoskeleton"
        )
    hand = config.get("hand_controller")
    if not isinstance(hand, dict) or hand.get("enabled") is not True:
        raise ValueError("paired collection must enable its embedded hand controller")
    if int(hand.get("left_c_button_index", -1)) != 7:
        raise ValueError("embedded hand controller must use left C buttons[7]")
    if str(hand.get("left_c_action_key", "")) != "4":
        raise ValueError("embedded hand controller must map left C to O6 action 4")
    takeover = config.get("base_publisher_takeover")
    if not isinstance(takeover, dict) or takeover.get("enabled") is not True:
        raise ValueError("paired collection must enable managed base publisher takeover")
    if str(takeover.get("scope")) != "first_right_a_through_s2_completion":
        raise ValueError("base publisher takeover must cover the complete paired attempt")
    services = takeover.get("managed_services")
    if not isinstance(services, list) or len(services) != 1:
        raise ValueError("paired collection must manage exactly one F710 service")
    service = services[0]
    expected_service = {
        "unit": "svtrobo-f710.service",
        "publisher_node": "my_controller_node",
        "enable_topic": "/f710/enable",
        "status_topic": "/f710/status",
    }
    if not isinstance(service, dict) or any(
        str(service.get(key)) != value for key, value in expected_service.items()
    ):
        raise ValueError(f"unexpected base publisher takeover service: {service}")
    for key in (
        "release_timeout_sec",
        "service_stop_timeout_sec",
        "service_start_timeout_sec",
        "status_timeout_sec",
    ):
        if float(takeover.get(key, 0)) <= 0:
            raise ValueError(f"base_publisher_takeover.{key} must be positive")
    timing = config.get("timing")
    if not isinstance(timing, dict):
        raise ValueError("paired collection config must define timing")
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


class PairStateMachine:
    def __init__(self) -> None:
        self.state = PairState.IDLE

    def press(self, button: str) -> PairAction | None:
        if button == "right_c":
            self.state = PairState.ABORTED
            return PairAction.EMERGENCY_STOP
        if self.state in {PairState.COMPLETE, PairState.ABORTED}:
            return None
        if button == "right_a":
            if self.state == PairState.IDLE:
                self.state = PairState.RUNNING_B0
                return PairAction.RUN_B0
            if self.state == PairState.RECORDING_S1:
                self.state = PairState.AWAITING_B1
                return PairAction.MARK_S1_GRASP_COMPLETE
            return None
        if button == "right_b":
            if self.state == PairState.AWAITING_B1:
                self.state = PairState.RUNNING_B1
                return PairAction.RUN_B1
            if self.state == PairState.RECORDING_S2:
                self.state = PairState.STOPPING_S2
                return PairAction.STOP_S2
            return None
        return None

    def base_complete(self, stage: str) -> None:
        expected = PairState.RUNNING_B0 if stage == "B0" else PairState.RUNNING_B1
        if self.state != expected:
            raise RuntimeError(f"cannot complete {stage} from {self.state.value}")
        self.state = PairState.STARTING_S1 if stage == "B0" else PairState.SWITCHING_TO_S2

    def collector_started(self, skill: str) -> None:
        if skill != "S1" or self.state != PairState.STARTING_S1:
            raise RuntimeError(f"cannot start {skill} collector from {self.state.value}")
        self.state = PairState.RECORDING_S1

    def switch_to_s2(self) -> None:
        if self.state != PairState.SWITCHING_TO_S2:
            raise RuntimeError(f"cannot switch to S2 from {self.state.value}")
        self.state = PairState.RECORDING_S2

    def collector_stopped(self, skill: str) -> None:
        if skill != "S2" or self.state != PairState.STOPPING_S2:
            raise RuntimeError(f"cannot stop {skill} collector from {self.state.value}")
        self.state = PairState.COMPLETE

    def abort(self) -> None:
        self.state = PairState.ABORTED

    def reset_completed_pair(self) -> None:
        if self.state != PairState.COMPLETE:
            raise RuntimeError(f"cannot reset pair from {self.state.value}")
        self.state = PairState.IDLE
