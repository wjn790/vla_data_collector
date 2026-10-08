from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, Iterable


WORKFLOW = "continuous_s4_s5_s6_single_collector_v1"
SEGMENTS_FILE = "paired_segments.json"
SUPPORTED_HAND_MODES = ("full_glove", "s5_index_only")
DEFAULT_HAND_MODE = "full_glove"


class S456State(str, Enum):
    IDLE = "idle"
    RECORDING_S4 = "recording_s4"
    RECORDING_S5 = "recording_s5"
    RECORDING_S6 = "recording_s6"
    COMPLETE = "complete"
    ABORTED = "aborted"


class S456Action(str, Enum):
    START_RECORDING = "start_recording"
    MARK_S5_BOUNDARY = "mark_s5_boundary"
    MARK_S6_BOUNDARY = "mark_s6_boundary"
    STOP_RECORDING = "stop_recording"
    FORCE_MODE_SYNC = "force_mode_sync"
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
        raise ValueError("S456 collection config must define buttons")
    mapping = ButtonMapping(
        right_a=int(buttons["right_a"]),
        right_b=int(buttons["right_b"]),
        right_c=int(buttons["right_c_emergency"]),
        right_d=int(buttons["right_d"]),
    )
    values = (mapping.right_a, mapping.right_b, mapping.right_c, mapping.right_d)
    if values != (11, 12, 13, 14):
        raise ValueError("S456 collection requires right A/B/C/D indices 11/12/13/14")

    if config.get("hand_controller") is not None:
        raise ValueError(
            "S456 collection must not embed a hand button controller; the mixed "
            "glove teleop process owns the dual-hand command lease"
        )
    if not str(config.get("hand_mode_file", "")).strip():
        raise ValueError("S456 collection config must define hand_mode_file")

    takeover = config.get("base_publisher_takeover")
    if not isinstance(takeover, dict) or takeover.get("enabled") is not True:
        raise ValueError("S456 collection must enable managed base publisher takeover")
    if str(takeover.get("scope")) != "first_right_a_through_recording_end":
        raise ValueError(
            "S456 collection takeover must span first right A through recording end"
        )

    collection = config.get("collection")
    if not isinstance(collection, dict):
        raise ValueError("S456 collection config must define collection")
    if str(collection.get("hand_control_mode")) != DEFAULT_HAND_MODE:
        raise ValueError(
            f"S456 collection must record with hand_control_mode {DEFAULT_HAND_MODE!r}"
        )
    if collection.get("no_offline_align") is not True:
        raise ValueError("S456 collection must defer offline alignment to after the take")

    timing = config.get("timing")
    if not isinstance(timing, dict):
        raise ValueError("S456 collection config must define timing")
    for key in (
        "button_debounce_sec",
        "minimum_skill_recording_sec",
        "collector_start_timeout_sec",
        "collector_stop_timeout_sec",
        "base_feedback_timeout_sec",
    ):
        if float(timing.get(key, 0)) <= 0:
            raise ValueError(f"timing.{key} must be positive")
    return mapping


def hand_mode_for_state(state: S456State) -> str | None:
    if state in {S456State.RECORDING_S4, S456State.RECORDING_S6}:
        return DEFAULT_HAND_MODE
    if state == S456State.RECORDING_S5:
        return "s5_index_only"
    return None


class S456StateMachine:
    def __init__(self) -> None:
        self.state = S456State.IDLE

    def press(self, button: str) -> S456Action | None:
        if button == "right_c":
            self.state = S456State.ABORTED
            return S456Action.EMERGENCY_STOP
        if self.state in {S456State.COMPLETE, S456State.ABORTED}:
            return None
        if button == "right_a":
            if self.state == S456State.IDLE:
                self.state = S456State.RECORDING_S4
                return S456Action.START_RECORDING
            if self.state == S456State.RECORDING_S6:
                self.state = S456State.COMPLETE
                return S456Action.STOP_RECORDING
            return None
        if button == "right_b":
            if self.state == S456State.RECORDING_S4:
                self.state = S456State.RECORDING_S5
                return S456Action.MARK_S5_BOUNDARY
            if self.state == S456State.RECORDING_S5:
                self.state = S456State.RECORDING_S6
                return S456Action.MARK_S6_BOUNDARY
            return None
        if button == "right_d":
            if self.state in {
                S456State.RECORDING_S4,
                S456State.RECORDING_S5,
                S456State.RECORDING_S6,
            }:
                return S456Action.FORCE_MODE_SYNC
            return None
        return None

    def collector_started(self, skill: str) -> None:
        if skill != "S4" or self.state != S456State.RECORDING_S4:
            raise RuntimeError(f"cannot start {skill} collector from {self.state.value}")

    def collector_stopped(self, skill: str) -> None:
        if skill != "S4" or self.state != S456State.COMPLETE:
            raise RuntimeError(f"cannot stop {skill} collector from {self.state.value}")

    def abort(self) -> None:
        self.state = S456State.ABORTED

    def reset_completed_attempt(self) -> None:
        if self.state != S456State.COMPLETE:
            raise RuntimeError(f"cannot reset S456 from {self.state.value}")
        self.state = S456State.IDLE


def build_s456_segments_document(
    *,
    attempt_id: str,
    source_episode: str,
    manifest: dict[str, Any],
    frame_rows: Iterable[dict[str, Any]],
    phase_markers: dict[str, Any],
    s5_start_ns: int,
    s6_start_ns: int,
    skills: dict[str, dict[str, Any]],
    minimum_frames: int,
    mode_switch_events: Iterable[dict[str, Any]],
) -> dict[str, Any]:
    """Slice one continuous S456 recording into the S4/S5/S6 training segments.

    Frame assignment is by timestamp against the two operator markers; the
    segments must jointly cover every recorded frame exactly once.
    """
    recording_start_ns = int(manifest["recording_start_ns"])
    recording_end_ns = int(manifest["recording_end_ns"])
    if manifest.get("status") != "complete":
        raise RuntimeError("continuous source manifest is not complete")
    if not recording_start_ns < s5_start_ns < s6_start_ns < recording_end_ns:
        raise RuntimeError(
            "S456 boundaries are not strictly inside the recording: "
            f"{recording_start_ns} < {s5_start_ns} < {s6_start_ns} < {recording_end_ns}"
        )
    for skill_id in ("S4", "S5", "S6"):
        skill = skills.get(skill_id)
        if not isinstance(skill, dict) or not str(skill.get("prompt", "")).strip():
            raise RuntimeError(f"S456 collection requires the {skill_id} skill definition")

    ranges: dict[str, dict[str, Any]] = {
        "S4": {
            "start_ns": recording_start_ns,
            "end_ns": s5_start_ns,
            "start_inclusive": True,
            "end_inclusive": False,
            "frame_indices": [],
        },
        "S5": {
            "start_ns": s5_start_ns,
            "end_ns": s6_start_ns,
            "start_inclusive": True,
            "end_inclusive": False,
            "frame_indices": [],
        },
        "S6": {
            "start_ns": s6_start_ns,
            "end_ns": recording_end_ns,
            "start_inclusive": True,
            "end_inclusive": True,
            "frame_indices": [],
        },
    }
    for row in frame_rows:
        frame_index = int(row["frame_index"])
        timestamp_ns = int(row["timestamp_ns"])
        if timestamp_ns < s5_start_ns:
            skill_id = "S4"
        elif timestamp_ns < s6_start_ns:
            skill_id = "S5"
        else:
            skill_id = "S6"
        ranges[skill_id]["frame_indices"].append(frame_index)

    segments: list[dict[str, Any]] = []
    for skill_id in ("S4", "S5", "S6"):
        indices = ranges[skill_id].pop("frame_indices")
        if len(indices) < minimum_frames:
            raise RuntimeError(
                f"continuous {skill_id} segment has {len(indices)} frames; "
                f"minimum is {minimum_frames}"
            )
        if indices != list(range(indices[0], indices[-1] + 1)):
            raise RuntimeError(f"continuous {skill_id} segment is not contiguous")
        skill = skills[skill_id]
        segments.append(
            {
                "skill_id": skill_id,
                "task": str(skill["prompt"]),
                "prompt_version": str(skill["prompt_version"]),
                "hand_control_mode": str(skill.get("collection_hand_mode", DEFAULT_HAND_MODE)),
                **ranges[skill_id],
                "source_start_frame": indices[0],
                "source_end_frame": indices[-1],
                "frame_count": len(indices),
                "base_motion": {
                    "allowed": False,
                    "concurrent_prior": None,
                    "included_in_training_action": False,
                },
            }
        )

    return {
        "schema_version": 1,
        "workflow": WORKFLOW,
        "parent_attempt_id": attempt_id,
        "source_episode": str(source_episode),
        "source_frames": "frames.jsonl",
        "camera_pipeline_continuous": True,
        "camera_restart_count_between_segments": 0,
        "phase_markers": dict(phase_markers),
        "mode_switch_events": [dict(event) for event in mode_switch_events],
        "segments": segments,
    }
