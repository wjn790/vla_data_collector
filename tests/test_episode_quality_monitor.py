from __future__ import annotations

import json
from pathlib import Path

from episode_quality_monitor import assess_start_capture, episode_is_selected


THRESHOLDS = {
    "pre_recording_left_arm_drift_max_rad": 0.05,
    "first_aligned_from_raw_initial_max_rad": 0.08,
    "ready_to_motion_min_sec": 0.25,
    "left_arm_motion_onset_rad": 0.05,
    "approach_motion_min_rad": 0.30,
}


def write_start_fixture(root: Path, *, pre_recording_value: float) -> tuple[dict, dict, list]:
    raw = root / "raw" / "ros"
    raw.mkdir(parents=True)
    records = [
        {"received_ns": 900_000_000, "value": [0.0] * 7},
        {"received_ns": 990_000_000, "value": [pre_recording_value] * 7},
    ]
    (raw / "arm_command_left.jsonl").write_text(
        "\n".join(json.dumps(value) for value in records) + "\n",
        encoding="utf-8",
    )
    manifest = {"recording_start_ns": 1_000_000_000}
    segments = {
        "phase_markers": {
            "s1_operator_ready_ns": 1_000_000_000,
            "s1_grasp_complete_ns": 1_666_666_670,
        }
    }
    frames = []
    for index in range(11):
        position = 0.0 if index < 5 else (index - 4) * 0.08
        frames.append(
            {
                "timestamp_ns": 1_000_000_000 + index * 66_666_667,
                "action": {"arm_position": [position] + [0.0] * 13},
            }
        )
    return manifest, segments, frames


def test_start_capture_accepts_idle_preroll_and_full_approach(tmp_path: Path) -> None:
    manifest, segments, frames = write_start_fixture(
        tmp_path, pre_recording_value=0.0
    )
    result = assess_start_capture(tmp_path, manifest, segments, frames, THRESHOLDS)

    assert result["status"] == "pass"
    assert result["issues"] == []
    assert result["ready_to_first_motion_sec"] >= 0.25
    assert result["approach_motion_rad"] >= 0.30


def test_start_capture_rejects_motion_before_recording(tmp_path: Path) -> None:
    manifest, segments, frames = write_start_fixture(
        tmp_path, pre_recording_value=0.20
    )
    result = assess_start_capture(tmp_path, manifest, segments, frames, THRESHOLDS)

    assert result["status"] == "failed"
    assert "left_arm_moved_before_recording" in result["issues"]


def test_selection_starts_at_configured_episode(tmp_path: Path) -> None:
    day = tmp_path / "2026-07-30"
    day.mkdir()
    config = {
        "selection": {
            "minimum_day": "2026-07-30",
            "minimum_episode_name": "145241_pair_continuous_s1_b1_s2",
            "required_name_suffix": "continuous_s1_b1_s2",
        }
    }

    assert not episode_is_selected(day / "145015_pair_continuous_s1_b1_s2", config)
    assert episode_is_selected(day / "145241_pair_continuous_s1_b1_s2", config)
    assert episode_is_selected(day / "150119_pair_continuous_s1_b1_s2", config)
