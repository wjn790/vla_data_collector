from __future__ import annotations

from argparse import Namespace
from pathlib import Path

import pytest

from collect_episode import resolve_collection_metadata
from validate_episode import camera_delta_allowed


def make_args(tmp_path: Path, *, task: str | None = None, mode: str = "button_primitive") -> Namespace:
    skills = tmp_path / "skills.yaml"
    skills.write_text(
        """schema_version: 1
skills:
  S1:
    prompt: grasp the cabinet door handle with the left hand
    prompt_version: v1
    collection_hand_mode: button_primitive
    allowed_executors: [left_arm, left_hand]
    standard_end_state: grasped
""",
        encoding="utf-8",
    )
    hand = tmp_path / "hand.json"
    hand.write_text("{}\n", encoding="utf-8")
    return Namespace(
        task=task,
        skill_id="S1",
        skills_config=skills,
        prompt_version=None,
        attempt_outcome="unreviewed",
        perturbation="standard",
        hand_control_mode=mode,
        operator="tester",
        hand_config=hand,
        allow_base_motion=False,
    )


def test_versioned_skill_derives_prompt_and_checksums(tmp_path: Path) -> None:
    config = tmp_path / "svt.yaml"
    config.write_text("schema_version: 1\n", encoding="utf-8")
    task, metadata = resolve_collection_metadata(make_args(tmp_path), config)
    assert task == "grasp the cabinet door handle with the left hand"
    assert metadata["skill_id"] == "S1"
    assert metadata["prompt_version"] == "v1"
    assert set(metadata["config_checksums"]) == {
        "collector_config",
        "skills_config",
        "hand_config",
    }


def test_prompt_or_hand_mode_mismatch_is_rejected(tmp_path: Path) -> None:
    config = tmp_path / "svt.yaml"
    config.write_text("schema_version: 1\n", encoding="utf-8")
    with pytest.raises(ValueError, match="does not exactly match"):
        resolve_collection_metadata(make_args(tmp_path, task="wrong"), config)
    with pytest.raises(ValueError, match="requires --hand-control-mode"):
        resolve_collection_metadata(make_args(tmp_path, mode="full_glove"), config)


def test_base_motion_requires_versioned_concurrent_prior(tmp_path: Path) -> None:
    config = tmp_path / "svt.yaml"
    config.write_text("schema_version: 1\n", encoding="utf-8")
    args = make_args(tmp_path)
    with pytest.raises(ValueError, match="does not authorize"):
        args.allow_base_motion = True
        resolve_collection_metadata(args, config)

    skills = args.skills_config.read_text(encoding="utf-8")
    args.skills_config.write_text(
        skills.replace(
            "    prompt_version: v1\n",
            "    prompt_version: v1\n    concurrent_base_prior: B1\n",
        ),
        encoding="utf-8",
    )
    _task, metadata = resolve_collection_metadata(args, config)
    assert metadata["base_motion"] == {
        "allowed": True,
        "concurrent_prior": "B1",
        "included_in_training_action": False,
    }


def test_validator_accepts_only_provenance_bounded_camera_repairs() -> None:
    policy = {"camera_max_delta_ms": 75, "timing_camera_max_delta_ms": 90}
    isolated = {
        "camera:camera_top": {"method": "accept_isolated_nearest_raw_image"}
    }
    timing = {
        "timing": {"method": "missing_timestep_interpolation_nearest_raw_images"}
    }

    assert camera_delta_allowed("camera_top", 60.0, 30.0, isolated, policy)
    assert camera_delta_allowed("camera_wrist_right", -80.8, 30.0, timing, policy)
    assert not camera_delta_allowed("camera_top", 76.0, 30.0, isolated, policy)
    assert not camera_delta_allowed("camera_top", 60.0, 30.0, {}, policy)
    assert not camera_delta_allowed(
        "camera_top",
        60.0,
        30.0,
        {"camera:camera_top": {"method": "unknown"}},
        policy,
    )
