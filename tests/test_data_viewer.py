from __future__ import annotations

import json
from pathlib import Path

import pytest

from data_viewer import DataRepository


def make_episode(root: Path) -> tuple[Path, str, str]:
    day = "2026-07-30"
    name = "120000_pair_test_continuous_s1_b1_s2"
    episode = root / day / name
    episode.mkdir(parents=True)
    task_s1 = "grasp and hold while the base retreats"
    task_s2 = "release and push with the back of the hand"
    manifest = {
        "status": "complete",
        "fps": 15,
        "frame_count": 4,
        "valid_frame_count": 4,
        "recording_start_ns": 1_000_000_000,
        "recording_end_ns": 1_200_000_000,
        "paired_continuous_source": {"segments_file": "paired_segments.json"},
    }
    (episode / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")
    (episode / "alignment_report.json").write_text(
        json.dumps(
            {
                "repair": {"repaired_frame_count": 1},
                "continuity": {
                    "complete_episode_candidate": True,
                    "internal_invalid_frame_count": 0,
                    "timing_gap_count": 0,
                },
            }
        ),
        encoding="utf-8",
    )
    segments = {
        "workflow": "continuous_s1_b1_s2_single_collector_v3",
        "phase_markers": {
            "s1_grasp_complete_ns": 1_050_000_000,
            "b1_start_ns": 1_080_000_000,
            "b1_complete_ns": 1_120_000_000,
            "s2_start_ns": 1_120_000_000,
        },
        "segments": [
            {
                "skill_id": "S1",
                "prompt_version": "v2",
                "task": task_s1,
                "source_start_frame": 0,
                "source_end_frame": 1,
                "frame_count": 2,
                "start_ns": 1_000_000_000,
                "end_ns": 1_120_000_000,
                "base_motion": {"allowed": True, "concurrent_prior": "B1"},
            },
            {
                "skill_id": "S2",
                "prompt_version": "v3",
                "task": task_s2,
                "source_start_frame": 2,
                "source_end_frame": 3,
                "frame_count": 2,
                "start_ns": 1_120_000_000,
                "end_ns": 1_200_000_000,
                "end_inclusive": True,
                "base_motion": {"allowed": False},
            },
        ],
    }
    (episode / "paired_segments.json").write_text(json.dumps(segments), encoding="utf-8")
    lines = []
    for index in range(4):
        skill = "S1" if index < 2 else "S2"
        prompt = task_s1 if index < 2 else task_s2
        camera_images = {}
        for camera in ("camera_top", "camera_wrist_left", "camera_wrist_right"):
            relative = f"raw/cameras/{camera}/{index}.jpg"
            image = episode / relative
            image.parent.mkdir(parents=True, exist_ok=True)
            image.write_bytes(b"jpeg")
            camera_images[camera] = {"path": relative}
        lines.append(
            json.dumps(
                {
                    "frame_index": index,
                    "timestamp_ns": 1_000_000_000 + index * 66_666_667,
                    "skill_id": skill,
                    "prompt_version": "v2" if skill == "S1" else "v3",
                    "task": prompt,
                    "paired_segment": {
                        "segment_index": 0 if skill == "S1" else 1,
                        "skill_id": skill,
                    },
                    "observation": {
                        "images": camera_images,
                        "state": {"arm_position": [0.0] * 14},
                    },
                    "action": {
                        "arm_position": [index * 0.01] * 14,
                        "hand_position": [0.0] * 12,
                        "base_velocity": [-0.05, 0.0, 0.0] if index == 1 else [0.0] * 3,
                    },
                    "validity": {
                        "valid_for_training": True,
                        "repaired": index == 1,
                        "repair_sources": ["hand"] if index == 1 else [],
                    },
                }
            )
        )
    (episode / "frames.jsonl").write_text("\n".join(lines) + "\n", encoding="utf-8")
    return episode, day, name


def test_repository_exposes_segments_frames_and_images(tmp_path: Path) -> None:
    _episode, day, name = make_episode(tmp_path)
    repository = DataRepository(tmp_path)

    index = repository.index()
    assert index["episodeCount"] == 1
    assert index["days"][0]["episodes"][0]["paired"] is True

    detail = repository.detail(day, name)
    assert [(value["skillId"], value["frameCount"]) for value in detail["segments"]] == [
        ("S1", 2),
        ("S2", 2),
    ]
    assert [frame["skillId"] for frame in detail["frames"]] == ["S1", "S1", "S2", "S2"]
    assert detail["frames"][1]["baseActive"] is True
    assert detail["frames"][1]["repaired"] is True
    assert detail["phaseMarkers"]["b1_start_ns"]["frame"] == 2
    assert repository.image_path(day, name, 0, "camera_top").read_bytes() == b"jpeg"


def test_repository_rejects_path_traversal(tmp_path: Path) -> None:
    _episode, day, name = make_episode(tmp_path)
    repository = DataRepository(tmp_path)

    with pytest.raises(ValueError):
        repository.episode_dir(day, "../escape")

    frames = repository.frames(repository.episode_dir(day, name))
    frames[0]["observation"]["images"]["camera_top"]["path"] = "../../outside.jpg"
    with pytest.raises(ValueError):
        repository.image_path(day, name, 0, "camera_top")
