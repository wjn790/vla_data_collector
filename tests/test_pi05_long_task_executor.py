from __future__ import annotations

import copy
import sys
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from supervisor.pi05_long_task_executor import (
    validate_pi05_config,
    validate_pi05_metadata,
)


def load_config() -> dict:
    return yaml.safe_load(
        (ROOT / "config" / "inference_pi05_b0_s1_b1_s2.yaml").read_text(
            encoding="utf-8"
        )
    )


def valid_metadata(config: dict) -> dict:
    policy = config["policy"]
    return {
        "protocol_version": policy["protocol_version"],
        "policy_type": "pi05",
        "robot_config": policy["robot_config"],
        "model_path": policy["expected_model_path"],
        "checkpoint_step": policy["expected_checkpoint_step"],
        "chunk_size": config["control"]["chunk_size"],
        "state_dim": 26,
        "action_dim": 26,
        "action_keys": ["action.arm.position", "action.hand.position"],
        "observation_keys": [
            "observation.state",
            "observation.images.camera_top",
            "observation.images.camera_wrist_left",
            "observation.images.camera_wrist_right",
            "task",
        ],
        "action_feature_names": policy["expected_action_feature_names"],
    }


def test_pi05_config_and_metadata_match() -> None:
    config = load_config()
    validate_pi05_config(config)
    validate_pi05_metadata(valid_metadata(config), config)


@pytest.mark.parametrize(
    ("key", "value"),
    [
        ("checkpoint_step", 15000),
        ("state_dim", 14),
        ("action_dim", 32),
        ("protocol_version", "svt-policy-v1"),
    ],
)
def test_pi05_metadata_rejects_incompatible_server(key: str, value: object) -> None:
    config = load_config()
    metadata = valid_metadata(config)
    metadata[key] = value
    with pytest.raises(RuntimeError, match=key):
        validate_pi05_metadata(metadata, config)


def test_pi05_metadata_rejects_feature_reordering() -> None:
    config = load_config()
    metadata = valid_metadata(config)
    metadata["action_feature_names"] = copy.copy(metadata["action_feature_names"])
    metadata["action_feature_names"][0:2] = reversed(metadata["action_feature_names"][0:2])
    with pytest.raises(RuntimeError, match="action_feature_names"):
        validate_pi05_metadata(metadata, config)
