#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any


COLLECTOR_ROOT = Path(__file__).resolve().parents[1]
if str(COLLECTOR_ROOT) not in sys.path:
    sys.path.insert(0, str(COLLECTOR_ROOT))

from supervisor.core import load_yaml
from supervisor.policy_client import PolicyClient


EXPECTED_POLICY_TYPE = "pi05"
EXPECTED_STATE_DIM = 26
EXPECTED_ACTION_DIM = 26
EXPECTED_ACTION_KEYS = {"action.arm.position", "action.hand.position"}
EXPECTED_OBSERVATION_KEYS = {
    "observation.state",
    "observation.images.camera_top",
    "observation.images.camera_wrist_left",
    "observation.images.camera_wrist_right",
    "task",
}


def _launcher_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "--config",
        type=Path,
        default=COLLECTOR_ROOT / "config" / "inference_pi05_b0_s1_b1_s2.yaml",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--execute", action="store_true")
    mode.add_argument("--shadow", action="store_true")
    args, _ = parser.parse_known_args(argv)
    return args


def validate_pi05_config(config: dict[str, Any]) -> None:
    policy = config.get("policy")
    if not isinstance(policy, dict):
        raise ValueError("PI0.5 inference config must define policy")
    if policy.get("policy_type") != EXPECTED_POLICY_TYPE:
        raise ValueError("PI0.5 executor requires policy.policy_type: pi05")
    if policy.get("protocol_version") != "svt-pi05-policy-v1":
        raise ValueError("PI0.5 executor requires protocol svt-pi05-policy-v1")
    if policy.get("policy_type") != EXPECTED_POLICY_TYPE:
        raise ValueError("PI0.5 executor requires policy_type: pi05")
    if not str(policy.get("expected_model_path", "")).startswith("/"):
        raise ValueError("policy.expected_model_path must be an absolute WJN path")
    if int(policy.get("expected_checkpoint_step", -1)) <= 0:
        raise ValueError("policy.expected_checkpoint_step must be positive")
    hash_fields = [name for name in ("checkpoint_index_sha256", "checkpoint_sha256") if policy.get(name)]
    if len(hash_fields) != 1:
        raise ValueError("PI0.5 policy must pin exactly one checkpoint hash")
    feature_order = policy.get("expected_action_feature_names")
    if not isinstance(feature_order, list) or len(feature_order) != EXPECTED_ACTION_DIM:
        raise ValueError("policy.expected_action_feature_names must contain 26 entries")
    if len(set(map(str, feature_order))) != EXPECTED_ACTION_DIM:
        raise ValueError("policy.expected_action_feature_names contains duplicates")


def validate_pi05_metadata(metadata: dict[str, Any], config: dict[str, Any]) -> None:
    policy = config["policy"]
    errors: list[str] = []

    expected_values = {
        "protocol_version": policy["protocol_version"],
        "policy_type": EXPECTED_POLICY_TYPE,
        "robot_config": policy["robot_config"],
        "model_path": policy["expected_model_path"],
        "checkpoint_step": int(policy["expected_checkpoint_step"]),
        "chunk_size": int(config["control"]["chunk_size"]),
        "state_dim": EXPECTED_STATE_DIM,
        "action_dim": EXPECTED_ACTION_DIM,
    }
    for key, expected in expected_values.items():
        if metadata.get(key) != expected:
            errors.append(f"{key}: expected {expected!r}, got {metadata.get(key)!r}")

    checkpoint_hash_field = next(
        name for name in ("checkpoint_index_sha256", "checkpoint_sha256") if policy.get(name)
    )
    for key in (
        "deployment_contract_id",
        "model_path",
        "normalization_sha256",
        "robot_config_sha256",
        "training_config_sha256",
        checkpoint_hash_field,
    ):
        expected = policy.get(key)
        if not isinstance(expected, str) or metadata.get(key) != expected:
            errors.append(f"{key}: expected {expected!r}, got {metadata.get(key)!r}")

    if set(metadata.get("action_keys", ())) != EXPECTED_ACTION_KEYS:
        errors.append(f"action_keys: got {metadata.get('action_keys')!r}")
    if set(metadata.get("observation_keys", ())) != EXPECTED_OBSERVATION_KEYS:
        errors.append(f"observation_keys: got {metadata.get('observation_keys')!r}")
    if list(metadata.get("action_feature_names", ())) != list(
        policy["expected_action_feature_names"]
    ):
        errors.append("action_feature_names does not match the trained 26-D ordering")

    if errors:
        raise RuntimeError("PI0.5 policy server is incompatible: " + "; ".join(errors))


def preflight_policy(config: dict[str, Any]) -> None:
    policy = config["policy"]
    client = PolicyClient(
        str(policy["host"]),
        int(policy["port"]),
        connect_timeout_sec=float(policy.get("startup_timeout_sec", 60.0)),
    )
    try:
        validate_pi05_metadata(client.metadata, config)
    finally:
        client.close()
    print(
        "[PI0.5 READY] compatible policy server: "
        f"{policy['host']}:{policy['port']} checkpoint="
        f"{policy['expected_checkpoint_step']}",
        flush=True,
    )


def main(argv: list[str] | None = None) -> int:
    args = _launcher_args(argv)
    config_path = args.config.expanduser().resolve()
    config = load_yaml(config_path)
    validate_pi05_config(config)
    if args.execute or args.shadow:
        preflight_policy(config)

    from supervisor import long_task_supervisor

    original_argv = sys.argv
    if argv is not None:
        sys.argv = [original_argv[0], *argv]
    try:
        return long_task_supervisor.main()
    finally:
        sys.argv = original_argv


if __name__ == "__main__":
    raise SystemExit(main())
