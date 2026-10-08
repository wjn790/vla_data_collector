#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import os
from pathlib import Path
from typing import Any

import yaml

from common import atomic_write_json
from offline_align import align_episode


def merge_alignment_policy(
    episode_config: dict[str, Any], policy_config: dict[str, Any]
) -> dict[str, Any]:
    """Apply the deployed offline policy without changing the episode snapshot."""
    merged = dict(episode_config)
    historical_alignment = episode_config.get("offline_alignment") or {}
    deployed_alignment = policy_config.get("offline_alignment") or {}
    if not isinstance(historical_alignment, dict):
        raise ValueError("episode offline_alignment must be a mapping")
    if not isinstance(deployed_alignment, dict) or not deployed_alignment:
        raise ValueError("policy config must define offline_alignment")

    alignment = dict(historical_alignment)
    for key, value in deployed_alignment.items():
        if isinstance(value, dict) and isinstance(alignment.get(key), dict):
            alignment[key] = {**alignment[key], **value}
        else:
            alignment[key] = value
    merged["offline_alignment"] = alignment
    return merged


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = [
        "episode",
        "result",
        "manifest_status",
        "quality_class",
        "hand_state_mode",
        "original_frame_count",
        "aligned_frame_count",
        "strict_valid_frame_count",
        "available_hand_aligned_frame_count",
        "repaired_frame_count",
        "invalid_frame_count",
        "leading_invalid_frame_count",
        "trailing_invalid_frame_count",
        "internal_invalid_frame_count",
        "complete_episode_candidate",
        "camera_top_p90_ms",
        "camera_top_max_ms",
        "wrist_left_p90_ms",
        "wrist_left_max_ms",
        "wrist_right_p90_ms",
        "wrist_right_max_ms",
        "arm_joint_state_p90_ms",
        "arm_joint_state_max_ms",
        "arm_command_left_p90_ms",
        "arm_command_left_max_ms",
        "arm_command_right_p90_ms",
        "arm_command_right_max_ms",
        "hand_action_p90_ms",
        "hand_action_max_ms",
        "hand_state_p90_ms",
        "hand_state_max_ms",
        "base_feedback_p90_ms",
        "base_feedback_max_ms",
        "base_joint_state_p90_ms",
        "base_joint_state_max_ms",
        "invalid_reason_counts",
        "error",
    ]
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            value = dict(row)
            value["invalid_reason_counts"] = json.dumps(
                value.get("invalid_reason_counts") or {}, ensure_ascii=False, sort_keys=True
            )
            writer.writerow({field: value.get(field) for field in fields})
    os.replace(temporary, path)


def write_markdown(path: Path, summary: dict[str, Any]) -> None:
    totals = summary["totals"]
    lines = [
        "# SVT Realignment Report",
        "",
        f"- Generated: {summary['generated_at']}",
        f"- Dataset: `{summary['dataset_dir']}`",
        f"- Episodes: {totals['episode_count']}",
        f"- Aligned: {totals['aligned_count']}",
        f"- Failed: {totals['failed_count']}",
        f"- Strict hardware-state episodes: {totals['strict_hardware_episode_count']}",
        f"- Legacy command-fallback episodes: {totals['legacy_fallback_episode_count']}",
        f"- Strict hardware frames: {totals['strict_valid_frame_count']}/"
        f"{totals['strict_hardware_frame_count']} valid "
        f"({totals['strict_valid_rate_percent']:.2f}%)",
        f"- Frames aligned with available hand source: "
        f"{totals['available_hand_aligned_frame_count']}/"
        f"{totals['aligned_frame_count']} "
        f"({totals['available_hand_aligned_rate_percent']:.2f}%)",
        f"- Repaired numeric-gap frames: {totals['repaired_frame_count']}",
        "",
        "| Episode | Result | Quality | Frames | Strict valid | Available aligned | Repaired | Invalid reasons |",
        "|---|---|---|---:|---:|---:|---:|---|",
    ]
    for row in summary["episodes"]:
        reasons = ", ".join(
            f"{key}={value}"
            for key, value in sorted((row.get("invalid_reason_counts") or {}).items())
        )
        lines.append(
            f"| {row.get('episode', '')} | {row.get('result', '')} | "
            f"{row.get('quality_class', '')} | {row.get('aligned_frame_count', '')} | "
            f"{row.get('strict_valid_frame_count', '')} | "
            f"{row.get('available_hand_aligned_frame_count', '')} | "
            f"{row.get('repaired_frame_count', '')} | {reasons or '-'} |"
        )
    lines.extend(
        [
            "",
            "## Source Timing P90 (ms)",
            "",
            "Absolute nearest-source deltas. The left wrist is the master timeline, so its delta is 0 ms.",
            "",
            "| Episode | ZED top | Right wrist | Arm state | Left arm cmd | Right arm cmd | Hand action | Hand state | Base feedback |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for row in summary["episodes"]:
        def value(name: str) -> str:
            item = row.get(name)
            return "-" if item is None else f"{float(item):.2f}"

        lines.append(
            f"| {row.get('episode', '')} | {value('camera_top_p90_ms')} | "
            f"{value('wrist_right_p90_ms')} | {value('arm_joint_state_p90_ms')} | "
            f"{value('arm_command_left_p90_ms')} | "
            f"{value('arm_command_right_p90_ms')} | {value('hand_action_p90_ms')} | "
            f"{value('hand_state_p90_ms')} | {value('base_feedback_p90_ms')} |"
        )
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text("\n".join(lines) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def realign_day(
    dataset_dir: Path, policy_config_path: Path | None = None
) -> dict[str, Any]:
    dataset_dir = dataset_dir.resolve()
    if policy_config_path is None:
        policy_config_path = Path(__file__).resolve().parent / "config" / "svt.yaml"
    policy_config = yaml.safe_load(policy_config_path.read_text(encoding="utf-8"))
    if not isinstance(policy_config, dict):
        raise ValueError(f"invalid policy config: {policy_config_path}")

    rows: list[dict[str, Any]] = []
    for episode in sorted(path for path in dataset_dir.iterdir() if path.is_dir()):
        manifest_path = episode / "manifest.json"
        config_path = episode / "config.yaml"
        raw_path = episode / "raw"
        if not manifest_path.is_file() or not config_path.is_file() or not raw_path.is_dir():
            continue
        try:
            original_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        except Exception as exc:
            rows.append(
                {
                    "episode": episode.name,
                    "result": "failed",
                    "quality_class": "unreadable_manifest",
                    "error": f"{type(exc).__name__}: {exc}",
                }
            )
            continue

        row: dict[str, Any] = {
            "episode": episode.name,
            "manifest_status": original_manifest.get("status"),
            "original_frame_count": original_manifest.get("frame_count"),
        }
        try:
            episode_config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
            if not isinstance(episode_config, dict):
                raise ValueError(f"invalid episode config: {config_path}")
            alignment_config = merge_alignment_policy(episode_config, policy_config)
            report = align_episode(episode, alignment_config)
            hand = report.get("hand_state") or {}
            continuity = report.get("continuity") or {}
            sources = report.get("source_alignment") or {}
            top = sources.get("camera_top") or {}
            wrist_left = sources.get("camera_wrist_left") or {}
            wrist = sources.get("camera_wrist_right") or {}
            arm_state = sources.get("arm_joint_state") or {}
            arm_left = sources.get("arm_command_left") or {}
            arm_right = sources.get("arm_command_right") or {}
            hand_action = sources.get("hand_action") or {}
            hand_state = sources.get("hand_state") or {}
            base_feedback = sources.get("base_feedback") or {}
            base_state = sources.get("base_joint_state") or {}
            strict_hand = hand.get("mode") == "hardware_feedback"
            valid = int(report.get("valid_frame_count", 0))
            frames = int(report.get("frame_count", 0))
            quality = "strict_ready"
            if not strict_hand:
                quality = "legacy_command_fallback"
            elif valid < frames:
                quality = "strict_partial"
            row.update(
                {
                    "result": "aligned",
                    "quality_class": quality,
                    "hand_state_mode": hand.get("mode"),
                    "aligned_frame_count": frames,
                    "strict_valid_frame_count": valid,
                    "available_hand_aligned_frame_count": report.get(
                        "available_hand_aligned_frame_count"
                    ),
                    "repaired_frame_count": (report.get("repair") or {}).get(
                        "repaired_frame_count", 0
                    ),
                    "invalid_frame_count": report.get("invalid_frame_count"),
                    "invalid_reason_counts": report.get("invalid_reason_counts") or {},
                    "leading_invalid_frame_count": continuity.get(
                        "leading_invalid_frame_count"
                    ),
                    "trailing_invalid_frame_count": continuity.get(
                        "trailing_invalid_frame_count"
                    ),
                    "internal_invalid_frame_count": continuity.get(
                        "internal_invalid_frame_count"
                    ),
                    "complete_episode_candidate": continuity.get(
                        "complete_episode_candidate"
                    ),
                    "camera_top_p90_ms": top.get("p90_ms"),
                    "camera_top_max_ms": top.get("max_ms"),
                    "wrist_left_p90_ms": wrist_left.get("p90_ms"),
                    "wrist_left_max_ms": wrist_left.get("max_ms"),
                    "wrist_right_p90_ms": wrist.get("p90_ms"),
                    "wrist_right_max_ms": wrist.get("max_ms"),
                    "arm_joint_state_p90_ms": arm_state.get("p90_ms"),
                    "arm_joint_state_max_ms": arm_state.get("max_ms"),
                    "arm_command_left_p90_ms": arm_left.get("p90_ms"),
                    "arm_command_left_max_ms": arm_left.get("max_ms"),
                    "arm_command_right_p90_ms": arm_right.get("p90_ms"),
                    "arm_command_right_max_ms": arm_right.get("max_ms"),
                    "hand_action_p90_ms": hand_action.get("p90_ms"),
                    "hand_action_max_ms": hand_action.get("max_ms"),
                    "hand_state_p90_ms": hand_state.get("p90_ms"),
                    "hand_state_max_ms": hand_state.get("max_ms"),
                    "base_feedback_p90_ms": base_feedback.get("p90_ms"),
                    "base_feedback_max_ms": base_feedback.get("max_ms"),
                    "base_joint_state_p90_ms": base_state.get("p90_ms"),
                    "base_joint_state_max_ms": base_state.get("max_ms"),
                    "error": None,
                }
            )
        except Exception as exc:
            row.update(
                {
                    "result": "failed",
                    "quality_class": "alignment_failed",
                    "error": f"{type(exc).__name__}: {exc}",
                }
            )
        rows.append(row)
        print(
            f"{row['episode']}: {row['result']} "
            f"quality={row.get('quality_class')} "
            f"valid={row.get('strict_valid_frame_count')}/"
            f"{row.get('aligned_frame_count')}",
            flush=True,
        )

    totals = {
        "episode_count": len(rows),
        "aligned_count": sum(row.get("result") == "aligned" for row in rows),
        "failed_count": sum(row.get("result") != "aligned" for row in rows),
        "strict_hardware_episode_count": sum(
            row.get("hand_state_mode") == "hardware_feedback" for row in rows
        ),
        "legacy_fallback_episode_count": sum(
            row.get("hand_state_mode") == "command_fallback_unverified" for row in rows
        ),
        "aligned_frame_count": sum(int(row.get("aligned_frame_count") or 0) for row in rows),
        "strict_valid_frame_count": sum(
            int(row.get("strict_valid_frame_count") or 0) for row in rows
        ),
        "available_hand_aligned_frame_count": sum(
            int(row.get("available_hand_aligned_frame_count") or 0) for row in rows
        ),
        "repaired_frame_count": sum(
            int(row.get("repaired_frame_count") or 0) for row in rows
        ),
    }
    totals["strict_hardware_frame_count"] = sum(
        int(row.get("aligned_frame_count") or 0)
        for row in rows
        if row.get("hand_state_mode") == "hardware_feedback"
    )
    strict_frames = totals["strict_hardware_frame_count"]
    aligned_frames = totals["aligned_frame_count"]
    totals["strict_hardware_invalid_frame_count"] = (
        strict_frames - totals["strict_valid_frame_count"]
    )
    totals["available_hand_invalid_frame_count"] = (
        aligned_frames - totals["available_hand_aligned_frame_count"]
    )
    totals["strict_valid_rate_percent"] = (
        100.0 * totals["strict_valid_frame_count"] / strict_frames
        if strict_frames
        else 0.0
    )
    totals["available_hand_aligned_rate_percent"] = (
        100.0 * totals["available_hand_aligned_frame_count"] / aligned_frames
        if aligned_frames
        else 0.0
    )
    summary = {
        "schema_version": 1,
        "generated_at": dt.datetime.now().astimezone().isoformat(),
        "dataset_dir": str(dataset_dir),
        "totals": totals,
        "episodes": rows,
    }
    atomic_write_json(dataset_dir / "realignment_report.json", summary)
    write_csv(dataset_dir / "realignment_report.csv", rows)
    write_markdown(dataset_dir / "realignment_report.md", summary)
    return summary


def main() -> int:
    parser = argparse.ArgumentParser(description="Realign all SVT episodes in one day directory.")
    parser.add_argument("dataset_dir", type=Path)
    parser.add_argument(
        "--policy-config",
        type=Path,
        help="Deployed config whose offline_alignment policy overrides episode snapshots.",
    )
    args = parser.parse_args()
    summary = realign_day(args.dataset_dir, args.policy_config)
    print(json.dumps(summary["totals"], ensure_ascii=False, indent=2))
    return 0 if summary["totals"]["failed_count"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
