#!/usr/bin/env python3
"""Apply a bounded offline repair policy to an explicit episode selection."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import math
import shutil
from pathlib import Path
from typing import Any

import yaml

from common import atomic_write_json
from offline_align import align_episode, load_jsonl
from realign_day import merge_alignment_policy


GENERATED_FILES = (
    "frames.jsonl",
    "alignment_report.json",
    "manifest.json",
    "paired_segments.json",
)


def raw_tree_fingerprint(raw_dir: Path) -> str:
    digest = hashlib.sha256()
    for path in sorted(value for value in raw_dir.rglob("*") if value.is_file()):
        stat = path.stat()
        digest.update(str(path.relative_to(raw_dir)).encode("utf-8"))
        digest.update(b"\0")
        digest.update(str(stat.st_size).encode("ascii"))
        digest.update(b"\0")
        digest.update(str(stat.st_mtime_ns).encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest()


def selected_prefixes(selection: dict[str, Any]) -> dict[str, str]:
    values: dict[str, str] = {}
    for group, prefixes in (selection.get("groups") or {}).items():
        for prefix in prefixes or []:
            prefix = str(prefix)
            if prefix in values:
                raise ValueError(f"duplicate episode prefix in selection: {prefix}")
            values[prefix] = str(group)
    if not values:
        raise ValueError("selection contains no episode prefixes")
    return values


def resolve_episode(dataset_dir: Path, prefix: str) -> Path:
    matches = sorted(
        path
        for path in dataset_dir.glob(f"{prefix}_*")
        if path.is_dir() and (path / "raw").is_dir()
    )
    if len(matches) != 1:
        raise ValueError(f"prefix {prefix} resolved to {len(matches)} episodes")
    return matches[0]


def frame_checks(frames: list[dict[str, Any]], episode_dir: Path) -> dict[str, Any]:
    expected_prompts: dict[str, str] = {}
    bad_action_dimensions = 0
    nonfinite_actions = 0
    missing_camera_paths = 0
    missing_camera_files = 0
    prompt_mismatches = 0
    bad_state_dimensions = 0
    nonfinite_states = 0
    cameras = ("camera_top", "camera_wrist_left", "camera_wrist_right")
    for frame in frames:
        action = frame.get("action") or {}
        state = ((frame.get("observation") or {}).get("state") or {})
        arm = action.get("arm_position")
        hand = action.get("hand_position")
        state_arm = state.get("arm_position")
        state_hand = state.get("hand_position")
        if not isinstance(arm, list) or len(arm) != 14 or not isinstance(hand, list) or len(hand) != 12:
            bad_action_dimensions += 1
        values = (arm if isinstance(arm, list) else []) + (hand if isinstance(hand, list) else [])
        try:
            nonfinite_actions += int(any(not math.isfinite(float(value)) for value in values))
        except (TypeError, ValueError):
            nonfinite_actions += 1
        if (
            not isinstance(state_arm, list)
            or len(state_arm) != 14
            or not isinstance(state_hand, list)
            or len(state_hand) != 12
        ):
            bad_state_dimensions += 1
        state_values = (
            (state_arm if isinstance(state_arm, list) else [])
            + (state_hand if isinstance(state_hand, list) else [])
        )
        try:
            nonfinite_states += int(
                any(not math.isfinite(float(value)) for value in state_values)
            )
        except (TypeError, ValueError):
            nonfinite_states += 1
        images = ((frame.get("observation") or {}).get("images") or {})
        missing_camera_paths += int(
            any(not isinstance(images.get(name), dict) or not images[name].get("path") for name in cameras)
        )
        for name in cameras:
            image = images.get(name)
            if not isinstance(image, dict) or not image.get("path"):
                continue
            path = Path(str(image["path"]))
            if not path.is_absolute():
                path = episode_dir / path
            missing_camera_files += int(not path.is_file())
        skill = str(frame.get("skill_id") or "")
        task = str(frame.get("task") or "")
        if skill in expected_prompts and expected_prompts[skill] != task:
            prompt_mismatches += 1
        elif skill:
            expected_prompts[skill] = task
    return {
        "bad_action_dimension_count": bad_action_dimensions,
        "nonfinite_action_count": nonfinite_actions,
        "bad_state_dimension_count": bad_state_dimensions,
        "nonfinite_state_count": nonfinite_states,
        "missing_camera_path_count": missing_camera_paths,
        "missing_camera_file_count": missing_camera_files,
        "prompt_mismatch_count": prompt_mismatches,
        "prompts": expected_prompts,
    }


def segment_results(
    frames: list[dict[str, Any]], restrictions: dict[str, Any]
) -> list[dict[str, Any]]:
    results = []
    include = {str(value) for value in restrictions.get("include") or []}
    exclude = {str(value) for value in restrictions.get("exclude") or []}
    by_skill: dict[str, list[dict[str, Any]]] = {}
    for frame in frames:
        by_skill.setdefault(str(frame.get("skill_id") or ""), []).append(frame)
    for skill, values in sorted(by_skill.items()):
        valid = sum(bool(frame["validity"].get("valid_for_training")) for frame in values)
        allowed = (not include or skill in include) and skill not in exclude
        results.append(
            {
                "skill_id": skill,
                "frame_count": len(values),
                "valid_frame_count": valid,
                "valid_rate": valid / len(values),
                "allowed_by_selection": allowed,
                "technically_ready": allowed and valid == len(values),
            }
        )
    return results


def repair_selection(
    dataset_dir: Path,
    policy_path: Path,
    selection_path: Path,
    backup_root: Path,
) -> dict[str, Any]:
    dataset_dir = dataset_dir.resolve()
    policy = yaml.safe_load(policy_path.read_text(encoding="utf-8"))
    selection = yaml.safe_load(selection_path.read_text(encoding="utf-8"))
    prefixes = selected_prefixes(selection)
    restrictions = selection.get("segment_restrictions") or {}
    max_repaired_frame_rate = float(
        (policy.get("admission") or {}).get("max_repaired_frame_rate", 0.05)
    )
    rows = []
    for prefix, group in prefixes.items():
        episode = resolve_episode(dataset_dir, prefix)
        backup_dir = backup_root / episode.name
        backup_dir.mkdir(parents=True, exist_ok=True)
        for name in GENERATED_FILES:
            source = episode / name
            destination = backup_dir / name
            if source.is_file() and not destination.exists():
                shutil.copy2(source, destination)
        raw_before = raw_tree_fingerprint(episode / "raw")
        row: dict[str, Any] = {"episode": episode.name, "prefix": prefix, "group": group}
        try:
            episode_config = yaml.safe_load((episode / "config.yaml").read_text(encoding="utf-8"))
            report = align_episode(episode, merge_alignment_policy(episode_config, policy))
            raw_after = raw_tree_fingerprint(episode / "raw")
            if raw_after != raw_before:
                raise RuntimeError("raw tree fingerprint changed during offline repair")
            frames = load_jsonl(episode / "frames.jsonl")
            checks = frame_checks(frames, episode)
            segments = segment_results(frames, restrictions.get(prefix) or {})
            continuity = report.get("continuity") or {}
            repaired_frame_count = int(
                (report.get("repair") or {}).get("repaired_frame_count", 0)
            )
            repaired_frame_rate = repaired_frame_count / len(frames)
            fully_valid = (
                report.get("valid_frame_count") == report.get("frame_count")
                and continuity.get("timing_gap_count") == 0
                and (report.get("hand_state") or {}).get("mode") == "hardware_feedback"
                and repaired_frame_rate <= max_repaired_frame_rate
                and all(
                    checks[key] == 0
                    for key in (
                        "bad_action_dimension_count",
                        "nonfinite_action_count",
                        "bad_state_dimension_count",
                        "nonfinite_state_count",
                        "missing_camera_path_count",
                        "missing_camera_file_count",
                        "prompt_mismatch_count",
                    )
                )
            )
            row.update(
                {
                    "result": "repaired",
                    "technical_disposition": (
                        "ready_after_bounded_repair"
                        if fully_valid
                        else (
                            "manual_visual_review_high_repair_rate"
                            if report.get("valid_frame_count") == report.get("frame_count")
                            else "manual_review_or_retake"
                        )
                    ),
                    "frame_count": report.get("frame_count"),
                    "valid_frame_count": report.get("valid_frame_count"),
                    "valid_rate": report.get("valid_frame_count", 0) / report.get("frame_count", 1),
                    "internal_invalid_frame_count": continuity.get("internal_invalid_frame_count"),
                    "timing_gap_count": continuity.get("timing_gap_count"),
                    "invalid_reason_counts": report.get("invalid_reason_counts") or {},
                    "repair": report.get("repair") or {},
                    "repaired_frame_rate": repaired_frame_rate,
                    "checks": checks,
                    "segments": segments,
                    "raw_fingerprint": raw_after,
                    "raw_unchanged": True,
                    "error": None,
                }
            )
        except Exception as exc:
            row.update(
                {
                    "result": "failed",
                    "technical_disposition": "repair_failed",
                    "raw_unchanged": raw_tree_fingerprint(episode / "raw") == raw_before,
                    "error": f"{type(exc).__name__}: {exc}",
                }
            )
        rows.append(row)
        print(
            f"{prefix}: {row['result']} {row['technical_disposition']} "
            f"valid={row.get('valid_frame_count')}/{row.get('frame_count')}",
            flush=True,
        )
    counts: dict[str, int] = {}
    for row in rows:
        key = row["technical_disposition"]
        counts[key] = counts.get(key, 0) + 1
    result = {
        "schema_version": 1,
        "generated_at": dt.datetime.now().astimezone().isoformat(),
        "dataset_dir": str(dataset_dir),
        "policy": str(policy_path.resolve()),
        "selection": str(selection_path.resolve()),
        "backup_root": str(backup_root.resolve()),
        "admission": {"max_repaired_frame_rate": max_repaired_frame_rate},
        "counts": counts,
        "episodes": rows,
    }
    atomic_write_json(dataset_dir / "dataset_repair_report_v2.json", result)
    markdown = [
        "# Dataset Repair Report V2",
        "",
        f"Generated: {result['generated_at']}",
        "",
        "`raw/` is immutable. A row is ready only when every aligned frame and all structural checks pass.",
        "Operator success confirmation remains a separate requirement.",
        "",
        "| Episode | Group | Result | Valid | Internal invalid | Timing gaps | Remaining reasons |",
        "|---|---|---|---:|---:|---:|---|",
    ]
    for row in rows:
        reasons = ", ".join(
            f"{name}={count}"
            for name, count in sorted((row.get("invalid_reason_counts") or {}).items())
        )
        markdown.append(
            f"| {row['prefix']} | {row['group']} | {row['technical_disposition']} | "
            f"{row.get('valid_frame_count', '-')}/{row.get('frame_count', '-')} | "
            f"{row.get('internal_invalid_frame_count', '-')} | "
            f"{row.get('timing_gap_count', '-')} | {reasons or '-'} |"
        )
    (dataset_dir / "dataset_repair_report_v2.md").write_text(
        "\n".join(markdown) + "\n", encoding="utf-8"
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset_dir", type=Path)
    parser.add_argument("--policy", type=Path, required=True)
    parser.add_argument("--selection", type=Path, required=True)
    parser.add_argument("--backup-root", type=Path, required=True)
    args = parser.parse_args()
    result = repair_selection(
        args.dataset_dir, args.policy, args.selection, args.backup_root
    )
    print(json.dumps(result["counts"], ensure_ascii=False, sort_keys=True))
    return int(any(row["result"] == "failed" for row in result["episodes"]))


if __name__ == "__main__":
    raise SystemExit(main())
