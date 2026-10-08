#!/usr/bin/env python3
"""Build a segment-aware curation report for one aligned SVT collection day."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
from pathlib import Path
from typing import Any

import yaml

from common import atomic_write_json
from episode_quality_monitor import assess_start_capture


def load_json(path: Path, default: Any = None) -> Any:
    if not path.is_file():
        return default
    return json.loads(path.read_text(encoding="utf-8"))


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    values = []
    if not path.is_file():
        return values
    with path.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            if not line.strip():
                continue
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(value, dict):
                values.append(value)
    return values


def invalid_runs(frames: list[dict[str, Any]]) -> list[tuple[int, int]]:
    runs: list[tuple[int, int]] = []
    start: int | None = None
    flags = [
        bool((frame.get("validity") or {}).get("valid_for_training"))
        for frame in frames
    ]
    for index, valid in enumerate(flags + [True]):
        if not valid and start is None:
            start = index
        elif valid and start is not None:
            runs.append((start, index - 1))
            start = None
    return runs


def technical_disposition(
    *,
    valid_rate: float,
    max_invalid_run: int,
    invalid_reasons: dict[str, int],
    repaired_rate: float = 0.0,
    max_repaired_rate: float = 0.05,
) -> tuple[str, list[str]]:
    arm_invalid = int(invalid_reasons.get("arm", 0))
    if valid_rate == 1.0 and max_invalid_run == 0 and not invalid_reasons:
        if repaired_rate > max_repaired_rate:
            return "manual_visual_review_high_repair_rate", [
                "all_frames_structurally_valid_after_bounded_repair",
                "repaired_frame_rate_exceeds_quality_threshold",
            ]
        return "ready_after_bounded_repair", [
            "all_aligned_frames_valid_after_bounded_repair",
            "operator_success_confirmation_still_required",
        ]
    if max_invalid_run >= 8 or arm_invalid >= 10:
        return "exclude_sync_gap", [
            "long_or_repeated_arm_alignment_gap",
            "do_not_compress_or_interpolate_contact_motion",
        ]
    if valid_rate < 0.85 or max_invalid_run > 6:
        return "retake_recommended", ["camera_alignment_loss_is_too_dense"]
    if valid_rate >= 0.90 and max_invalid_run <= 5 and arm_invalid <= 5:
        return "strong_repair_candidate", [
            "task_flow_complete",
            "remaining_gaps_are_short_and_camera_dominated",
        ]
    return "repair_candidate", [
        "task_flow_complete",
        "requires_bounded_camera_gap_repair_and_manual_review",
    ]


def load_attempts(attempt_dir: Path) -> dict[str, dict[str, Any]]:
    values: dict[str, dict[str, Any]] = {}
    for path in sorted(attempt_dir.glob("*.json")):
        attempt = load_json(path, {}) or {}
        source = Path(str(attempt.get("source_episode") or "")).name
        if source:
            values[source] = attempt
    return values


def action_checks(frames: list[dict[str, Any]]) -> dict[str, Any]:
    bad_dimensions = []
    nonfinite = []
    missing_images = []
    cameras = ("camera_top", "camera_wrist_left", "camera_wrist_right")
    for index, frame in enumerate(frames):
        action = frame.get("action") or {}
        arm = action.get("arm_position")
        hand = action.get("hand_position")
        if not isinstance(arm, list) or len(arm) != 14 or not isinstance(hand, list) or len(hand) != 12:
            bad_dimensions.append(index)
        try:
            values = (arm if isinstance(arm, list) else []) + (
                hand if isinstance(hand, list) else []
            )
            if any(not math.isfinite(float(value)) for value in values):
                nonfinite.append(index)
        except (TypeError, ValueError):
            nonfinite.append(index)
        images = ((frame.get("observation") or {}).get("images") or {})
        if any(not isinstance(images.get(camera), dict) or not images[camera].get("path") for camera in cameras):
            missing_images.append(index)
    return {
        "bad_action_dimension_count": len(bad_dimensions),
        "nonfinite_action_count": len(nonfinite),
        "missing_camera_path_count": len(missing_images),
    }


def review_day(
    dataset_dir: Path, attempt_dir: Path, monitor_config_path: Path
) -> dict[str, Any]:
    monitor_config = yaml.safe_load(monitor_config_path.read_text(encoding="utf-8"))
    thresholds = monitor_config["quality_thresholds"]
    attempts = load_attempts(attempt_dir)
    rows = []
    for episode in sorted(path for path in dataset_dir.iterdir() if path.is_dir()):
        manifest = load_json(episode / "manifest.json", {}) or {}
        alignment = load_json(episode / "alignment_report.json", {}) or {}
        frames = load_jsonl(episode / "frames.jsonl")
        if not manifest or not frames:
            continue
        segment_document = load_json(episode / "paired_segments.json", {}) or {}
        segments = segment_document.get("segments") or []
        attempt = attempts.get(episode.name, {})
        flags = [
            bool((frame.get("validity") or {}).get("valid_for_training"))
            for frame in frames
        ]
        runs = invalid_runs(frames)
        valid_count = sum(flags)
        valid_rate = valid_count / len(frames)
        reasons = alignment.get("invalid_reason_counts") or {}
        repaired_count = int((alignment.get("repair") or {}).get("repaired_frame_count", 0))
        repaired_rate = repaired_count / len(frames)
        disposition, disposition_reasons = technical_disposition(
            valid_rate=valid_rate,
            max_invalid_run=max((end - start + 1 for start, end in runs), default=0),
            invalid_reasons=reasons,
            repaired_rate=repaired_rate,
            max_repaired_rate=float(thresholds["repair_rate_max"]),
        )
        if attempt.get("status") == "aborted" or not segments:
            disposition = "exclude_aborted_or_unsegmented"
            disposition_reasons = [
                "paired_attempt_did_not_finalize",
                str(attempt.get("error") or "missing_paired_segments"),
            ]
        start_capture = (
            assess_start_capture(episode, manifest, segment_document, frames, thresholds)
            if segments
            else {"status": "failed", "issues": ["missing_paired_segments"]}
        )
        segment_rows = []
        for segment in segments:
            skill_id = str(segment.get("skill_id"))
            start = int(segment["source_start_frame"])
            end = int(segment["source_end_frame"])
            segment_flags = flags[start : end + 1]
            segment_frames = frames[start : end + 1]
            segment_repaired_count = sum(
                bool((frame.get("validity") or {}).get("repaired"))
                for frame in segment_frames
            )
            segment_repaired_rate = segment_repaired_count / len(segment_frames)
            recommendation = disposition
            recommendation_reasons = list(disposition_reasons)
            if segment_flags and all(segment_flags):
                # The audit contract qualifies a continuous paired episode as a unit.
                if repaired_rate <= float(thresholds["repair_rate_max"]):
                    recommendation = "ready_after_bounded_repair"
                    recommendation_reasons = [
                        "all_segment_frames_valid_after_bounded_repair",
                        "operator_success_confirmation_still_required",
                    ]
                else:
                    recommendation = "manual_visual_review_high_repair_rate"
                    recommendation_reasons = [
                        "all_segment_frames_structurally_valid_after_bounded_repair",
                        "repaired_frame_rate_exceeds_quality_threshold",
                    ]
            if skill_id == "S1" and start_capture.get("status") != "pass":
                drift = float(start_capture.get("pre_recording_left_arm_drift_rad") or 0.0)
                first_delta = float(start_capture.get("first_aligned_from_raw_initial_rad") or 0.0)
                if drift > 0.08 or first_delta > 0.08:
                    recommendation = "exclude_s1_initial_motion_missing"
                else:
                    recommendation = "manual_review_s1_start_pose"
                recommendation_reasons = list(start_capture.get("issues") or [])
            segment_rows.append(
                {
                    "skill_id": skill_id,
                    "task": segment.get("task"),
                    "prompt_version": segment.get("prompt_version"),
                    "start_frame": start,
                    "end_frame": end,
                    "frame_count": len(segment_flags),
                    "valid_frame_count": sum(segment_flags),
                    "valid_rate": sum(segment_flags) / len(segment_flags),
                    "repaired_frame_count": segment_repaired_count,
                    "repaired_frame_rate": segment_repaired_rate,
                    "recommendation": recommendation,
                    "reasons": recommendation_reasons,
                }
            )
        continuity = alignment.get("continuity") or {}
        rows.append(
            {
                "episode": episode.name,
                "manifest_status": manifest.get("status"),
                "attempt_status": attempt.get("status"),
                "attempt_outcome": attempt.get("outcome"),
                "attempt_error": attempt.get("error"),
                "frame_count": len(frames),
                "valid_frame_count": valid_count,
                "valid_rate": valid_rate,
                "repaired_frame_count": (alignment.get("repair") or {}).get("repaired_frame_count", 0),
                "repaired_frame_rate": repaired_rate,
                "internal_invalid_frame_count": continuity.get("internal_invalid_frame_count"),
                "timing_gap_count": continuity.get("timing_gap_count"),
                "invalid_run_count": len(runs),
                "max_invalid_run": max((end - start + 1 for start, end in runs), default=0),
                "invalid_reason_counts": reasons,
                "hand_state_mode": (alignment.get("hand_state") or {}).get("mode"),
                "start_capture": start_capture,
                "checks": action_checks(frames),
                "episode_recommendation": disposition,
                "recommendation_reasons": disposition_reasons,
                "segments": segment_rows,
                "success_confirmation_required": manifest.get("attempt_outcome") != "success",
            }
        )
    counts: dict[str, int] = {}
    for row in rows:
        key = str(row["episode_recommendation"])
        counts[key] = counts.get(key, 0) + 1
    technically_ready_segments = []
    for row in rows:
        operator_success_confirmed = row.get("attempt_outcome") == "success"
        for segment in row["segments"]:
            if segment["recommendation"] != "ready_after_bounded_repair":
                continue
            technically_ready_segments.append(
                {
                    "episode": row["episode"],
                    "skill_id": segment["skill_id"],
                    "task": segment.get("task"),
                    "prompt_version": segment.get("prompt_version"),
                    "start_frame": segment["start_frame"],
                    "end_frame": segment["end_frame"],
                    "frame_count": segment["frame_count"],
                    "operator_success_confirmed": operator_success_confirmed,
                    "training_eligible": operator_success_confirmed,
                    "status": (
                        "training_eligible"
                        if operator_success_confirmed
                        else "technical_ready_pending_operator_success"
                    ),
                }
            )
    report = {
        "schema_version": 1,
        "generated_at": dt.datetime.now().astimezone().isoformat(),
        "dataset_dir": str(dataset_dir),
        "policy": {
            "strict_current_alignment_is_not_directly_trainable_with_internal_gaps": True,
            "ready_after_bounded_repair": (
                "all frames in the episode or segment pass bounded alignment repair; "
                "operator success confirmation remains required"
            ),
            "repair_rate_max": float(thresholds["repair_rate_max"]),
            "strong_repair_candidate": "valid_rate >= 0.90, max invalid run <= 5, arm-invalid <= 5",
            "repair_candidate": "valid_rate >= 0.85 without long/repeated arm gaps",
            "exclude_sync_gap": "max invalid run >= 8 or arm-invalid >= 10",
            "success_labels": "all unreviewed episodes still require operator confirmation",
        },
        "counts": counts,
        "technically_ready_segment_count": len(technically_ready_segments),
        "training_eligible_segment_count": sum(
            bool(value["training_eligible"]) for value in technically_ready_segments
        ),
        "technically_ready_segments": technically_ready_segments,
        "episodes": rows,
    }
    return report


def write_markdown(path: Path, report: dict[str, Any]) -> None:
    lines = [
        "# 2026-07-30 SVT Dataset Review",
        "",
        f"Generated: {report['generated_at']}",
        "",
        "No episode is directly trainable as a complete continuous trajectory while internal invalid frames remain.",
        "Recommendations identify repair priority; operator success confirmation is still required.",
        f"Technically ready segments: {report.get('technically_ready_segment_count', 0)}; "
        f"training eligible after operator confirmation: "
        f"{report.get('training_eligible_segment_count', 0)}.",
        "",
        "| Episode | Recommendation | Valid | Repaired | Internal invalid | Max run | Timing gaps | S1 | S2 |",
        "|---|---|---:|---:|---:|---:|---:|---|---|",
    ]
    for row in report["episodes"]:
        segment_values = {value["skill_id"]: value["recommendation"] for value in row["segments"]}
        lines.append(
            f"| {row['episode'][:6]} | {row['episode_recommendation']} | "
            f"{row['valid_frame_count']}/{row['frame_count']} ({100 * row['valid_rate']:.2f}%) | "
            f"{row['repaired_frame_count']} | {row['internal_invalid_frame_count']} | "
            f"{row['max_invalid_run']} | {row['timing_gap_count']} | "
            f"{segment_values.get('S1', '-')} | {segment_values.get('S2', '-')} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset_dir", type=Path)
    parser.add_argument(
        "--attempt-dir",
        type=Path,
        default=Path("/home/svt/lingbot_data_collector/paired_attempts"),
    )
    parser.add_argument(
        "--monitor-config",
        type=Path,
        default=Path("/home/svt/lingbot_data_collector/config/data_quality_monitor.yaml"),
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = review_day(args.dataset_dir.resolve(), args.attempt_dir, args.monitor_config)
    output = args.output or args.dataset_dir / "curation_manifest_v1.json"
    atomic_write_json(output, report)
    write_markdown(output.with_suffix(".md"), report)
    print(json.dumps(report["counts"], ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
