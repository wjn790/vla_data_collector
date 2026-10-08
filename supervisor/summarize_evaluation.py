#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

import yaml


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize SVT single-skill and long-chain trials.")
    parser.add_argument("results", type=Path, help="One JSON object per independent trial.")
    parser.add_argument(
        "--protocol",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config" / "evaluation_protocol.yaml",
    )
    args = parser.parse_args()
    protocol = yaml.safe_load(args.protocol.read_text(encoding="utf-8"))
    records = []
    for line_number, line in enumerate(args.results.read_text(encoding="utf-8").splitlines(), 1):
        if not line.strip():
            continue
        value = json.loads(line)
        if not isinstance(value, dict):
            raise ValueError(f"line {line_number} is not an object")
        records.append(value)
    if not records:
        raise ValueError("evaluation results are empty")
    run_ids = [str(record.get("run_id", "")) for record in records]
    if any(not run_id for run_id in run_ids) or len(run_ids) != len(set(run_ids)):
        raise ValueError("each trial needs a unique non-empty run_id")

    errors = []
    safety_count = sum(bool(record.get("safety_violation")) for record in records)
    maximum_safety = int(protocol["required_safety"]["maximum_safety_violations"])
    if safety_count > maximum_safety:
        errors.append(f"safety violations {safety_count} exceed {maximum_safety}")

    single = defaultdict(list)
    chains = defaultdict(list)
    failure_reasons: Counter[str] = Counter()
    allowed_reasons = set(protocol["failure_reasons"])
    for record in records:
        if not record.get("success"):
            reason = str(record.get("failure_reason", ""))
            if reason not in allowed_reasons:
                errors.append(f"run {record['run_id']} has invalid failure_reason {reason!r}")
            failure_reasons[reason] += 1
        scope = record.get("scope")
        if scope == "single_skill":
            single[str(record.get("skill_id"))].append(record)
        elif scope == "long_chain":
            chains[str(record.get("scene"))].append(record)
            stage_results = record.get("stage_results")
            if not isinstance(stage_results, dict) or set(stage_results) != {f"S{i}" for i in range(1, 7)}:
                errors.append(f"run {record['run_id']} must report S1-S6 stage_results")
        else:
            errors.append(f"run {record['run_id']} has invalid scope {scope!r}")

    single_summary: dict[str, Any] = {}
    required_single = int(protocol["single_skill"]["trials_per_skill"])
    for skill_id, threshold in protocol["single_skill"]["minimum_success_rate"].items():
        trials = single[skill_id]
        success = sum(bool(record.get("success")) for record in trials)
        rate = success / len(trials) if trials else 0.0
        single_summary[skill_id] = {"trials": len(trials), "successes": success, "rate": rate}
        if len(trials) < required_single:
            errors.append(f"{skill_id} has {len(trials)}/{required_single} trials")
        if len(trials) >= required_single and rate < float(threshold):
            errors.append(f"{skill_id} success rate {rate:.3f} is below {threshold}")

    chain_summary: dict[str, Any] = {}
    chain_protocol = protocol["long_chain"]
    for scene, required in (
        ("standard", int(chain_protocol["standard_trials"])),
        ("perturbation", int(chain_protocol["perturbation_trials"])),
    ):
        trials = chains[scene]
        success = sum(bool(record.get("success")) for record in trials)
        rate = success / len(trials) if trials else 0.0
        chain_summary[scene] = {"trials": len(trials), "successes": success, "rate": rate}
        if len(trials) < required:
            errors.append(f"long-chain {scene} has {len(trials)}/{required} trials")
    chain_trials = chains["standard"] + chains["perturbation"]
    overall_chain_rate = (
        sum(bool(record.get("success")) for record in chain_trials) / len(chain_trials)
        if chain_trials
        else 0.0
    )
    required_chain_trials = int(chain_protocol["standard_trials"]) + int(
        chain_protocol["perturbation_trials"]
    )
    if (
        len(chain_trials) >= required_chain_trials
        and overall_chain_rate < float(chain_protocol["minimum_overall_success_rate"])
    ):
        errors.append(
            f"overall long-chain success rate {overall_chain_rate:.3f} is below "
            f"{chain_protocol['minimum_overall_success_rate']}"
        )

    result = {
        "single_skill": single_summary,
        "long_chain": {**chain_summary, "overall_rate": overall_chain_rate},
        "safety_violations": safety_count,
        "failure_reasons": dict(sorted(failure_reasons.items())),
        "accepted": not errors,
        "errors": errors,
    }
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0 if not errors else 1


if __name__ == "__main__":
    raise SystemExit(main())
