from __future__ import annotations

import json
import math
import os
import tempfile
import time
from pathlib import Path
from typing import Any, Iterable


def now_ns() -> int:
    return time.time_ns()


def atomic_write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            json.dump(value, handle, ensure_ascii=False, indent=2)
            handle.write("\n")
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass
        raise


def flatten(values: Any) -> list[float] | None:
    if values is None:
        return None
    output: list[float] = []

    def visit(value: Any) -> None:
        if isinstance(value, (list, tuple)):
            for item in value:
                visit(item)
        else:
            output.append(float(value))

    try:
        visit(values)
    except (TypeError, ValueError):
        return None
    return output


def fixed_vector(values: Any, size: int) -> list[float] | None:
    output = flatten(values)
    if output is None or len(output) != size or not all(math.isfinite(v) for v in output):
        return None
    return output


def prefix_vector(values: Any, size: int) -> list[float] | None:
    output = flatten(values)
    if output is None or len(output) < size or not all(math.isfinite(v) for v in output):
        return None
    return output[:size]


def reorder_joint_state(sample: dict[str, Any] | None, names: Iterable[str]) -> dict[str, Any] | None:
    if not sample:
        return None
    source_names = sample.get("name") or []
    index = {str(name): i for i, name in enumerate(source_names)}
    wanted = list(names)
    if any(name not in index for name in wanted):
        return None

    output: dict[str, Any] = {
        "timestamp_ns": sample.get("timestamp_ns"),
        "received_ns": sample.get("received_ns"),
        "name": wanted,
    }
    for key in ("position", "velocity", "effort"):
        values = sample.get(key) or []
        if len(values) < len(source_names):
            output[key] = None
        else:
            output[key] = [float(values[index[name]]) for name in wanted]
    return output


def age_ms(sample_timestamp_ns: int | None, frame_timestamp_ns: int) -> float | None:
    if not sample_timestamp_ns:
        return None
    return (frame_timestamp_ns - int(sample_timestamp_ns)) / 1_000_000.0


def is_fresh(sample_timestamp_ns: int | None, frame_timestamp_ns: int, threshold_ms: float) -> bool:
    age = age_ms(sample_timestamp_ns, frame_timestamp_ns)
    return age is not None and -threshold_ms <= age <= threshold_ms
