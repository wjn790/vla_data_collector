from __future__ import annotations

import glob
import json
import os
import time
from pathlib import Path
from typing import Any

from common import fixed_vector


class TeleopLogSource:
    """Tail the current dual-hand teleoperation JSONL without locking it."""

    def __init__(self, pattern: str, hand_config: dict[str, Any]) -> None:
        self.pattern = pattern
        self.hand_config = hand_config
        self.path: Path | None = None
        self.handle = None
        self.latest: dict[str, Any] | None = None
        self.pending: list[dict[str, Any]] = []
        self.last_scan = 0.0

    def _newest_file(self) -> Path | None:
        candidates = [Path(path) for path in glob.glob(os.path.expanduser(self.pattern))]
        candidates = [path for path in candidates if path.is_file()]
        return max(candidates, key=lambda path: path.stat().st_mtime_ns) if candidates else None

    def _open(self, path: Path) -> None:
        if self.handle is not None:
            self.handle.close()
        self.path = path
        self.handle = path.open("r", encoding="utf-8", errors="replace")

        # Prime with the most recent complete line, then follow new writes.
        size = path.stat().st_size
        self.handle.seek(max(0, size - 256 * 1024))
        if self.handle.tell() > 0:
            self.handle.readline()
        self._drain(collect=False)

    def _drain(self, collect: bool = True) -> None:
        if self.handle is None:
            return
        while True:
            position = self.handle.tell()
            line = self.handle.readline()
            if not line:
                self.handle.seek(position)
                break
            try:
                record = json.loads(line)
            except (json.JSONDecodeError, UnicodeDecodeError):
                continue
            if isinstance(record, dict) and isinstance(record.get("_timestamp_ns"), int):
                self.latest = record
                if collect:
                    self.pending.append(record)

    def poll(self) -> dict[str, Any] | None:
        now = time.monotonic()
        if now - self.last_scan >= 0.5:
            self.last_scan = now
            newest = self._newest_file()
            if newest is not None and newest != self.path:
                self._open(newest)
        if self.handle is not None:
            self._drain()
        return self.latest

    def drain_pending(self) -> list[dict[str, Any]]:
        records = self.pending
        self.pending = []
        return records

    def close(self) -> None:
        if self.handle is not None:
            self.handle.close()
            self.handle = None


def project_wuji(values: Any, projection: list[dict[str, Any]]) -> list[float] | None:
    raw = fixed_vector(values, 20)
    if raw is None:
        return None
    output: list[float] = []
    try:
        for joint in projection:
            terms = joint["terms"]
            output.append(sum(raw[int(term["index"])] * float(term["weight"]) for term in terms))
    except (KeyError, IndexError, TypeError, ValueError):
        return None
    return output if len(output) == 6 else None


def decode_hand_record(record: dict[str, Any] | None, config: dict[str, Any]) -> dict[str, Any]:
    if not record:
        return {
            "timestamp_ns": None,
            "actual_timestamp_ns": None,
            "action": None,
            "state": None,
            "raw": None,
            "state_source": "missing",
        }

    left_command = fixed_vector(record.get("left_o6_command"), 6)
    right_command_raw = fixed_vector(record.get("right_wuji_command"), 20)
    right_command = project_wuji(right_command_raw, config["right_virtual_projection"])
    action = left_command + right_command if left_command is not None and right_command is not None else None

    left_actual = fixed_vector(record.get("left_o6_actual"), 6)
    right_actual_raw = fixed_vector(record.get("right_wuji_actual"), 20)
    right_actual = project_wuji(right_actual_raw, config["right_virtual_projection"])
    if left_actual is not None and right_actual is not None:
        state = left_actual + right_actual
        state_source = "hardware_feedback"
    else:
        state = action
        state_source = "command_fallback" if action is not None else "missing"

    return {
        "timestamp_ns": int(record["_timestamp_ns"]),
        "actual_timestamp_ns": record.get("_actual_state_timestamp_ns"),
        "action": action,
        "state": state,
        "state_source": state_source,
        "raw": {
            "left_o6_command": left_command,
            "right_wuji_command": right_command_raw,
            "left_o6_actual": left_actual,
            "right_wuji_actual": right_actual_raw,
            "left_glove_received_at": record.get("left_glove_received_at"),
            "right_glove_received_at": record.get("right_glove_received_at"),
            "sent_o6": record.get("sent_o6"),
            "sent_wuji": record.get("sent_wuji"),
            "errors": record.get("errors") or [],
            "source": record.get("source"),
            "log_path": None,
        },
    }
