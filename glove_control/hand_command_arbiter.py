from __future__ import annotations

import fcntl
import json
import math
import os
import socket
import time
from pathlib import Path
from typing import Any, Iterable


class HandCommandLease:
    """Exclusive lease shared by every process capable of commanding either hand."""

    def __init__(self, path: str | Path, *, mode: str, owner: str) -> None:
        self.path = Path(path)
        self.mode = str(mode)
        self.owner = str(owner)
        self.handle: Any | None = None

    def acquire(self) -> "HandCommandLease":
        self.path.parent.mkdir(parents=True, exist_ok=True)
        handle = self.path.open("a+", encoding="utf-8")
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            handle.seek(0)
            current = handle.read().strip() or "unknown owner"
            handle.close()
            raise RuntimeError(
                f"dual-hand command lease is already held ({self.path}): {current}"
            ) from exc
        handle.seek(0)
        handle.truncate()
        json.dump(
            {
                "pid": os.getpid(),
                "host": socket.gethostname(),
                "mode": self.mode,
                "owner": self.owner,
                "acquired_at_ns": time.time_ns(),
            },
            handle,
            sort_keys=True,
        )
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
        self.handle = handle
        return self

    def close(self) -> None:
        if self.handle is not None:
            fcntl.flock(self.handle.fileno(), fcntl.LOCK_UN)
            self.handle.close()
            self.handle = None

    def __enter__(self) -> "HandCommandLease":
        return self.acquire()

    def __exit__(self, _type: object, _value: object, _traceback: object) -> None:
        self.close()


def finite_vector(values: Iterable[Any], size: int, name: str) -> list[float]:
    result = [float(value) for value in values]
    if len(result) != size or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain {size} finite values")
    return result


class HandCommandArbiter:
    """Maps the model's 12 hand values while preserving unmodelled Wuji joints."""

    DEFAULT_RIGHT_MAPPING = (
        (0, (2, 3)),
        (1, (0,)),
        (2, (4,)),
        (3, (8,)),
        (4, (12,)),
        (5, (16,)),
    )

    def __init__(self, right_mapping: Iterable[dict[str, Any]] | None = None) -> None:
        if right_mapping is None:
            self.right_mapping = self.DEFAULT_RIGHT_MAPPING
        else:
            parsed = []
            for item in right_mapping:
                virtual_index = int(item["virtual_index"])
                raw_indices = tuple(int(index) for index in item["raw_indices"])
                if not 0 <= virtual_index < 6 or not raw_indices:
                    raise ValueError(f"invalid right-hand mapping entry: {item}")
                if any(not 0 <= index < 20 for index in raw_indices):
                    raise ValueError(f"raw Wuji index outside 0..19: {item}")
                parsed.append((virtual_index, raw_indices))
            if {index for index, _ in parsed} != set(range(6)):
                raise ValueError("right-hand mapping must define every virtual index 0..5 once")
            self.right_mapping = tuple(parsed)

    @staticmethod
    def clamp_left_o6(values: Iterable[Any]) -> list[int]:
        return [int(round(max(0.0, min(255.0, value)))) for value in finite_vector(values, 6, "left O6")]

    def expand_model_action(
        self,
        hand_action: Iterable[Any],
        latched_right_raw: Iterable[Any],
    ) -> tuple[list[int], list[float]]:
        action = finite_vector(hand_action, 12, "model hand action")
        right = finite_vector(latched_right_raw, 20, "latched Wuji command")
        for virtual_index, raw_indices in self.right_mapping:
            for raw_index in raw_indices:
                right[raw_index] = action[6 + virtual_index]
        return self.clamp_left_o6(action[:6]), right

    @staticmethod
    def s5_index_only(
        glove_right_raw: Iterable[Any],
        latched_right_raw: Iterable[Any],
    ) -> list[float]:
        glove = finite_vector(glove_right_raw, 20, "Wuji glove command")
        output = finite_vector(latched_right_raw, 20, "latched Wuji command")
        output[4] = glove[4]
        return output
