from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from camera_sync import sync_timestamp_ns


class JsonlWriter:
    def __init__(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.path = path
        self.handle = path.open("a", encoding="utf-8", buffering=1)
        self.count = 0

    def write(self, value: dict[str, Any]) -> None:
        self.handle.write(json.dumps(value, ensure_ascii=False, separators=(",", ":")) + "\n")
        self.count += 1

    def close(self) -> None:
        if not self.handle.closed:
            self.handle.close()


class RawCameraWriter:
    """Persist every encoded frame before queueing its metadata to the parent."""

    def __init__(self, episode_dir: Path, camera: str) -> None:
        self.episode_dir = episode_dir
        self.camera = camera
        self.image_dir = episode_dir / "raw" / "cameras" / camera / "images"
        self.image_dir.mkdir(parents=True, exist_ok=True)
        self.timestamps = JsonlWriter(
            episode_dir / "raw" / "cameras" / camera / "timestamps.jsonl"
        )
        self.sequence = 0

    def write(self, sample: dict[str, Any]) -> dict[str, Any]:
        jpeg = sample.pop("jpeg")
        host_timestamp_ns = int(sample["host_timestamp_ns"])
        filename = f"{self.sequence:08d}_{host_timestamp_ns}.jpg"
        absolute = self.image_dir / filename
        absolute.write_bytes(jpeg)
        relative = absolute.relative_to(self.episode_dir).as_posix()
        value = dict(sample)
        value["path"] = relative
        value["sequence"] = self.sequence
        value["sync_timestamp_ns"] = sync_timestamp_ns(value)
        self.timestamps.write({key: item for key, item in value.items() if key != "kind"})
        self.sequence += 1
        return value

    def close(self) -> None:
        self.timestamps.close()
