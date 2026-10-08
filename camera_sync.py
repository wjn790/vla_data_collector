from __future__ import annotations

from collections import deque
from typing import Any


def sync_timestamp_ns(sample: dict[str, Any]) -> int:
    """Return a timestamp comparable across cameras on the SVT host."""
    hardware = sample.get("hardware_timestamp_ns")
    domain = str(sample.get("hardware_timestamp_domain", "")).lower()
    if hardware is not None and "global_time" in domain:
        return int(hardware)
    return int(sample["host_timestamp_ns"])


class CameraSynchronizer:
    def __init__(self, camera_names: list[str], max_frames: int = 32) -> None:
        self.buffers = {name: deque(maxlen=max_frames) for name in camera_names}
        self.last_selected_host_ns: dict[str, int] = {}

    def add(self, sample: dict[str, Any]) -> None:
        camera = str(sample["camera"])
        if camera not in self.buffers:
            return
        buffer = self.buffers[camera]
        timestamp = int(sample["host_timestamp_ns"])
        if buffer and int(buffer[-1]["host_timestamp_ns"]) == timestamp:
            return
        buffer.append(sample)

    def has_any(self, camera: str) -> bool:
        return bool(self.buffers.get(camera))

    def prune(self, reference_ns: int, retention_ms: float) -> None:
        oldest_ns = int(reference_ns - retention_ms * 1_000_000)
        for buffer in self.buffers.values():
            while buffer and int(buffer[0]["host_timestamp_ns"]) < oldest_ns:
                buffer.popleft()

    def _candidates(self, camera: str, reference_ns: int, max_age_ms: float) -> list[dict[str, Any]]:
        last_host_ns = self.last_selected_host_ns.get(camera, -1)
        max_age_ns = int(max_age_ms * 1_000_000)
        return [
            sample
            for sample in self.buffers.get(camera, ())
            if int(sample["host_timestamp_ns"]) > last_host_ns
            # Arrival freshness and exposure synchronization are different.
            # RealSense global_time precedes host delivery by the USB pipeline latency.
            and abs(int(sample["host_timestamp_ns"]) - reference_ns) <= max_age_ns
        ]

    def has_unselected(self, camera: str, reference_ns: int, max_age_ms: float) -> bool:
        return bool(self._candidates(camera, reference_ns, max_age_ms))

    def _consume(self, camera: str, sample: dict[str, Any]) -> None:
        self.last_selected_host_ns[camera] = int(sample["host_timestamp_ns"])

    def select_single(
        self,
        camera: str,
        reference_ns: int,
        max_age_ms: float,
        *,
        consume: bool = True,
        target_sync_ns: int | None = None,
    ) -> dict[str, Any] | None:
        candidates = self._candidates(camera, reference_ns, max_age_ms)
        if not candidates:
            return None
        target_ns = reference_ns if target_sync_ns is None else target_sync_ns
        selected = min(candidates, key=lambda sample: abs(sync_timestamp_ns(sample) - target_ns))
        if consume:
            self._consume(camera, selected)
        return selected

    def select_wrist_pair(
        self,
        left_camera: str,
        right_camera: str,
        reference_ns: int,
        max_age_ms: float,
        max_skew_ms: float,
        *,
        consume: bool = True,
    ) -> tuple[dict[str, Any] | None, dict[str, Any] | None, dict[str, Any]]:
        left_candidates = self._candidates(left_camera, reference_ns, max_age_ms)
        right_candidates = self._candidates(right_camera, reference_ns, max_age_ms)
        max_skew_ns = int(max_skew_ms * 1_000_000)
        valid_pairs: list[tuple[tuple[int, int], dict[str, Any], dict[str, Any]]] = []
        nearest_skew_ns: int | None = None

        for left in left_candidates:
            left_ns = sync_timestamp_ns(left)
            for right in right_candidates:
                right_ns = sync_timestamp_ns(right)
                skew_ns = abs(right_ns - left_ns)
                if nearest_skew_ns is None or skew_ns < nearest_skew_ns:
                    nearest_skew_ns = skew_ns
                if skew_ns <= max_skew_ns:
                    reference_error = max(abs(left_ns - reference_ns), abs(right_ns - reference_ns))
                    # Exposure agreement matters more than proximity to the
                    # collector tick. Both frames are already bounded by
                    # max_age_ms, so prefer the best synchronized pair first.
                    valid_pairs.append(((skew_ns, reference_error), left, right))

        if not valid_pairs:
            reason = "missing_new_frame" if not left_candidates or not right_candidates else "skew_exceeded"
            return None, None, {
                "required": True,
                "valid": False,
                "reason": reason,
                "skew_ms": nearest_skew_ns / 1_000_000.0 if nearest_skew_ns is not None else None,
                "max_skew_ms": float(max_skew_ms),
            }

        _, left, right = min(valid_pairs, key=lambda item: item[0])
        if consume:
            self._consume(left_camera, left)
            self._consume(right_camera, right)
        skew_ms = abs(sync_timestamp_ns(right) - sync_timestamp_ns(left)) / 1_000_000.0
        return left, right, {
            "required": True,
            "valid": True,
            "reason": "matched",
            "skew_ms": skew_ms,
            "max_skew_ms": float(max_skew_ms),
        }

    def select(
        self,
        reference_ns: int,
        enabled_cameras: list[str],
        camera_max_age_ms: float,
        wrist_config: dict[str, Any],
    ) -> tuple[dict[str, dict[str, Any]], dict[str, Any]]:
        selected: dict[str, dict[str, Any]] = {}
        left = str(wrist_config["left_camera"])
        right = str(wrist_config["right_camera"])
        wrists_required = left in enabled_cameras and right in enabled_cameras

        if wrists_required:
            left_sample, right_sample, wrist_status = self.select_wrist_pair(
                left,
                right,
                reference_ns,
                float(wrist_config["max_age_ms"]),
                float(wrist_config["max_skew_ms"]),
                consume=False,
            )
            if left_sample is not None and right_sample is not None:
                selected[left] = left_sample
                selected[right] = right_sample
                target_sync_ns = (
                    sync_timestamp_ns(left_sample) + sync_timestamp_ns(right_sample)
                ) // 2
            else:
                target_sync_ns = reference_ns
        else:
            wrist_status = {
                "required": False,
                "valid": True,
                "reason": "disabled",
                "skew_ms": None,
                "max_skew_ms": float(wrist_config["max_skew_ms"]),
            }
            target_sync_ns = reference_ns

        for camera in enabled_cameras:
            if wrists_required and camera in (left, right):
                continue
            sample = self.select_single(
                camera,
                reference_ns,
                camera_max_age_ms,
                consume=False,
                target_sync_ns=target_sync_ns,
            )
            if sample is not None:
                selected[camera] = sample

        complete = len(selected) == len(enabled_cameras)
        timestamps = [sync_timestamp_ns(sample) for sample in selected.values()]
        span_ms = (
            (max(timestamps) - min(timestamps)) / 1_000_000.0
            if len(timestamps) >= 2
            else None
        )
        max_group_span_ms = wrist_config.get("max_group_span_ms")
        span_valid = (
            max_group_span_ms is None
            or span_ms is None
            or span_ms <= float(max_group_span_ms)
        )
        group_status = {
            "required": max_group_span_ms is not None and len(enabled_cameras) > 1,
            "valid": complete and span_valid,
            "reason": (
                "matched"
                if complete and span_valid
                else "group_skew_exceeded"
                if complete
                else "missing_new_frame"
            ),
            "span_ms": span_ms,
            "max_span_ms": (
                None if max_group_span_ms is None else float(max_group_span_ms)
            ),
        }

        # Selection is transactional: a partial or over-skewed group remains
        # available for the next poll instead of advancing individual cameras.
        if complete and wrist_status["valid"] and span_valid:
            for camera, sample in selected.items():
                self._consume(camera, sample)
            return selected, {"wrist_pair": wrist_status, "camera_group": group_status}
        if complete and not span_valid:
            selected = {}
        return selected, {"wrist_pair": wrist_status, "camera_group": group_status}
