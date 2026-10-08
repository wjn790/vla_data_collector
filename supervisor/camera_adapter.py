from __future__ import annotations

import multiprocessing as mp
import queue
import subprocess
import time
from pathlib import Path
from typing import Any

import numpy as np
import yaml


class LiveCameraAdapter:
    def __init__(self, collector_config_path: Path) -> None:
        from camera_capture import camera_worker
        from camera_sync import CameraSynchronizer

        self.config = yaml.safe_load(collector_config_path.read_text(encoding="utf-8"))
        camera_config = self.config["cameras"]
        if any(
            value.get("enabled", True)
            and value.get("backend") == "zed"
            and value.get("conflicts_service") == "svtrobo-web.service"
            for value in camera_config.values()
        ):
            status = subprocess.run(
                ["systemctl", "is-active", "--quiet", "svtrobo-web.service"],
                check=False,
            )
            if status.returncode == 0:
                raise RuntimeError(
                    "svtrobo-web.service owns the ZED camera; stop it before execution"
                )

        self.names = [
            name for name, value in camera_config.items() if value.get("enabled", True)
        ]
        expected = {"camera_top", "camera_wrist_left", "camera_wrist_right"}
        if set(self.names) != expected:
            raise ValueError(f"live policy requires exactly {sorted(expected)}, got {self.names}")
        self.stop_event = mp.Event()
        self.queues = {
            name: mp.Queue(maxsize=max(8, int(camera_config[name]["fps"])))
            for name in self.names
        }
        self.processes: list[mp.Process] = []
        self.synchronizer = CameraSynchronizer(
            self.names,
            max_frames=int(self.config["timing"]["camera_buffer_frames"]),
        )
        self.errors: dict[str, str] = {}
        for name in self.names:
            value = dict(camera_config[name])
            value["jpeg_quality"] = int(self.config.get("jpeg_quality", 92))
            value["encoder_queue_frames"] = int(self.config.get("camera_encoder_queue_frames", 8))
            value["diagnostics"] = dict(self.config.get("camera_diagnostics", {}))
            process = mp.Process(
                target=camera_worker,
                name=f"policy-camera-{name}",
                args=(name, value, self.queues[name], self.stop_event, None),
            )
            process.start()
            self.processes.append(process)
        self._wait_ready(float(self.config["timing"].get("startup_timeout_sec", 20)))

    def _drain(self) -> None:
        for name, camera_queue in self.queues.items():
            while True:
                try:
                    item = camera_queue.get_nowait()
                except queue.Empty:
                    break
                kind = item.get("kind")
                if kind == "frame":
                    self.synchronizer.add(item)
                elif kind == "error":
                    self.errors[name] = str(item.get("error"))

    def _wait_ready(self, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            self._drain()
            if self.errors:
                raise RuntimeError(f"camera startup failed: {self.errors}")
            if all(self.synchronizer.has_any(name) for name in self.names):
                return
            time.sleep(0.02)
        raise RuntimeError("camera startup timeout")

    @staticmethod
    def _decode(sample: dict[str, Any]) -> np.ndarray:
        import cv2

        encoded = np.frombuffer(sample["jpeg"], dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError("camera JPEG decode failed")
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.resize(image, (224, 224), interpolation=cv2.INTER_AREA)
        if image.shape != (224, 224, 3) or image.dtype != np.uint8:
            raise RuntimeError(f"unexpected policy image shape/dtype: {image.shape}/{image.dtype}")
        return image

    def observation_images(self, timeout_sec: float | None = None) -> dict[str, np.ndarray]:
        if timeout_sec is None:
            timeout_sec = float(
                self.config["timing"].get("camera_observation_timeout_sec", 0.7)
            )
        deadline = time.monotonic() + timeout_sec
        timing = self.config["timing"]
        last_status = None
        last_selected = set()
        while time.monotonic() < deadline:
            self._drain()
            if self.errors:
                raise RuntimeError(f"camera runtime error: {self.errors}")
            reference_ns = time.time_ns()
            self.synchronizer.prune(
                reference_ns,
                float(timing["camera_buffer_retention_ms"]),
            )
            selected, status = self.synchronizer.select(
                reference_ns,
                self.names,
                float(timing["camera_stale_after_ms"]),
                timing["wrist_pair"],
            )
            last_status = status
            last_selected = set(selected)
            if len(selected) == 3 and status["wrist_pair"]["valid"]:
                return {
                    f"observation.images.{name}": self._decode(selected[name])
                    for name in self.names
                }
            time.sleep(0.005)
        wp = (last_status or {}).get("wrist_pair", {})
        cg = (last_status or {}).get("camera_group", {})
        missing = sorted(set(self.names) - last_selected)
        missing_str = ",".join(missing) if missing else "none"
        wp_valid = wp.get("valid")
        wp_reason = wp.get("reason")
        wp_skew = wp.get("skew_ms")
        wp_max = wp.get("max_skew_ms")
        cg_valid = cg.get("valid")
        cg_reason = cg.get("reason")
        cg_span = cg.get("span_ms")
        cg_max = cg.get("max_span_ms")
        raise RuntimeError(
            "no fresh synchronized three-camera observation: "
            f"missing={missing_str}; "
            f"wrist_pair(valid={wp_valid}, reason={wp_reason}, "
            f"skew_ms={wp_skew}, max_skew_ms={wp_max}); "
            f"camera_group(valid={cg_valid}, reason={cg_reason}, "
            f"span_ms={cg_span}, max_span_ms={cg_max})"
        )

    def close(self) -> None:
        self.stop_event.set()
        for process in self.processes:
            process.join(timeout=3.0)
            if process.is_alive():
                process.terminate()
                process.join(timeout=1.0)
