from __future__ import annotations

import json
import sys
from pathlib import Path
from types import SimpleNamespace
from typing import Any

import numpy as np


def prepare_linkerhand_sdk_paths(glove_root: Path) -> None:
    """Expose the vendor SDK's top-level ``core`` package deterministically."""
    paths = (
        glove_root,
        glove_root / "linkerhand_sdk",
        glove_root / "linkerhand_sdk" / "LinkerHand",
    )
    for path in paths:
        value = str(path)
        while value in sys.path:
            sys.path.remove(value)
        sys.path.insert(0, value)


class HandHardwareAdapter:
    def __init__(self, config: dict[str, Any], collector_config: dict[str, Any]) -> None:
        glove_root = Path("/home/svt/glove_control")
        if not glove_root.is_dir():
            glove_root = Path(__file__).resolve().parents[1] / "glove_control"
        prepare_linkerhand_sdk_paths(glove_root)
        from hand_command_arbiter import HandCommandArbiter
        from mixed_glove_teleop import FeedbackSampler, MixedHardware

        hardware_config_path = Path(str(config["hardware_config"]))
        hardware_config = json.loads(hardware_config_path.read_text(encoding="utf-8"))
        args = SimpleNamespace(
            o6_can=str(hardware_config["o6"].get("can", "can1")),
            hand_serial=str(config["wuji_serial"]),
            hardware_limit_margin_rad=float(config.get("hardware_limit_margin_rad", 0.02)),
        )
        self.hardware = MixedHardware(args, hardware_config)
        initial_left, initial_right = self.hardware.open()
        self.feedback = FeedbackSampler(
            self.hardware,
            rate_hz=30.0,
            initial_left=initial_left,
            initial_right=initial_right,
        )
        self.feedback.start()
        self.arbiter = HandCommandArbiter(config.get("right_virtual_to_raw"))
        self.collector_hand_config = collector_config["hands"]
        self.left_command = list(initial_left)
        self.right_command = initial_right.reshape(-1).astype(float).tolist()

    def _project_right(self, raw: np.ndarray) -> list[float]:
        flat = raw.reshape(-1)
        output = []
        for projection in self.collector_hand_config["right_virtual_projection"]:
            output.append(
                sum(
                    float(flat[int(term["index"])]) * float(term["weight"])
                    for term in projection["terms"]
                )
            )
        return output

    def state(self, max_age_sec: float = 0.25) -> np.ndarray:
        import time

        left, right, timestamp_ns, errors, _voltage = self.feedback.snapshot()
        if timestamp_ns is None:
            raise RuntimeError("dual-hand hardware feedback has no timestamp")
        age = (time.time_ns() - timestamp_ns) / 1_000_000_000.0
        if age > max_age_sec or errors:
            raise RuntimeError(f"dual-hand hardware feedback stale/error: age={age:.3f}, {errors}")
        return np.asarray([*left, *self._project_right(right)], dtype=np.float32)

    def publish(self, model_hand_action: Any) -> None:
        left, right = self.arbiter.expand_model_action(
            model_hand_action,
            self.right_command,
        )
        right_array = np.asarray(right, dtype=float).reshape(5, 4)
        right_array = self.hardware.clamp_right(right_array)
        self.hardware.send_o6(left)
        self.hardware.send_wuji(right_array)
        self.left_command = list(left)
        self.right_command = right_array.reshape(-1).tolist()

    def latched_action(self) -> np.ndarray:
        raw = np.asarray(self.right_command, dtype=float).reshape(5, 4)
        return np.asarray([*self.left_command, *self._project_right(raw)], dtype=np.float32)

    def close(self) -> None:
        self.feedback.close()
        self.hardware.close()
