from __future__ import annotations

import functools
import time
from typing import Any

import msgpack
import numpy as np


def _pack_array(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        if value.dtype.kind in ("V", "O", "c"):
            raise ValueError(f"unsupported numpy dtype: {value.dtype}")
        return {
            b"__ndarray__": True,
            b"data": value.tobytes(),
            b"dtype": value.dtype.str,
            b"shape": value.shape,
        }
    if isinstance(value, np.generic):
        return {
            b"__npgeneric__": True,
            b"data": value.item(),
            b"dtype": value.dtype.str,
        }
    return value


def _unpack_array(value: dict[bytes, Any]) -> Any:
    if b"__ndarray__" in value:
        return np.ndarray(
            buffer=value[b"data"],
            dtype=np.dtype(value[b"dtype"]),
            shape=value[b"shape"],
        )
    if b"__npgeneric__" in value:
        return np.dtype(value[b"dtype"]).type(value[b"data"])
    return value


Packer = functools.partial(msgpack.Packer, default=_pack_array)
unpackb = functools.partial(msgpack.unpackb, object_hook=_unpack_array)


class PolicyClient:
    def __init__(self, host: str, port: int, *, connect_timeout_sec: float = 10.0) -> None:
        try:
            import websockets.sync.client
        except ImportError as exc:
            raise RuntimeError("websockets with sync client support is required") from exc
        uri = f"ws://{host}:{int(port)}"
        try:
            self._connection = websockets.sync.client.connect(
                uri,
                compression=None,
                max_size=None,
                open_timeout=connect_timeout_sec,
                ping_interval=None,
                ping_timeout=None,
            )
        except TypeError:
            # Older websockets releases do not accept keepalive parameters.
            # Disabling pings prevents a long first-request compile on the
            # server (which blocks its event loop) from being killed by the
            # client's 20 s keepalive ping timeout.
            self._connection = websockets.sync.client.connect(
                uri,
                compression=None,
                max_size=None,
                open_timeout=connect_timeout_sec,
            )
        self.metadata = unpackb(self._connection.recv(timeout=connect_timeout_sec))
        if not isinstance(self.metadata, dict):
            raise RuntimeError("policy server metadata is not a mapping")
        self._packer = Packer()

    def _request(self, payload: dict[str, Any], timeout_sec: float) -> tuple[dict[str, Any], float]:
        started = time.monotonic()
        self._connection.send(self._packer.pack(payload))
        response = self._connection.recv(timeout=timeout_sec)
        elapsed = time.monotonic() - started
        if isinstance(response, str):
            raise RuntimeError(f"policy server returned an error:\n{response}")
        value = unpackb(response)
        if not isinstance(value, dict):
            raise RuntimeError("policy server response is not a mapping")
        return value, elapsed

    def reset(self, robot_config_id: str, timeout_sec: float = 10.0) -> float:
        """Load the robot feature transform before the first policy observation."""
        response, elapsed = self._request(
            {"reset": True, "robo_name": str(robot_config_id)},
            timeout_sec,
        )
        if response.get("action") is not None:
            raise RuntimeError(f"unexpected policy reset response: {response}")
        return elapsed

    def infer(self, observation: dict[str, Any], timeout_sec: float) -> tuple[dict[str, Any], float]:
        return self._request(observation, timeout_sec)

    def close(self) -> None:
        try:
            self._connection.close()
        except Exception:
            pass
