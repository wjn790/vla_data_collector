from __future__ import annotations

import queue
import time
import traceback
from pathlib import Path
from typing import Any

from raw_recording import RawCameraWriter


def _put_latest(output_queue: Any, item: dict[str, Any]) -> None:
    while True:
        try:
            output_queue.put_nowait(item)
            return
        except queue.Full:
            try:
                output_queue.get_nowait()
            except queue.Empty:
                return


def _put_diagnostic(output_queue: Any, item: dict[str, Any]) -> None:
    """Do not evict frame metadata just to report a diagnostic event."""
    try:
        output_queue.put_nowait(item)
    except queue.Full:
        pass


def _queue_depth(value: Any) -> int | None:
    try:
        return max(0, int(value.qsize()))
    except (AttributeError, NotImplementedError, OSError):
        return None


def frame_gap_diagnostics(
    previous_timestamp_ns: int | None,
    timestamp_ns: int,
    fps: float,
    warning_threshold_ms: float,
) -> dict[str, Any]:
    expected_ms = 1000.0 / max(float(fps), 1e-6)
    gap_ms = (
        None
        if previous_timestamp_ns is None
        else (int(timestamp_ns) - int(previous_timestamp_ns)) / 1_000_000.0
    )
    estimated_missing = 0
    if gap_ms is not None and gap_ms > 1.5 * expected_ms:
        estimated_missing = max(0, int(round(gap_ms / expected_ms)) - 1)
    return {
        "expected_frame_period_ms": expected_ms,
        "hardware_timestamp_gap_ms": gap_ms,
        "estimated_missing_frames": estimated_missing,
        "hardware_gap_warning": bool(
            gap_ms is not None and gap_ms > float(warning_threshold_ms)
        ),
        "hardware_gap_warning_threshold_ms": float(warning_threshold_ms),
    }


def _encode(image: Any, quality: int) -> bytes:
    import cv2

    ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    if not ok:
        raise RuntimeError("cv2.imencode returned false")
    return encoded.tobytes()


def _emit_frame(
    output_queue: Any,
    sample: dict[str, Any],
    raw_writer: RawCameraWriter | None,
) -> None:
    if raw_writer is not None:
        sample = raw_writer.write(sample)
    _put_latest(output_queue, sample)


def _encoder_worker(
    name: str,
    config: dict[str, Any],
    input_queue: Any,
    output_queue: Any,
    episode_dir: str | None,
    error_event: Any,
) -> None:
    """Encode and persist frames outside the camera grab process."""
    output_queue.cancel_join_thread()
    raw_writer = RawCameraWriter(Path(episode_dir), name) if episode_dir else None
    try:
        while True:
            sample = input_queue.get()
            if sample is None:
                break
            encoding_started_ns = time.time_ns()
            queued_ns = int(sample.get("encoding_queued_ns", encoding_started_ns))
            sample["encoding_started_ns"] = encoding_started_ns
            sample["encoder_queue_delay_ms"] = max(
                0.0, (encoding_started_ns - queued_ns) / 1_000_000.0
            )
            image = sample.pop("_image")
            if sample.pop("_bgra", False):
                import cv2

                image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
            sample["jpeg"] = _encode(image, int(config.get("jpeg_quality", 92)))
            encoding_completed_ns = time.time_ns()
            sample["encoding_completed_ns"] = encoding_completed_ns
            sample["encoding_duration_ms"] = max(
                0.0, (encoding_completed_ns - encoding_started_ns) / 1_000_000.0
            )
            sample["capture_to_encoding_complete_ms"] = max(
                0.0,
                (encoding_completed_ns - int(sample["host_timestamp_ns"]))
                / 1_000_000.0,
            )
            _emit_frame(output_queue, sample, raw_writer)
    except BaseException as exc:
        error_event.set()
        _put_latest(
            output_queue,
            {
                "kind": "error",
                "camera": name,
                "error": f"encoder {type(exc).__name__}: {exc}",
                "traceback": traceback.format_exc(),
            },
        )
    finally:
        if raw_writer is not None:
            raw_writer.close()


def _submit_frame(
    name: str,
    image_queue: Any,
    output_queue: Any,
    sample: dict[str, Any],
    image: Any,
    *,
    bgra: bool = False,
    dropped_count: list[int],
    error_event: Any,
) -> None:
    if error_event.is_set():
        raise RuntimeError(f"{name} encoder process failed")
    sample["encoding_queued_ns"] = time.time_ns()
    sample["encoder_queue_depth_at_submit"] = _queue_depth(image_queue)
    sample["_image"] = image
    sample["_bgra"] = bgra
    try:
        image_queue.put_nowait(sample)
    except queue.Full:
        dropped_count[0] += 1
        _put_latest(
            output_queue,
            {
                "kind": "capture_drop",
                "camera": name,
                "dropped_count": dropped_count[0],
            },
        )


def _capture_realsense(
    name: str,
    config: dict[str, Any],
    output_queue: Any,
    stop_event: Any,
    image_queue: Any,
    dropped_count: list[int],
    encoder_error_event: Any,
) -> None:
    import pyrealsense2 as rs

    pipeline = rs.pipeline()
    pipeline_config = rs.config()
    pipeline_config.enable_device(str(config["serial"]))
    pipeline_config.enable_stream(
        rs.stream.color,
        int(config["width"]),
        int(config["height"]),
        rs.format.bgr8,
        int(config["fps"]),
    )
    profile = pipeline.start(pipeline_config)
    device = profile.get_device()
    actual_serial = device.get_info(rs.camera_info.serial_number)
    if actual_serial != str(config["serial"]):
        pipeline.stop()
        raise RuntimeError(f"opened serial {actual_serial}, expected {config['serial']}")

    for sensor in device.query_sensors():
        if sensor.supports(rs.option.global_time_enabled):
            sensor.set_option(rs.option.global_time_enabled, 1)

    diagnostics = dict(config.get("diagnostics", {}))
    gap_warning_ms = float(diagnostics.get("hardware_gap_warn_ms", 50.0))
    previous_hardware_timestamp_ns: int | None = None
    capture_sequence = 0
    _put_latest(output_queue, {"kind": "status", "camera": name, "status": "ready"})
    try:
        while not stop_event.is_set():
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            color = frames.get_color_frame()
            if not color:
                continue
            import numpy as np

            image = np.asanyarray(color.get_data()).copy()
            host_timestamp_ns = time.time_ns()
            hardware_timestamp_ns = int(float(color.get_timestamp()) * 1_000_000)
            timing = frame_gap_diagnostics(
                previous_hardware_timestamp_ns,
                hardware_timestamp_ns,
                float(config["fps"]),
                gap_warning_ms,
            )
            previous_hardware_timestamp_ns = hardware_timestamp_ns
            _submit_frame(
                name,
                image_queue,
                output_queue,
                {
                    "kind": "frame",
                    "camera": name,
                    "capture_sequence": capture_sequence,
                    "host_timestamp_ns": host_timestamp_ns,
                    "hardware_timestamp_ns": hardware_timestamp_ns,
                    "hardware_timestamp_domain": str(color.get_frame_timestamp_domain()),
                    "width": int(image.shape[1]),
                    "height": int(image.shape[0]),
                    **timing,
                },
                image,
                dropped_count=dropped_count,
                error_event=encoder_error_event,
            )
            capture_sequence += 1
    finally:
        pipeline.stop()


def _zed_resolution(sl: Any, name: str) -> Any:
    value = str(name).upper()
    options = {
        "VGA": sl.RESOLUTION.VGA,
        "HD720": sl.RESOLUTION.HD720,
        "HD1080": sl.RESOLUTION.HD1080,
        "HD2K": sl.RESOLUTION.HD2K,
    }
    if value not in options:
        raise ValueError(f"unsupported ZED resolution: {name}")
    return options[value]


def _capture_zed(
    name: str,
    config: dict[str, Any],
    output_queue: Any,
    stop_event: Any,
    image_queue: Any,
    dropped_count: list[int],
    encoder_error_event: Any,
    ready_event: Any | None = None,
) -> None:
    import pyzed.sl as sl

    init = sl.InitParameters()
    init.set_from_serial_number(int(config["serial"]))
    init.camera_resolution = _zed_resolution(sl, str(config.get("resolution", "VGA")))
    init.camera_fps = int(config["fps"])
    init.depth_mode = sl.DEPTH_MODE.NONE
    init.sensors_required = bool(config.get("sensors_required", False))
    open_attempts = max(1, int(config.get("open_retry_count", 1)))
    open_retry_delay_sec = max(0.0, float(config.get("open_retry_delay_sec", 1.0)))
    camera = None
    status = None
    for attempt in range(1, open_attempts + 1):
        candidate = sl.Camera()
        status = candidate.open(init)
        if status == sl.ERROR_CODE.SUCCESS:
            camera = candidate
            break
        try:
            candidate.close()
        except Exception:
            pass
        if attempt < open_attempts:
            print(
                f"[camera-warning] {name} open attempt {attempt}/{open_attempts} "
                f"failed: {status}; retrying in {open_retry_delay_sec:.1f}s",
                flush=True,
            )
            if stop_event.wait(open_retry_delay_sec):
                raise RuntimeError("ZED open cancelled during shutdown")
    if camera is None:
        raise RuntimeError(f"ZED open failed after {open_attempts} attempts: {status}")
    if ready_event is not None:
        ready_event.set()

    image = sl.Mat()
    runtime = sl.RuntimeParameters()
    diagnostics = dict(config.get("diagnostics", {}))
    gap_warning_ms = float(diagnostics.get("hardware_gap_warn_ms", 50.0))
    previous_hardware_timestamp_ns: int | None = None
    capture_sequence = 0
    grab_error_count = 0
    grab_error_codes: dict[str, int] = {}
    last_error_emit_monotonic = 0.0
    _put_latest(output_queue, {"kind": "status", "camera": name, "status": "ready"})
    try:
        while not stop_event.is_set():
            grab_started_ns = time.monotonic_ns()
            grab_status = camera.grab(runtime)
            grab_duration_ms = (time.monotonic_ns() - grab_started_ns) / 1_000_000.0
            if grab_status != sl.ERROR_CODE.SUCCESS:
                grab_error_count += 1
                status_name = str(grab_status)
                grab_error_codes[status_name] = grab_error_codes.get(status_name, 0) + 1
                now_monotonic = time.monotonic()
                if grab_error_count == 1 or now_monotonic - last_error_emit_monotonic >= 1.0:
                    _put_diagnostic(
                        output_queue,
                        {
                            "kind": "camera_diagnostic",
                            "camera": name,
                            "event": "zed_grab_error",
                            "timestamp_ns": time.time_ns(),
                            "status": status_name,
                            "grab_duration_ms": grab_duration_ms,
                            "zed_grab_error_count": grab_error_count,
                            "zed_grab_error_codes": dict(grab_error_codes),
                        },
                    )
                    last_error_emit_monotonic = now_monotonic
                continue
            camera.retrieve_image(image, sl.VIEW.LEFT)
            bgra = image.get_data().copy()
            hardware_timestamp_ns = int(
                camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
            )
            timing = frame_gap_diagnostics(
                previous_hardware_timestamp_ns,
                hardware_timestamp_ns,
                float(config["fps"]),
                gap_warning_ms,
            )
            previous_hardware_timestamp_ns = hardware_timestamp_ns
            _submit_frame(
                name,
                image_queue,
                output_queue,
                {
                    "kind": "frame",
                    "camera": name,
                    "capture_sequence": capture_sequence,
                    "host_timestamp_ns": time.time_ns(),
                    "hardware_timestamp_ns": hardware_timestamp_ns,
                    "hardware_timestamp_domain": "zed_global_time",
                    "width": int(bgra.shape[1]),
                    "height": int(bgra.shape[0]),
                    "grab_duration_ms": grab_duration_ms,
                    "zed_grab_error_count": grab_error_count,
                    "zed_grab_error_codes": dict(grab_error_codes),
                    **timing,
                },
                bgra,
                bgra=True,
                dropped_count=dropped_count,
                error_event=encoder_error_event,
            )
            capture_sequence += 1
    finally:
        camera.close()


def _capture_v4l2(
    name: str,
    config: dict[str, Any],
    output_queue: Any,
    stop_event: Any,
    image_queue: Any,
    dropped_count: list[int],
    encoder_error_event: Any,
) -> None:
    import cv2

    device = str(config["device"])
    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not capture.isOpened():
        raise RuntimeError(f"cannot open V4L2 device {device}")
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(config.get("capture_width", config["width"])))
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(config.get("capture_height", config["height"])))
    capture.set(cv2.CAP_PROP_FPS, int(config["fps"]))
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    expected_width = int(config.get("capture_width", config["width"]))
    expected_height = int(config.get("capture_height", config["height"]))
    diagnostics = dict(config.get("diagnostics", {}))
    gap_warning_ms = float(diagnostics.get("hardware_gap_warn_ms", 50.0))
    previous_timestamp_ns: int | None = None
    capture_sequence = 0
    _put_latest(output_queue, {"kind": "status", "camera": name, "status": "ready"})
    try:
        while not stop_event.is_set():
            ok, frame = capture.read()
            host_timestamp_ns = time.time_ns()
            if not ok or frame is None:
                raise RuntimeError(f"read failed from V4L2 device {device}")
            if frame.shape[1] != expected_width or frame.shape[0] != expected_height:
                raise RuntimeError(
                    f"V4L2 returned {frame.shape[1]}x{frame.shape[0]}, "
                    f"expected {expected_width}x{expected_height}"
                )
            crop = str(config.get("crop", "none"))
            if crop == "left_half":
                frame = frame[:, : frame.shape[1] // 2]
            elif crop == "right_half":
                frame = frame[:, frame.shape[1] // 2 :]
            elif crop != "none":
                raise ValueError(f"unsupported V4L2 crop: {crop}")
            timing = frame_gap_diagnostics(
                previous_timestamp_ns,
                host_timestamp_ns,
                float(config["fps"]),
                gap_warning_ms,
            )
            previous_timestamp_ns = host_timestamp_ns
            _submit_frame(
                name,
                image_queue,
                output_queue,
                {
                    "kind": "frame",
                    "camera": name,
                    "capture_sequence": capture_sequence,
                    "host_timestamp_ns": host_timestamp_ns,
                    "hardware_timestamp_ns": None,
                    "hardware_timestamp_domain": "v4l2_host_receive_clock",
                    "width": int(frame.shape[1]),
                    "height": int(frame.shape[0]),
                    **timing,
                },
                frame,
                dropped_count=dropped_count,
                error_event=encoder_error_event,
            )
            capture_sequence += 1
    finally:
        capture.release()


def _capture_mock(
    name: str,
    config: dict[str, Any],
    output_queue: Any,
    stop_event: Any,
    image_queue: Any,
    dropped_count: list[int],
    encoder_error_event: Any,
) -> None:
    import cv2
    import numpy as np

    width = int(config.get("width", 640))
    height = int(config.get("height", 480))
    fps = float(config.get("fps", 15))
    period = 1.0 / fps
    frame_index = 0
    diagnostics = dict(config.get("diagnostics", {}))
    gap_warning_ms = float(diagnostics.get("hardware_gap_warn_ms", 50.0))
    previous_timestamp_ns: int | None = None
    _put_latest(output_queue, {"kind": "status", "camera": name, "status": "ready"})
    while not stop_event.is_set():
        started = time.monotonic()
        image = np.zeros((height, width, 3), dtype=np.uint8)
        image[:, :, 1] = (frame_index * 3) % 255
        cv2.putText(
            image,
            f"{name} {frame_index}",
            (20, max(40, height // 2)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
        )
        timestamp_ns = time.time_ns()
        timing = frame_gap_diagnostics(
            previous_timestamp_ns,
            timestamp_ns,
            fps,
            gap_warning_ms,
        )
        previous_timestamp_ns = timestamp_ns
        _submit_frame(
            name,
            image_queue,
            output_queue,
            {
                "kind": "frame",
                "camera": name,
                "capture_sequence": frame_index,
                "host_timestamp_ns": timestamp_ns,
                "hardware_timestamp_ns": timestamp_ns,
                "hardware_timestamp_domain": "mock_system_clock",
                "width": width,
                "height": height,
                **timing,
            },
            image,
            dropped_count=dropped_count,
            error_event=encoder_error_event,
        )
        frame_index += 1
        stop_event.wait(max(0.0, period - (time.monotonic() - started)))


def camera_worker(
    name: str,
    config: dict[str, Any],
    output_queue: Any,
    stop_event: Any,
    episode_dir: str | None = None,
    ready_event: Any | None = None,
) -> None:
    # Do not make process shutdown wait for the multiprocessing queue feeder to
    # flush obsolete camera frames after the parent has stopped consuming.
    output_queue.cancel_join_thread()
    import multiprocessing as mp

    image_queue = mp.Queue(maxsize=max(2, int(config.get("encoder_queue_frames", 8))))
    encoder_error_event = mp.Event()
    encoder = mp.Process(
        target=_encoder_worker,
        name=f"camera-encoder-{name}",
        args=(name, config, image_queue, output_queue, episode_dir, encoder_error_event),
    )
    encoder.start()
    dropped_count = [0]
    try:
        backend = str(config["backend"]).lower()
        if backend == "realsense":
            _capture_realsense(
                name,
                config,
                output_queue,
                stop_event,
                image_queue,
                dropped_count,
                encoder_error_event,
            )
        elif backend == "zed":
            _capture_zed(
                name,
                config,
                output_queue,
                stop_event,
                image_queue,
                dropped_count,
                encoder_error_event,
                ready_event,
            )
        elif backend == "v4l2":
            _capture_v4l2(
                name,
                config,
                output_queue,
                stop_event,
                image_queue,
                dropped_count,
                encoder_error_event,
            )
        elif backend == "mock":
            _capture_mock(
                name,
                config,
                output_queue,
                stop_event,
                image_queue,
                dropped_count,
                encoder_error_event,
            )
        else:
            raise ValueError(f"unknown camera backend: {backend}")
    except BaseException as exc:
        _put_latest(
            output_queue,
            {
                "kind": "error",
                "camera": name,
                "error": f"{type(exc).__name__}: {exc}",
                "traceback": traceback.format_exc(),
            },
        )
    finally:
        # The sentinel is queued after all accepted frames, so shutdown flushes
        # the short encoder backlog without changing capture timestamps.
        try:
            image_queue.put(
                None,
                timeout=float(config.get("encoder_sentinel_timeout_sec", 0.5)),
            )
        except queue.Full:
            encoder.terminate()
        encoder.join(timeout=float(config.get("encoder_shutdown_timeout_sec", 2.0)))
        if encoder.is_alive():
            encoder.terminate()
            encoder.join(timeout=1)
        image_queue.close()
