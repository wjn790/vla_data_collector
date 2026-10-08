#!/usr/bin/env python3
"""SVTROBO Web Control Server - aiohttp backend for camera MJPEG streaming + static files."""

import argparse
import asyncio
import json
import logging
import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time
import traceback
from datetime import datetime
from pathlib import Path

import cv2
from aiohttp import web, WSMsgType

from teleop_log_utils import copy_teleop_window

try:
    import aiohttp
except ImportError:
    aiohttp = None

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')
logger = logging.getLogger(__name__)

BASE_DIR = Path(__file__).parent
STATIC_DIR = BASE_DIR / 'static'

# --- Camera Configuration ---
CAMERA_CONFIG = {
    'd405_1': {'type': 'realsense', 'serial': '409122272399', 'size': (1280, 720), 'fps': 5, 'depth': True},
    'd405_2': {'type': 'realsense', 'serial': '409122273344', 'size': (1280, 720), 'fps': 5, 'depth': True},
    'zed':    {'type': 'zed',       'serial': None,           'size': None,       'fps': 15, 'depth': True, 'resolution': 'HD720'},
}

JPEG_QUALITY = 95

# --- Point Cloud Saving Options ---
PC_DOWNSAMPLE = 1          # Downsample factor for point cloud (1=full, 2=half, 3=third, etc.)
PC_DTYPE = 'float16'       # Point cloud numpy dtype: 'float16' (half size) or 'float32' (full precision)
STREAM_FPS = 15

# Recording: deadline-based 2Hz frame saving
RECORD_INTERVAL = 0.5  # seconds between saved frames
IMU_DATA_HZ = 10       # IMU recording frequency (Hz)
ROS_DATA_HZ = 10       # ROS2 topic recording frequency (Hz)
TELEOP_ACTION_LOG_GLOB = os.environ.get(
    'SVTROBO_TELEOP_ACTION_LOG_GLOB',
    '/home/svt/glove_control/logs/teleop_actions_*.jsonl',
)
TELEOP_ACTION_SESSION_FILE = 'teleop_actions.jsonl'

VISUAL_RELOCALIZATION_TARGET_RE = re.compile(r"^[A-Za-z0-9_-]{1,64}$")
VISUAL_RELOCALIZATION_RECORD_TIMEOUT_SEC = 30
VISUAL_RELOCALIZATION_LOG_LIMIT = 120


def _atomic_write_array(path, array):
    tmp_path = f'{path}.tmp'
    array.tofile(tmp_path)
    os.replace(tmp_path, path)


def _atomic_write_json(path, payload):
    tmp_path = f'{path}.tmp'
    with open(tmp_path, 'w') as fp:
        json.dump(payload, fp)
    os.replace(tmp_path, path)


class CameraManager:
    """Manages camera instances, capture threads, and frame queues."""

    def __init__(self):
        self.cameras = {}  # name -> {instance, thread, stop_event, frame_queue, running}
        self._lock = threading.Lock()

    def start_camera(self, name):
        """Start a camera capture thread."""
        if name not in CAMERA_CONFIG:
            return False, f"Unknown camera: {name}"

        with self._lock:
            if name in self.cameras and self.cameras[name]['running']:
                return True, f"Camera {name} already running"

            cfg = CAMERA_CONFIG[name]
            try:
                # Import camera drivers
                sys.path.insert(0, str(Path(__file__).parent.parent / 'camera_driver'))
                from camera_driver import RealSenseCamera, ZEDCamera

                if cfg['type'] == 'realsense':
                    cam = RealSenseCamera(
                        serial=cfg['serial'],
                        color_size=cfg['size'],
                        depth_size=cfg['size'],
                        fps=cfg['fps'],
                        color_only=not cfg.get('depth', False),
                        warmup_frames=cfg.get('warmup_frames'),
                    )
                else:
                    # Stop IMU-only reader to avoid ZED device conflict
                    stop_imu_reader()
                    cam = ZEDCamera(
                        resolution=cfg.get('resolution', 'HD720'),
                        fps=cfg['fps'],
                        color_only=not cfg.get('depth', False),
                    )

                cam.start()

                # Save ZED intrinsics to shared memory for inference pipeline
                if name == 'zed':
                    try:
                        _intr = cam.get_intrinsics()
                        if _intr:
                            _atomic_write_json('/dev/shm/zed_intrinsics.json', _intr)
                    except Exception:
                        pass

                stop_event = threading.Event()
                frame_queue = queue.Queue(maxsize=2)
                right_queue = queue.Queue(maxsize=2) if (name == 'zed') else None
                depth_queue = queue.Queue(maxsize=2) if cfg.get('depth') else None
                t = threading.Thread(
                    target=self._capture_loop,
                    args=(name, cam, frame_queue, stop_event, depth_queue, right_queue),
                    daemon=True,
                )
                t.start()

                self.cameras[name] = {
                    'instance': cam,
                    'thread': t,
                    'stop_event': stop_event,
                    'frame_queue': frame_queue,
                    'right_queue': right_queue,
                    'depth_queue': depth_queue,
                    'running': True,
                }
                logger.info(f"Camera {name} started")
                return True, f"Camera {name} started"

            except Exception as e:
                logger.error(f"Failed to start camera {name}: {e}")
                traceback.print_exc()
                # If ZED failed to start, restart IMU-only reader
                if name == 'zed':
                    start_imu_reader()
                return False, str(e)

    def stop_camera(self, name):
        """Stop a camera and free resources."""
        with self._lock:
            if name not in self.cameras:
                return True, f"Camera {name} already stopped"

            info = self.cameras[name]
            if not info['running']:
                return True, f"Camera {name} already stopped"

            info['stop_event'].set()
            info['thread'].join(timeout=5.0)
            try:
                info['instance'].stop()
            except Exception:
                pass
            info['running'] = False
            del self.cameras[name]
            # Restart IMU-only reader and clear cache when ZED stops
            if name == 'zed':
                start_imu_reader()
                with imu_cache['lock']:
                    imu_cache['data'] = None
            logger.info(f"Camera {name} stopped")
            return True, f"Camera {name} stopped"

    def get_status(self):
        """Return status of all cameras including hardware presence."""
        import subprocess
        lsusb_out = ""
        try:
            lsusb_out = subprocess.check_output(["lsusb"], text=True).lower()
        except Exception:
            pass

        result = {}
        for name, cfg in CAMERA_CONFIG.items():
            running = name in self.cameras and self.cameras[name]["running"]
            device = False
            if cfg["type"] == "realsense":
                try:
                    import pyrealsense2 as rs
                    for d in rs.context().query_devices():
                        if d.get_info(rs.camera_info.serial_number) == cfg["serial"]:
                            device = True
                            break
                except Exception:
                    pass
            elif cfg["type"] == "zed":
                device = "2b03:f880" in lsusb_out or "stereolabs" in lsusb_out
            result[name] = {"running": running, "device": device}
        return result

    def get_frame(self, name):
        """Get the latest JPEG frame and timestamp for a camera (non-blocking).

        Returns:
            (jpeg_bytes, timestamp_us) or None
        """
        info = self.cameras.get(name)
        if not info or not info.get('running'):
            return None
        try:
            return info['frame_queue'].get_nowait()
        except queue.Empty:
            return None

    def peek_frame(self, name):
        """Return the newest frame without consuming it from the recording queue."""
        info = self.cameras.get(name)
        if not info or not info.get('running'):
            return None
        fq = info.get('frame_queue')
        if fq is None:
            return None
        try:
            with fq.mutex:
                if not fq.queue:
                    return None
                return fq.queue[-1]
        except Exception:
            return None

    def get_depth_frame(self, name):
        """Get the latest depth JPEG frame and timestamp for a camera (non-blocking).

        Returns:
            (jpeg_bytes, timestamp_us) or None
        """
        if name not in self.cameras or not self.cameras[name]['running']:
            return None
        dq = self.cameras[name].get('depth_queue')
        if dq is None:
            return None
        try:
            return dq.get_nowait()
        except queue.Empty:
            return None

    def get_right_frame(self, name):
        """Get the latest right eye JPEG frame and timestamp for ZED (non-blocking).

        Returns:
            (jpeg_bytes, timestamp_us) or None
        """
        if name not in self.cameras or not self.cameras[name]['running']:
            return None
        rq = self.cameras[name].get('right_queue')
        if rq is None:
            return None
        try:
            return rq.get_nowait()
        except queue.Empty:
            return None

    @staticmethod
    def _capture_loop(name, cam, frame_queue, stop_event, depth_queue=None, right_queue=None):
        """Background thread: continuously capture frames and encode as JPEG.
        
        depth_queue: if provided, raw depth frames are queued for recording.
        right_queue: if provided (ZED only), right eye JPEG frames are queued.
        Point cloud is retrieved inside _capture_sdk() and stored in cam._latest_pc.
        """
        frame_count = 0

        while not stop_event.is_set():
            try:
                # For ZED with stereo, use _capture_sdk directly to get (left, right, depth)
                if right_queue is not None and hasattr(cam, '_capture_sdk'):
                    result = cam._capture_sdk()
                    right_img = result[1] if len(result) == 3 else None
                    depth_raw = result[2] if len(result) == 3 else result[1]
                else:
                    result = cam.capture()
                    right_img = None
                    depth_raw = result[1] if result else None

                if result is None:
                    continue
                color = result[0]
                if color is None:
                    continue

                # Store raw BGR numpy (no JPEG encode here - encoding moved to save/stream)
                timestamp_us = int(time.time() * 1_000_000)

                # Drop old frame if queue is full
                try:
                    frame_queue.get_nowait()
                except queue.Empty:
                    pass
                frame_queue.put((color, timestamp_us))

                # Queue raw right eye BGR for ZED stereo recording
                if right_queue is not None and right_img is not None:
                    try:
                        try:
                            right_queue.get_nowait()
                        except queue.Empty:
                            pass
                        right_queue.put((right_img, timestamp_us))
                    except Exception as e:
                        logger.debug(f"Camera {name} right queue error: {e}")

                # Save raw depth for recording
                if depth_queue is not None and depth_raw is not None:
                    try:
                        try:
                            depth_queue.get_nowait()
                        except queue.Empty:
                            pass
                        depth_queue.put((depth_raw, timestamp_us))
                    except Exception as e:
                        logger.debug(f"Camera {name} depth queue error: {e}")

                # Write raw data to shared memory for DiffusionVN inference pipeline
                if name == 'zed' and color is not None:
                    try:
                        _atomic_write_array('/dev/shm/zed_color.raw', color)
                        if depth_raw is not None:
                            _atomic_write_array('/dev/shm/zed_depth.raw', depth_raw)
                        _atomic_write_json('/dev/shm/zed_meta.json', {
                            'color_shape': list(color.shape),
                            'depth_shape': list(depth_raw.shape) if depth_raw is not None else None,
                            'depth_dtype': str(depth_raw.dtype) if depth_raw is not None else None,
                            'timestamp': timestamp_us,
                        })
                    except Exception as e:
                        logger.debug(f"ZED shared memory write error: {e}")

                frame_count += 1

                # Update IMU cache after each grab (ZED sensors refresh on grab)
                if name == 'zed' and hasattr(cam, 'get_imu_data'):
                    try:
                        imu_data = cam.get_imu_data()
                        if imu_data is not None:
                            imu_data = dict(imu_data)
                            imu_data["server_ts"] = time.time()
                            with imu_cache['lock']:
                                imu_cache['data'] = imu_data
                            _atomic_write_json('/dev/shm/zed_imu.json', imu_data)
                            # Push to recording queue (dedup handled by grab-tied refresh)
                            sample = dict(imu_data)
                            try:
                                imu_rec_queue.put_nowait(sample)
                            except Exception:
                                pass
                    except Exception:
                        pass

            except Exception as e:
                logger.warning(f"Camera {name} capture error: {e}")
                stop_event.wait(0.1)

        logger.info(f"Camera {name} capture thread exiting")


    def _force_recover_zed(self):
        """Hard destroy and recreate ZED camera instance.

        Used when ZED enters a bad state (e.g. from rapid start/stop cycles).
        Completely tears down the old instance and creates a fresh one.
        Returns True on success.
        """
        logger.warning("Force recovering ZED camera...")

        # Step 1: Stop and remove existing ZED if present
        if 'zed' in self.cameras:
            try:
                info = self.cameras['zed']
                info['stop_event'].set()
                info['thread'].join(timeout=5.0)
                try:
                    info['instance'].stop()
                except Exception:
                    pass
                del self.cameras['zed']
                logger.info("Force recover: old ZED instance removed")
            except Exception as e:
                logger.warning(f"Force recover: error removing old ZED: {e}")

        # Step 2: Ensure IMU reader is stopped (it holds ZED open)
        stop_imu_reader()
        time.sleep(0.5)  # Let USB device fully release

        # Step 3: Fresh ZED open
        try:
            sys.path.insert(0, str(Path(__file__).parent.parent / 'camera_driver'))
            from camera_driver import ZEDCamera

            cam = ZEDCamera(
                resolution=CAMERA_CONFIG['zed'].get('resolution', 'HD720'),
                fps=CAMERA_CONFIG['zed']['fps'],
                color_only=not CAMERA_CONFIG['zed'].get('depth', False),
            )
            cam.start()

            stop_event = threading.Event()
            frame_queue = queue.Queue(maxsize=2)
            right_queue = queue.Queue(maxsize=2)
            depth_queue = queue.Queue(maxsize=2) if CAMERA_CONFIG['zed'].get('depth') else None
            t = threading.Thread(
                target=self._capture_loop,
                args=('zed', cam, frame_queue, stop_event, depth_queue, right_queue),
                daemon=True,
            )
            t.start()

            self.cameras['zed'] = {
                'instance': cam,
                'thread': t,
                'stop_event': stop_event,
                'frame_queue': frame_queue,
                'right_queue': right_queue,
                'depth_queue': depth_queue,
                'running': True,
            }
            logger.info("Force recover: ZED camera recreated successfully")
            return True
        except Exception as e:
            logger.error(f"Force recover failed: {e}")
            # Restart IMU reader as fallback
            start_imu_reader()
            return False

    def _check_zed_healthy(self, timeout=3.0):
        """Verify ZED camera is actually producing frames.

        Waits up to timeout seconds for at least one frame from the ZED capture queue.
        Returns True if frames are flowing.
        """
        if 'zed' not in self.cameras or not self.cameras['zed']['running']:
            return False

        fq = self.cameras['zed']['frame_queue']
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                with fq.mutex:
                    if fq.queue:
                        return True
            except Exception:
                pass
            time.sleep(0.1)

        logger.warning("ZED health check: no frames received within timeout")
        return False

    def stop_all(self):
        """Stop all running cameras."""
        for name in list(self.cameras.keys()):
            self.stop_camera(name)


# --- Recording Manager ---


RECORDING_DIR = Path('/svtrobo_data/recordings')

# Topics to record via ros2 bag
RECORD_TOPICS = [
    '/svtrobot_cmd',
    '/lift_control_cmd',
    '/chassis/joint_states',
    '/chassis/diagnostics',
    '/f710/joy',
    # Arm teleoperation commands and controller feedback
    '/joint_states',
    '/dynamic_joint_states',
    '/exo/joint_command',
    '/exo/gamepad_keys',
    '/exo_retargeting/status',
    '/left_arm/joint_command',
    '/right_arm/joint_command',
    # LinkerHand O6 commands, state, and diagnostics/info
    '/cb_left_hand_state',
    '/cb_right_hand_state',
    '/cb_left_hand_control_cmd',
    '/cb_right_hand_control_cmd',
    '/cb_left_hand_info',
    '/cb_right_hand_info',
    '/cb_hand_setting_cmd',
    # IMU data saved directly to imu.jsonl (no ROS2 publisher for these topics)
    # '/zed/imu/data',
    # '/zed/imu/mag',
    # '/zed/imu/temperature',
]

ROS_TOPIC_STATUS_MAP = {
    'ros_cmd': '/svtrobot_cmd',
    'ros_joy': '/f710/joy',
    'ros_lift': '/lift_control_cmd',
    'ros_chassis': '/chassis/joint_states',
    'ros_diag': '/chassis/diagnostics',
    'arm_joint_states': '/joint_states',
    'arm_dynamic_joint_states': '/dynamic_joint_states',
    'exo_joint_command': '/exo/joint_command',
    'exo_gamepad_keys': '/exo/gamepad_keys',
    'exo_retargeting_status': '/exo_retargeting/status',
    'left_arm_joint_command': '/left_arm/joint_command',
    'right_arm_joint_command': '/right_arm/joint_command',
    'left_hand_state': '/cb_left_hand_state',
    'right_hand_state': '/cb_right_hand_state',
    'left_hand_control_cmd': '/cb_left_hand_control_cmd',
    'right_hand_control_cmd': '/cb_right_hand_control_cmd',
    'left_hand_info': '/cb_left_hand_info',
    'right_hand_info': '/cb_right_hand_info',
    'hand_setting_cmd': '/cb_hand_setting_cmd',
}

# After recording stops, convert .db3 to JSONL and delete the original db file
DELETE_DB_AFTER_CONVERT = False


class RecordingManager:
    """Manages data collection: ros2 bag subprocess + periodic camera frame saving."""

    def __init__(self, camera_mgr: CameraManager):
        self.camera_mgr = camera_mgr
        self.running = False
        self.bag_process = None
        self.save_thread = None
        self.imu_save_thread = None
        self.distance_save_thread = None
        self.stop_event = threading.Event()
        self.output_dir = None
        self.start_time = None
        self.start_time_ns = None
        self._lock = threading.Lock()
        # Track cameras started by recording (so we only stop those we started)
        self._cameras_started = set()

    def start(self, test_mode=False):
        """Start data recording. Returns (ok, message, path)."""
        with self._lock:
            if self.running:
                return False, 'Already recording', None

            timestamp = datetime.now().strftime('%Y-%m-%d')
            time_str = datetime.now().strftime('%H%M%S')
            if test_mode:
                self.output_dir = RECORDING_DIR / '_test' / timestamp / time_str
            else:
                self.output_dir = RECORDING_DIR / timestamp / time_str
            # Pre-check disk space
            try:
                stat = os.statvfs(str(RECORDING_DIR))
                free_mb = (stat.f_bavail * stat.f_frsize) / (1024 * 1024)
                if free_mb < 500:
                    return False, f'Disk space low ({free_mb:.0f}MB free), cannot start recording', None
            except Exception:
                pass

            self.output_dir.mkdir(parents=True, exist_ok=True)

            # Start ros2 bag record
            bag_dir = self.output_dir / 'rosbag'
            try:
                cmd = (
                    'source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=56 && export ROS_LOCALHOST_ONLY=1 && '
                    'source /home/svt/svtrobo_ws/install/setup.bash && '
                    'exec ros2 bag record ' + ' '.join(RECORD_TOPICS) + f' -o {bag_dir}'
                )
                self.bag_process = subprocess.Popen(
                    ['bash', '-c', cmd],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                logger.info(f"ros2 bag record started, saving to {bag_dir}")
            except FileNotFoundError:
                return False, 'ros2 command not found. Is ROS2 sourced?', None
            except Exception as e:
                return False, str(e), None

            # Stop IMU reader to avoid ZED device conflict before starting cameras
            if 'zed' in CAMERA_CONFIG:
                stop_imu_reader()

            # Auto-start all cameras (skip those already running)
            self._cameras_started = set()
            cam_start_errors = []
            for cam_name in CAMERA_CONFIG:
                status = self.camera_mgr.get_status().get(cam_name, {})
                if not status.get('running', False):
                    ok, msg = self.camera_mgr.start_camera(cam_name)
                    if ok:
                        self._cameras_started.add(cam_name)
                        logger.info(f"Recording auto-started camera: {cam_name}")
                    else:
                        cam_start_errors.append(f"{cam_name}: {msg}")
                        logger.warning(f"Recording failed to start camera {cam_name}: {msg}")

            # ── Camera health check: ZED must be producing frames ──
            zed_ok = 'zed' in self._cameras_started
            if zed_ok:
                zed_ok = self.camera_mgr._check_zed_healthy(timeout=3.0)

            # Recovery loop: retry ZED up to 3 times with full destroy+recreate
            MAX_ZED_RETRIES = 3
            retry_count = 0
            while not zed_ok and 'zed' in CAMERA_CONFIG:
                retry_count += 1
                if retry_count > MAX_ZED_RETRIES:
                    logger.error(f"ZED camera failed after {MAX_ZED_RETRIES} recovery attempts")
                    # ── Ultimate fallback: restart this service ──
                    logger.error("Initiating service restart as last resort...")
                    # Clean up what we started
                    self.stop_event.set()
                    if self.bag_process and self.bag_process.poll() is None:
                        self.bag_process.terminate()
                        self.bag_process.wait(timeout=5)
                    for cn in list(self._cameras_started):
                        try:
                            self.camera_mgr.stop_camera(cn)
                        except Exception:
                            pass
                    self._cameras_started.clear()
                    # Remove the empty session directory
                    try:
                        import shutil
                        if self.output_dir and self.output_dir.exists():
                            shutil.rmtree(self.output_dir, ignore_errors=True)
                    except Exception:
                        pass

                    # Fork a child process to restart svtrobo-web
                    import os as _os
                    try:
                        pid = _os.fork()
                        if pid == 0:
                            _os.setsid()
                            import subprocess as _sp
                            _sp.Popen(
                                ['sudo', 'systemctl', 'restart', 'svtrobo-web'],
                                stdout=_sp.DEVNULL, stderr=_sp.DEVNULL,
                                preexec_fn=_os.setpgrp
                            )
                            _os._exit(0)
                    except Exception as fork_err:
                        logger.error(f"Fork restart failed: {fork_err}")
                        try:
                            subprocess.Popen(
                                ['sudo', 'systemctl', 'restart', 'svtrobo-web'],
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                preexec_fn=os.setpgrp
                            )
                        except Exception:
                            pass

                    return False, 'ZED camera unrecoverable, restarting svtrobo-web service...', None

                logger.warning(f"ZED not healthy, recovery attempt {retry_count}/{MAX_ZED_RETRIES}...")
                time.sleep(1)

                # Full force recover: destroy + recreate
                recovered = self.camera_mgr._force_recover_zed()
                if recovered:
                    self._cameras_started.add('zed')
                    zed_ok = self.camera_mgr._check_zed_healthy(timeout=3.0)
                    if zed_ok:
                        logger.info(f"ZED recovered on attempt {retry_count}")
                else:
                    logger.warning(f"ZED force recover failed on attempt {retry_count}")

            if not zed_ok and 'zed' in CAMERA_CONFIG:
                logger.error("ZED camera not available - recording WITHOUT camera is invalid")
                # Clean up
                self.stop_event.set()
                if self.bag_process and self.bag_process.poll() is None:
                    self.bag_process.terminate()
                    self.bag_process.wait(timeout=5)
                for cn in list(self._cameras_started):
                    try:
                        self.camera_mgr.stop_camera(cn)
                    except Exception:
                        pass
                self._cameras_started.clear()
                try:
                    import shutil
                    if self.output_dir and self.output_dir.exists():
                        shutil.rmtree(self.output_dir, ignore_errors=True)
                except Exception:
                    pass
                return False, 'ZED camera not available, recording aborted', None

            # Start camera frame saver
            self.stop_event.clear()
            self.save_thread = threading.Thread(
                target=self._save_camera_frames_loop,
                daemon=True,
            )
            self.save_thread.start()

            self.imu_save_thread = threading.Thread(
                target=self._save_imu_loop, daemon=True)
            self.imu_save_thread.start()

            self.distance_save_thread = threading.Thread(
                target=self._save_distance_loop, daemon=True)
            self.distance_save_thread.start()

            # Start recording watchdog (monitors ZED + rosbag + disk)
            self._watchdog_abort = False
            self.watchdog_thread = threading.Thread(
                target=self._recording_watchdog, daemon=True)
            self.watchdog_thread.start()

            self.running = True
            self.start_time = time.time()
            self.start_time_ns = time.time_ns()
            return True, 'Recording started', str(self.output_dir)

    def stop(self):
        """Stop recording. Returns (ok, message, info_dict)."""
        with self._lock:
            if not self.running:
                return False, 'Not recording', {}

            # Stop ros2 bag
            if self.bag_process and self.bag_process.poll() is None:
                self.bag_process.terminate()
                try:
                    self.bag_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.bag_process.kill()
                logger.info("ros2 bag record stopped")

            # Stop camera frame saver
            self.stop_event.set()
            if self.save_thread:
                self.save_thread.join(timeout=5)
            if self.imu_save_thread:
                self.imu_save_thread.join(timeout=3)
            if self.distance_save_thread:
                self.distance_save_thread.join(timeout=3)

            # Join watchdog thread
            if hasattr(self, 'watchdog_thread') and self.watchdog_thread:
                self.watchdog_thread.join(timeout=3)

            # Stop cameras that were auto-started by recording
            for cam_name in list(self._cameras_started):
                try:
                    self.camera_mgr.stop_camera(cam_name)
                    logger.info(f"Recording auto-stopped camera: {cam_name}")
                except Exception as e:
                    logger.warning(f"Failed to stop camera {cam_name}: {e}")
            self._cameras_started.clear()

            # Restart IMU reader now that ZED camera is released
            if 'zed' in CAMERA_CONFIG:
                start_imu_reader()

            stop_time_ns = time.time_ns()
            duration = time.time() - self.start_time if self.start_time else 0
            info = {
                'path': str(self.output_dir),
                'duration': round(duration, 1),
            }

            if self.output_dir and self.start_time_ns:
                info['teleop_actions'] = self._copy_teleop_actions(
                    self.output_dir,
                    self.start_time_ns,
                    stop_time_ns,
                )

            # Write summary.json
            self._write_summary(self.output_dir, duration)

            self.running = False
            self.start_time = None
            self.start_time_ns = None

            # Convert .db3 to JSONL in background
            if self.output_dir:
                bag_dir = self.output_dir / 'rosbag'
                if bag_dir.exists():
                    self._convert_bag(bag_dir, self.output_dir)

            abort_reason = ''
            if hasattr(self, '_watchdog_abort') and self._watchdog_abort:
                abort_reason = ' [ABORTED by watchdog]'
                logger.warning(f"Recording was aborted by watchdog{abort_reason}")
            return True, f'Recording stopped{abort_reason}', info

    def _copy_teleop_actions(self, output_dir, start_ns, end_ns):
        """Copy manually started teleop action records into this recording session."""
        try:
            source_paths = sorted(Path('/').glob(TELEOP_ACTION_LOG_GLOB.lstrip('/')))
            output_path = Path(output_dir) / TELEOP_ACTION_SESSION_FILE
            stats = copy_teleop_window(source_paths, output_path, start_ns, end_ns)
            if stats['samples']:
                logger.info(
                    "Copied %s teleop action samples to %s",
                    stats['samples'],
                    output_path,
                )
            else:
                logger.info("No teleop action samples found for this recording window")
            return stats
        except Exception as e:
            logger.warning(f"Failed to copy teleop action logs: {e}")
            return {'samples': 0, 'sources': [], 'output': None, 'error': str(e)}

    def _write_summary(self, output_dir, duration):
        """Write summary.json to the recording output directory."""
        try:
            from datetime import timezone

            output_path = Path(output_dir)
            # Extract timestamp from directory name (e.g. 20250101_120000)
            timestamp = output_path.name
            full_timestamp = output_path.parent.name.replace('-', '') + '_' + timestamp

            cameras = {}
            # Stat cameras: images (jpg) and depth (npy)
            for cam_dir in sorted((output_path / 'images').glob('*')):
                if cam_dir.is_dir():
                    cam_name = cam_dir.name
                    img_count = sum(1 for f in cam_dir.iterdir() if f.suffix == '.jpg')
                    depth_count = 0
                    depth_dir = output_path / 'depth' / cam_name
                    if depth_dir.is_dir():
                        depth_count = sum(1 for f in depth_dir.iterdir() if f.suffix == '.jpg')
                    cameras[cam_name] = {"images": img_count, "depth": depth_count}

            # Pointcloud
            pointcloud = {}
            pc_dir = output_path / 'pointcloud' / 'zed'
            if pc_dir.is_dir():
                pointcloud["zed"] = sum(1 for f in pc_dir.iterdir() if f.suffix == '.npz')

            # Rosbag size
            rosbag_size_mb = 0.0
            bag_dir = output_path / 'rosbag'
            if bag_dir.is_dir():
                for f in bag_dir.rglob('*'):
                    if f.is_file():
                        rosbag_size_mb += f.stat().st_size
                rosbag_size_mb = round(rosbag_size_mb / (1024 * 1024), 2)

            # Total directory size
            total_bytes = 0
            for f in output_path.rglob('*'):
                if f.is_file():
                    total_bytes += f.stat().st_size
            total_size_mb = round(total_bytes / (1024 * 1024), 2)

            # Distance sensor count
            distance_count = 0
            dist_file = output_path / 'distance.jsonl'
            if dist_file.is_file():
                with open(dist_file) as df:
                    distance_count = sum(1 for _ in df)

            summary = {
                "timestamp": full_timestamp,
                "duration_seconds": round(duration, 2),
                "cameras": cameras,
                "pointcloud": pointcloud,
                "distance_sensor": {"samples": distance_count},
                "rosbag_size_mb": rosbag_size_mb,
                "total_size_mb": total_size_mb,
                "created_at": datetime.now(timezone.utc).isoformat(),
            }

            summary_path = output_path / 'summary.json'
            with open(summary_path, 'w') as sf:
                json.dump(summary, sf, indent=2, ensure_ascii=False)
            logger.info(f"summary.json written to {summary_path}")
        except Exception as e:
            logger.warning(f"Failed to write summary.json: {e}")

    def _convert_bag(self, bag_dir, output_dir):
        """Run bag-to-JSONL conversion in a background thread."""
        def _do():
            try:
                script = Path(__file__).parent / 'bag_converter.py'
                cmd = [sys.executable, str(script), str(bag_dir), str(output_dir)]
                if DELETE_DB_AFTER_CONVERT:
                    cmd.append('--delete-db')
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
                if result.returncode == 0:
                    logger.info(f"Bag→JSONL: {result.stdout.strip()}")
                else:
                    logger.warning(f"Bag→JSONL failed: {result.stderr.strip()}")
            except Exception as e:
                logger.warning(f"Bag→JSONL error: {e}")

        threading.Thread(target=_do, daemon=True, name='bag-converter').start()

    def _count_files(self, directory, ext='*.jpg'):
        """Count files matching pattern in directory. Returns 0 if not exists."""
        if not directory or not directory.exists():
            return 0
        return sum(1 for f in directory.glob(ext) if f.is_file())

    def get_status(self):
        """Return current recording status with per-source health info."""
        elapsed = 0
        if self.running and self.start_time:
            elapsed = time.time() - self.start_time

        result = {
            'running': self.running,
            'path': str(self.output_dir) if self.output_dir else None,
            'elapsed': round(elapsed, 1),
        }
        # Add abort message for frontend detection
        if hasattr(self, '_watchdog_abort') and self._watchdog_abort:
            result['message'] = 'Recording ABORTED by watchdog'

        if self.running and self.output_dir:
            d = self.output_dir

            # ── Camera frame counts ──
            zed_left = self._count_files(d / 'images' / 'zed')
            zed_right = self._count_files(d / 'images' / 'zed_right')
            zed_depth = self._count_files(d / 'depth' / 'zed')
            zed_pc = self._count_files(d / 'pointcloud' / 'zed', '*.npz')
            d405_1 = self._count_files(d / 'images' / 'd405_1')
            d405_2 = self._count_files(d / 'images' / 'd405_2')

            # ── IMU samples ──
            imu_count = 0
            imu_path = d / 'imu.jsonl'
            if imu_path.exists():
                try:
                    imu_count = sum(1 for _ in open(imu_path))
                except Exception:
                    pass

            # ── Rosbag status ──
            rosbag_alive = False
            if self.bag_process and self.bag_process.poll() is None:
                rosbag_alive = True

            # ── ZED frame freshness (from watchdog) ──
            zed_fresh = True
            if hasattr(self, '_watchdog_zed_last_frame'):
                zed_fresh = (time.monotonic() - self._watchdog_zed_last_frame) < 10

            # ── Determine overall status for each source ──
            # ok = producing data, error = not producing, idle = not applicable
            cameras_running = self._cameras_started

            sources = {}

            # ZED
            if 'zed' in cameras_running:
                sources['zed_left'] = {'status': 'ok' if zed_left > 0 else ('error' if not zed_fresh else 'idle'), 'count': zed_left}
                sources['zed_right'] = {'status': 'ok' if zed_right > 0 else ('error' if not zed_fresh else 'idle'), 'count': zed_right}
                sources['zed_depth'] = {'status': 'ok' if zed_depth > 0 else ('error' if not zed_fresh else 'idle'), 'count': zed_depth}
                sources['pointcloud'] = {'status': 'ok' if zed_pc > 0 else ('error' if not zed_fresh else 'idle'), 'count': zed_pc}
            elif 'zed' not in CAMERA_CONFIG:
                pass  # ZED not configured

            # D405
            for cam, cnt in [('d405_1', d405_1), ('d405_2', d405_2)]:
                if cam in cameras_running:
                    sources[cam] = {'status': 'ok' if cnt > 0 else 'idle', 'count': cnt}
                elif cam in CAMERA_CONFIG and cam not in cameras_running:
                    sources[cam] = {'status': 'idle', 'count': 0, 'note': '未连接'}

            # IMU
            if 'zed' in cameras_running:
                sources['imu'] = {'status': 'ok' if imu_count > 0 else ('error' if not zed_fresh else 'idle'), 'count': imu_count}

            # Rosbag: per-topic message counts from sqlite3 (merge all rosbag*/ dirs)
            import glob as _glob, sqlite3 as _sqlite3
            ros_topic_map = ROS_TOPIC_STATUS_MAP
            if rosbag_alive or (d / 'rosbag').exists():
                try:
                    # Find all rosbag*/ subdirectories and collect all db3 files
                    all_db_files = []
                    for bag_sub in sorted(d.iterdir()):
                        if bag_sub.is_dir() and bag_sub.name.startswith('rosbag'):
                            all_db_files.extend(sorted(_glob.glob(str(bag_sub / '*.db3'))))
                    # Aggregate message counts across all db3 files
                    total_msg_counts = {}  # topic_name -> count
                    for db_file in all_db_files:
                        db = _sqlite3.connect(f'file:{db_file}?mode=ro', uri=True)
                        topic_ids = {}
                        for row in db.execute('SELECT id, name FROM topics'):
                            topic_ids[row[0]] = row[1]
                        for row in db.execute('SELECT topic_id, COUNT(*) FROM messages GROUP BY topic_id'):
                            tname = topic_ids.get(row[0], '')
                            total_msg_counts[tname] = total_msg_counts.get(tname, 0) + row[1]
                        db.close()
                    for fkey, topic_name in ros_topic_map.items():
                        cnt = total_msg_counts.get(topic_name, 0)
                        sources[fkey] = {'status': 'ok' if cnt > 0 else 'idle', 'count': cnt}
                except Exception as e:
                    logger.debug(f'Rosbag topic count error: {e}')
                    for fkey in ros_topic_map:
                        sources[fkey] = {'status': 'idle', 'count': 0}
            else:
                for fkey in ros_topic_map:
                    sources[fkey] = {'status': 'error', 'count': 0}

            # Distance sensor
            dist_file = d / 'distance.jsonl'
            if dist_file.exists():
                try:
                    dist_count = sum(1 for _ in open(dist_file))
                except Exception:
                    dist_count = 0
                sources['distance'] = {'status': 'ok' if dist_count > 0 else 'idle', 'count': dist_count}

            result['sources'] = sources

        return result


    def _recording_watchdog(self):
        """Background thread: monitor recording health during active recording.

        Checks every 2 seconds:
        1. ZED camera is still producing frames (no mid-recording drop)
        2. ros2 bag subprocess is still alive
        3. Disk has enough free space
        If any check fails, attempts recovery or aborts recording.
        """
        ZED_TIMEOUT = 6.0       # No frames for 6s = ZED dropped
        DISK_MIN_MB = 500       # Minimum 500MB free to continue
        CHECK_INTERVAL = 2.0

        zed_last_frame_time = time.monotonic()  # Updated by save loop

        # We need a shared ref for the save loop to update
        # Store on self so the save loop can poke it
        self._watchdog_zed_last_frame = zed_last_frame_time
        self._watchdog_abort = False  # Set by watchdog to signal abort

        while not self.stop_event.is_set():
            self.stop_event.wait(CHECK_INTERVAL)
            if self.stop_event.is_set():
                break
            if not self.running:
                break

            # ── Check 1: ZED frame flow ──
            elapsed_since_frame = time.monotonic() - self._watchdog_zed_last_frame
            if elapsed_since_frame > ZED_TIMEOUT and 'zed' in self._cameras_started:
                logger.error(f"Watchdog: ZED no frames for {elapsed_since_frame:.1f}s, attempting recovery...")
                # Try force recover
                recovered = self.camera_mgr._force_recover_zed()
                if recovered:
                    self._cameras_started.add('zed')
                    # Verify it actually produces frames
                    if self.camera_mgr._check_zed_healthy(timeout=3.0):
                        logger.info("Watchdog: ZED recovered during recording")
                        self._watchdog_zed_last_frame = time.monotonic()
                        continue

                logger.error("Watchdog: ZED recovery failed during recording, aborting...")
                self._watchdog_abort = True
                # Trigger async stop from main thread via the save thread's event
                self.stop_event.set()
                break

            # ── Check 2: ros2 bag subprocess alive ──
            if self.bag_process and self.bag_process.poll() is not None:
                logger.error(f"Watchdog: ros2 bag died (exit code {self.bag_process.returncode}), restarting...")
                # Try to restart ros2 bag (numbered subfolder)
                if not hasattr(self, '_bag_restart_idx'):
                    self._bag_restart_idx = 1
                else:
                    self._bag_restart_idx += 1
                bag_dir = self.output_dir / f'rosbag_{self._bag_restart_idx}'
                try:
                    cmd = (
                        'source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=56 && export ROS_LOCALHOST_ONLY=1 && '
                        'source /home/svt/svtrobo_ws/install/setup.bash && '
                        'exec ros2 bag record ' + ' '.join(RECORD_TOPICS) + f' -o {bag_dir}'
                    )
                    self.bag_process = subprocess.Popen(
                        ['bash', '-c', cmd],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                    )
                    logger.info(f"Watchdog: ros2 bag restarted as {bag_dir.name}")
                except Exception as e:
                    logger.error(f"Watchdog: ros2 bag restart failed: {e}, aborting recording")
                    self._watchdog_abort = True
                    self.stop_event.set()
                    break

            # ── Check 3: Disk space ──
            try:
                stat = os.statvfs(str(RECORDING_DIR))
                free_mb = (stat.f_bavail * stat.f_frsize) / (1024 * 1024)
                if free_mb < DISK_MIN_MB:
                    logger.error(f"Watchdog: disk space low ({free_mb:.0f}MB free), aborting recording")
                    self._watchdog_abort = True
                    self.stop_event.set()
                    break
            except Exception:
                pass

        logger.info("Recording watchdog exiting")

    def _save_camera_frames_loop(self):
        """Continuously save camera frames (color + depth + pointcloud + right eye) + IMU from queues.
        
        Uses deadline-based scheduling for precise 2Hz frame saving:
        - Maintains per-camera next_deadline timestamps
        - Only saves when current time >= next_deadline
        - Deadlines never drift — each is offset from the recording start time
        """
        import numpy as _np
        cam_names = ['d405_1', 'd405_2', 'zed']

        # Deadline-based 2Hz scheduling: one deadline per camera
        _start_time = time.monotonic()
        _next_deadline = {name: _start_time + RECORD_INTERVAL for name in cam_names}

        # IMU saving is handled by a separate _save_imu_loop thread

        try:
            while not self.stop_event.is_set():
                any_saved = False
                _now = time.monotonic()

                for name in cam_names:
                    # Deadline-based throttling: only save if deadline reached
                    if _now < _next_deadline[name]:
                        continue

                    camera_saved = False
                    frame_timestamp_us = None

                    # Drain all queued color frames, save latest only
                    latest_color = None
                    while True:
                        result = self.camera_mgr.get_frame(name)
                        if result is None:
                            break
                        latest_color = result
                    if latest_color:
                        frame, frame_timestamp_us = latest_color
                        img_dir = self.output_dir / 'images' / name
                        img_dir.mkdir(parents=True, exist_ok=True)
                        path = img_dir / f'{frame_timestamp_us}.jpg'
                        try:
                            # JPEG encode at save time (2Hz), not in capture loop (15Hz)
                            if isinstance(frame, bytes):
                                path.write_bytes(frame)
                            else:
                                _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                                path.write_bytes(jpeg.tobytes())
                            any_saved = True
                            camera_saved = True
                        except Exception as e:
                            logger.warning(f"Failed to save {name} frame: {e}")

                    # Drain all queued depth frames, save latest only (if deadline met)
                    latest_depth = None
                    while True:
                        depth_result = self.camera_mgr.get_depth_frame(name)
                        if depth_result is None:
                            break
                        latest_depth = depth_result
                    if latest_depth:
                        depth_raw, d_timestamp_us = latest_depth
                        if frame_timestamp_us is None:
                            frame_timestamp_us = d_timestamp_us
                        try:
                            depth_max = 20000.0 if name == 'zed' else 1000.0
                            if depth_raw.dtype == _np.float32:
                                depth_vis = _np.clip(depth_raw / depth_max, 0, 1)
                            else:
                                depth_vis = _np.clip(depth_raw.astype(_np.float32) / depth_max, 0, 1)
                            depth_u8 = (depth_vis * 255).astype(_np.uint8)
                            depth_colored = cv2.applyColorMap(depth_u8, cv2.COLORMAP_JET)
                            _, depth_jpeg = cv2.imencode('.jpg', depth_colored, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                            depth_dir = self.output_dir / 'depth' / name
                            depth_dir.mkdir(parents=True, exist_ok=True)
                            depth_path = depth_dir / f'{d_timestamp_us}.jpg'
                            depth_path.write_bytes(depth_jpeg.tobytes())
                            any_saved = True
                            camera_saved = True
                        except Exception as e:
                            logger.warning(f"Failed to save {name} depth: {e}")

                    # Save ZED right eye + point cloud if available
                    if name == 'zed':
                        # Drain right eye frames
                        latest_right = None
                        while True:
                            r_result = self.camera_mgr.get_right_frame(name)
                            if r_result is None:
                                break
                            latest_right = r_result
                        if latest_right:
                            right_frame, r_ts = latest_right
                            if frame_timestamp_us is None:
                                frame_timestamp_us = r_ts
                            right_dir = self.output_dir / 'images' / 'zed_right'
                            right_dir.mkdir(parents=True, exist_ok=True)
                            right_path = right_dir / f'{r_ts}.jpg'
                            try:
                                if isinstance(right_frame, bytes):
                                    right_path.write_bytes(right_frame)
                                else:
                                    _, r_jpeg = cv2.imencode('.jpg', right_frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                                    right_path.write_bytes(r_jpeg.tobytes())
                                any_saved = True
                                camera_saved = True
                            except Exception as e:
                                logger.warning(f"Failed to save zed right frame: {e}")

                        # Save point cloud from cam._latest_pc (retrieved in grab loop)
                        cam_inst = self.camera_mgr.cameras[name]['instance'] if name in self.camera_mgr.cameras else None
                        if (
                            cam_inst
                            and frame_timestamp_us is not None
                            and hasattr(cam_inst, '_latest_pc')
                            and cam_inst._latest_pc is not None
                        ):
                            pc_data = cam_inst._latest_pc
                            pc_ts = frame_timestamp_us
                            try:
                                pc_dir = self.output_dir / 'pointcloud' / name
                                pc_dir.mkdir(parents=True, exist_ok=True)
                                pc_path = pc_dir / f'{pc_ts}.npz'
                                # Apply downsample and dtype conversion
                                pc_arr = pc_data
                                if PC_DOWNSAMPLE > 1:
                                    pc_arr = pc_arr[::PC_DOWNSAMPLE, ::PC_DOWNSAMPLE, :]
                                if PC_DTYPE != pc_arr.dtype:
                                    pc_arr = pc_arr.astype(PC_DTYPE)
                                _np.savez(pc_path, xyzrgba=pc_arr)
                                any_saved = True
                                camera_saved = True
                            except Exception as e:
                                logger.warning(f"Failed to save {name} pointcloud: {e}")

                    if camera_saved:
                        # Advance deadline for this camera
                        _next_deadline[name] += RECORD_INTERVAL
                        # If we fell behind multiple deadlines, skip to next valid one
                        if _next_deadline[name] < _now:
                            _next_deadline[name] = _now + RECORD_INTERVAL

                        # Poke watchdog when any ZED modality is saved
                        if name == 'zed' and hasattr(self, '_watchdog_zed_last_frame'):
                            self._watchdog_zed_last_frame = time.monotonic()
                # Sleep until next deadline or 10ms idle
                if not any_saved:
                    nearest = min(_next_deadline.values())
                    wait_time = min(0.01, max(0.001, nearest - time.monotonic()))
                    if self.stop_event.wait(wait_time):
                        break
        finally:
            logger.info("Camera frame saver exiting")


    def _save_imu_loop(self):
        """Dedicated thread: write IMU samples from queue to imu.jsonl.

        The _capture_loop pushes every unique IMU sample (after each ZED grab)
        into imu_rec_queue. This thread simply drains the queue and writes to disk.
        Since IMU data refreshes only on grab(), recording at grab rate (~15Hz)
        gives accurate, drift-free timestamps.
        """
        imu_path = self.output_dir / 'imu.jsonl'
        count = 0

        with open(imu_path, 'w') as f:
            while not self.stop_event.is_set():
                try:
                    sample = imu_rec_queue.get(timeout=0.05)
                    f.write(json.dumps(sample) + chr(10))
                    count += 1
                except Exception:
                    pass

        # Drain remaining
        while not imu_rec_queue.empty():
            try:
                sample = imu_rec_queue.get_nowait()
                f.write(json.dumps(sample) + chr(10))
                count += 1
            except Exception:
                break

        logger.info(f"IMU saver exiting, saved {count} samples")

    def _save_distance_loop(self):
        """Dedicated thread: poll distance_cache and write to distance.jsonl at ~5Hz."""
        dist_path = self.output_dir / 'distance.jsonl'
        count = 0

        with open(dist_path, 'w') as f:
            while not self.stop_event.is_set():
                with distance_cache['lock']:
                    data = dict(distance_cache['data'])
                    ok = distance_cache['ok']
                    ts = distance_cache['timestamp']
                if ok and any(v is not None for v in data.values()):
                    record = {
                        'timestamp': round(ts, 6),
                        'front': data.get('front'),
                        'right': data.get('right'),
                        'rear': data.get('rear'),
                        'left': data.get('left'),
                    }
                    f.write(json.dumps(record) + chr(10))
                    f.flush()
                    count += 1
                self.stop_event.wait(0.2)  # 5Hz

        logger.info(f"Distance saver exiting, saved {count} samples")


class F710Manager:
    """Manages the f710_teleop node lifecycle."""

    def __init__(self):
        self.process = None
        self._lock = threading.Lock()

    def start(self):
        """Start the f710_teleop node. Returns (ok, message)."""
        with self._lock:
            if self.process and self.process.poll() is None:
                return False, 'F710 node already running'

            try:
                cmd = (
                    'source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=56 && export ROS_LOCALHOST_ONLY=1 && '
                    'source /home/svt/svtrobo_ws/install/setup.bash && '
                    'exec ros2 launch f710_teleop f710_teleop.launch.py'
                )
                self.process = subprocess.Popen(
                    ['bash', '-c', cmd],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    preexec_fn=os.setpgrp,
                )
                logger.info(f"F710 node started (pid={self.process.pid})")
                return True, 'F710 node started'
            except FileNotFoundError:
                return False, 'ros2 command not found. Is ROS2 sourced?'
            except Exception as e:
                return False, str(e)

    def stop(self):
        """Stop the f710_teleop node. Returns (ok, message)."""
        with self._lock:
            if not self.process or self.process.poll() is not None:
                return False, 'F710 node not running'

            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
            except Exception:
                pass
            self.process = None
            logger.info("F710 node stopped")
            return True, 'F710 node stopped'

    def get_status(self):
        """Return whether the f710 node process is alive."""
        alive = self.process is not None and self.process.poll() is None
        return {'running': alive}


class VisualRelocalizationManager:
    """Runs the visual relocalization ROS2 commands from fixed web actions."""

    def __init__(self):
        self.process = None
        self._lock = threading.Lock()
        self._logs = []
        self._last_command = None
        self._last_returncode = None
        self._last_stdout = ''
        self._last_stderr = ''
        self._target_name = None
        self._started_at = None

    @staticmethod
    def _ros_prefix():
        return (
            'source /opt/ros/humble/setup.bash && '
            'export ROS_DOMAIN_ID=56 && export ROS_LOCALHOST_ONLY=1 && '
            'source /home/svt/svtrobo_ws/install/setup.bash'
        )

    @staticmethod
    def _validate_target_name(target_name):
        target_name = str(target_name or '').strip()
        if not VISUAL_RELOCALIZATION_TARGET_RE.fullmatch(target_name):
            raise ValueError('target_name must match ^[A-Za-z0-9_-]{1,64}$')
        return target_name

    def _append_log(self, line):
        text = str(line).strip()
        if not text:
            return
        with self._lock:
            self._logs.append(text)
            if len(self._logs) > VISUAL_RELOCALIZATION_LOG_LIMIT:
                self._logs = self._logs[-VISUAL_RELOCALIZATION_LOG_LIMIT:]

    def _refresh_locked(self):
        if self.process and self.process.poll() is not None:
            self._last_returncode = self.process.returncode
            self.process = None
            self._started_at = None

    def _build_record_command(self, target_name):
        return (
            f'{self._ros_prefix()} && '
            'exec ros2 run visual_relocalization record_target '
            f'--ros-args -p target_name:={target_name}'
        )

    def _build_start_command(self, target_name):
        return (
            f'{self._ros_prefix()} && '
            'exec ros2 run visual_relocalization visual_servo '
            f'--ros-args -p target_name:={target_name}'
        )

    def _popen_kwargs(self):
        kwargs = {}
        if hasattr(os, 'setpgrp'):
            kwargs['preexec_fn'] = os.setpgrp
        return kwargs

    def record_target(self, target_name):
        try:
            target_name = self._validate_target_name(target_name)
        except ValueError as exc:
            return False, str(exc), self.get_status()

        with self._lock:
            self._refresh_locked()
            if self.process is not None:
                return False, 'visual_servo is running; stop it before recording a target', self._status_locked()
            cmd = self._build_record_command(target_name)
            self._target_name = target_name
            self._last_command = cmd
            self._last_returncode = None
            self._last_stdout = ''
            self._last_stderr = ''
            self._logs.append(f'$ {cmd}')

        try:
            completed = subprocess.run(
                ['bash', '-c', cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                encoding='utf-8',
                errors='replace',
                timeout=VISUAL_RELOCALIZATION_RECORD_TIMEOUT_SEC,
            )
            stdout = completed.stdout or ''
            stderr = completed.stderr or ''
            with self._lock:
                self._last_returncode = completed.returncode
                self._last_stdout = stdout[-4000:]
                self._last_stderr = stderr[-4000:]
                for line in (stdout + stderr).splitlines()[-VISUAL_RELOCALIZATION_LOG_LIMIT:]:
                    if line.strip():
                        self._logs.append(line.strip())
                if len(self._logs) > VISUAL_RELOCALIZATION_LOG_LIMIT:
                    self._logs = self._logs[-VISUAL_RELOCALIZATION_LOG_LIMIT:]
                status = self._status_locked()
            ok = completed.returncode == 0
            message = 'target recorded' if ok else f'record_target exited with code {completed.returncode}'
            return ok, message, status
        except subprocess.TimeoutExpired as exc:
            stdout = (exc.stdout or '') if isinstance(exc.stdout, str) else ''
            stderr = (exc.stderr or '') if isinstance(exc.stderr, str) else ''
            with self._lock:
                self._last_returncode = None
                self._last_stdout = stdout[-4000:]
                self._last_stderr = stderr[-4000:]
                self._logs.append('record_target timed out')
                status = self._status_locked()
            return False, 'record_target timed out', status
        except FileNotFoundError:
            return False, 'bash or ros2 command not found. Is ROS2 sourced?', self.get_status()
        except Exception as exc:
            return False, str(exc), self.get_status()

    def start(self, target_name):
        try:
            target_name = self._validate_target_name(target_name)
        except ValueError as exc:
            return False, str(exc), self.get_status()

        with self._lock:
            self._refresh_locked()
            if self.process is not None:
                return False, 'visual_servo already running', self._status_locked()
            cmd = self._build_start_command(target_name)
            self._target_name = target_name
            self._last_command = cmd
            self._last_returncode = None
            self._last_stdout = ''
            self._last_stderr = ''
            self._started_at = datetime.now().isoformat(timespec='seconds')
            self._logs.append(f'$ {cmd}')

        try:
            process = subprocess.Popen(
                ['bash', '-c', cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding='utf-8',
                errors='replace',
                bufsize=1,
                **self._popen_kwargs(),
            )
            with self._lock:
                self.process = process
                status = self._status_locked()
            threading.Thread(
                target=self._read_process_output,
                args=(process,),
                daemon=True,
                name='visual-relocalization-log',
            ).start()
            logger.info(f"visual_servo started (pid={process.pid}, target={target_name})")
            return True, 'visual_servo started', status
        except FileNotFoundError:
            return False, 'bash or ros2 command not found. Is ROS2 sourced?', self.get_status()
        except Exception as exc:
            return False, str(exc), self.get_status()

    def _read_process_output(self, process):
        try:
            if process.stdout:
                for line in process.stdout:
                    self._append_log(line)
            process.wait()
        finally:
            with self._lock:
                if self.process is process:
                    self._last_returncode = process.returncode
                    self.process = None
                    self._started_at = None
            self._append_log(f'visual_servo exited with code {process.returncode}')

    def stop(self):
        with self._lock:
            self._refresh_locked()
            process = self.process
            if process is None:
                return False, 'visual_servo not running', self._status_locked()

        try:
            if hasattr(os, 'killpg') and hasattr(os, 'getpgid'):
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            else:
                process.terminate()
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            if hasattr(os, 'killpg') and hasattr(os, 'getpgid'):
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            else:
                process.kill()
            process.wait(timeout=5)
        except Exception as exc:
            logger.warning(f"visual_servo stop failed: {exc}")

        with self._lock:
            if self.process is process:
                self._last_returncode = process.returncode
                self.process = None
                self._started_at = None
            self._logs.append('visual_servo stop requested')
            return True, 'visual_servo stopped', self._status_locked()

    def _status_locked(self):
        running = self.process is not None and self.process.poll() is None
        return {
            'running': running,
            'pid': self.process.pid if running else None,
            'target_name': self._target_name,
            'started_at': self._started_at,
            'last_command': self._last_command,
            'last_returncode': self._last_returncode,
            'last_stdout': self._last_stdout,
            'last_stderr': self._last_stderr,
            'logs': list(self._logs[-VISUAL_RELOCALIZATION_LOG_LIMIT:]),
        }

    def get_status(self):
        with self._lock:
            self._refresh_locked()
            return self._status_locked()


# --- HTTP Handlers ---

camera_mgr = CameraManager()
recording_mgr = RecordingManager(camera_mgr)
f710_mgr = F710Manager()
visual_reloc_mgr = VisualRelocalizationManager()
imu_cache = {'data': None, 'lock': threading.Lock()}
imu_rec_queue = queue.Queue(maxsize=100)
_imu_thread = None
_imu_stop = threading.Event()


# --- Distance Sensor (SEN0492 Laser Range Finder via RS485/Modbus RTU) ---

distance_cache = {
    'data': {'front': None, 'right': None, 'rear': None, 'left': None},
    'timestamp': 0,
    'lock': threading.Lock(),
    'ok': False,
}

DISTANCE_PORT = '/dev/distance_sensor'
DISTANCE_BAUD = 115200
DISTANCE_SENSORS = {
    'front': 0x51,
    'right': 0x52,
    'rear': 0x53,
    'left': 0x54,
}
DISTANCE_READ_INTERVAL = 0.2  # 5 Hz polling


def _crc16(buf):
    """Modbus CRC16."""
    crc = 0xFFFF
    for b in buf:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def _read_distance_sensor(ser, addr):
    """Read distance (mm) from a single SEN0492 sensor via Modbus RTU."""
    cmd = bytes([addr, 0x03, 0x00, 0x34, 0x00, 0x01])
    c = _crc16(cmd)
    cmd += bytes([c & 0xFF, (c >> 8) & 0xFF])
    try:
        ser.reset_input_buffer()
        ser.write(cmd)
        time.sleep(0.08)
        data = ser.read(32)
        if data and len(data) >= 7 and data[0] == addr and data[1] == 0x03:
            return data[3] * 256 + data[4]
    except Exception:
        pass
    return None


def _distance_reader_loop():
    """Background thread: continuously poll all 4 distance sensors at ~5Hz."""
    logger.info('Distance sensor reader thread starting')
    import serial
    ser = None
    max_retries = 30
    for attempt in range(1, max_retries + 1):
        try:
            ser = serial.Serial(
                port=DISTANCE_PORT,
                baudrate=DISTANCE_BAUD,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=0.15,
            )
            logger.info(f'Distance sensor: opened {DISTANCE_PORT} on attempt {attempt}')
            break
        except Exception as e:
            logger.warning(f'Distance sensor: attempt {attempt}/{max_retries} failed to open {DISTANCE_PORT}: {e}')
            if attempt >= max_retries:
                logger.error(f'Distance sensor: giving up after {max_retries} retries')
                return
            time.sleep(5)

    sensor_names = list(DISTANCE_SENSORS.keys())
    sensor_addrs = list(DISTANCE_SENSORS.values())
    fail_count = {n: 0 for n in sensor_names}

    while True:
        try:
            readings = {}
            any_ok = False
            for i, name in enumerate(sensor_names):
                val = _read_distance_sensor(ser, sensor_addrs[i])
                readings[name] = val
                if val is not None:
                    any_ok = True
                    fail_count[name] = 0
                else:
                    fail_count[name] += 1

            with distance_cache['lock']:
                distance_cache['data'] = readings
                distance_cache['timestamp'] = time.time()
                distance_cache['ok'] = any_ok

            time.sleep(DISTANCE_READ_INTERVAL)
        except Exception as e:
            logger.debug(f'Distance sensor read error: {e}')
            time.sleep(1.0)

    try:
        ser.close()
    except Exception:
        pass


# Start distance sensor reader thread at module load
_distance_thread = threading.Thread(target=_distance_reader_loop, daemon=True, name='distance-sensor')
_distance_thread.start()


async def index_handler(request):
    return web.FileResponse(STATIC_DIR / 'index.html')


async def camera_stream_handler(request):
    name = request.match_info['name']
    if name not in CAMERA_CONFIG:
        return web.Response(status=404, text="Unknown camera")

    status = camera_mgr.get_status()
    if not status.get(name, {}).get('running'):
        return web.Response(status=404, text="Camera not active. Start it first via POST /camera/start")

    response = web.StreamResponse()
    response.content_type = 'multipart/x-mixed-replace; boundary=frame'
    response.headers['Cache-Control'] = 'no-cache'
    await response.prepare(request)

    boundary = b'--frame\r\n'
    header = b'Content-Type: image/jpeg\r\n\r\n'

    try:
        while True:
            result = camera_mgr.peek_frame(name)
            if result is not None:
                frame, _ = result
                # JPEG encode for streaming (raw numpy from capture loop)
                if isinstance(frame, bytes):
                    jpeg_bytes = frame
                else:
                    _, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                    jpeg_bytes = jpeg.tobytes()
                msg = boundary + header + jpeg_bytes + b'\r\n'
                await response.write(msg)
            await asyncio.sleep(1.0 / STREAM_FPS)
    except (ConnectionResetError, ConnectionError):
        pass
    return response


async def camera_start_handler(request):
    if recording_mgr.running:
        return web.json_response({'ok': False, 'message': 'Cannot start camera during active recording'})
    data = await request.json()
    name = data.get('camera', '')
    ok, msg = camera_mgr.start_camera(name)
    return web.json_response({'ok': ok, 'message': msg})


async def camera_stop_handler(request):
    data = await request.json()
    name = data.get('camera', '')
    if recording_mgr.running:
        return web.json_response({'ok': False, 'message': 'Cannot stop camera during active recording'})
    ok, msg = camera_mgr.stop_camera(name)
    return web.json_response({'ok': ok, 'message': msg})


async def camera_status_handler(request):
    return web.json_response(camera_mgr.get_status())


async def recording_start_handler(request):
    try:
        body = await request.json() if request.content_type == 'application/json' else {}
    except Exception:
        body = {}
    test_mode = body.get('test', False)
    ok, msg, path = recording_mgr.start(test_mode=test_mode)
    return web.json_response({'ok': ok, 'message': msg, 'path': path})


async def recording_stop_handler(request):
    ok, msg, info = recording_mgr.stop()
    resp = {'ok': ok, 'message': msg}
    resp.update(info)
    return web.json_response(resp)


async def recording_status_handler(request):
    return web.json_response(recording_mgr.get_status())


async def f710_start_handler(request):
    ok, msg = f710_mgr.start()
    return web.json_response({'ok': ok, 'message': msg})


async def f710_stop_handler(request):
    ok, msg = f710_mgr.stop()
    return web.json_response({'ok': ok, 'message': msg})


async def f710_status_handler(request):
    return web.json_response(f710_mgr.get_status())


async def _visual_relocalization_request_body(request):
    try:
        if request.content_type == 'application/json':
            return await request.json()
    except Exception:
        pass
    return {}


async def visual_relocalization_status_handler(request):
    return web.json_response(visual_reloc_mgr.get_status())


async def visual_relocalization_record_handler(request):
    body = await _visual_relocalization_request_body(request)
    target_name = body.get('target_name', 'bench_station')
    ok, message, status = visual_reloc_mgr.record_target(target_name)
    return web.json_response({'ok': ok, 'message': message, 'status': status})


async def visual_relocalization_start_handler(request):
    body = await _visual_relocalization_request_body(request)
    target_name = body.get('target_name', 'bench_station')
    ok, message, status = visual_reloc_mgr.start(target_name)
    return web.json_response({'ok': ok, 'message': message, 'status': status})


async def visual_relocalization_stop_handler(request):
    ok, message, status = visual_reloc_mgr.stop()
    return web.json_response({'ok': ok, 'message': message, 'status': status})


async def exit_kiosk_handler(request):
    """退出 Firefox kiosk 全屏模式."""
    try:
        await asyncio.create_subprocess_exec(
            "killall", "firefox",
            stdout=asyncio.subprocess.DEVNULL,
            stderr=asyncio.subprocess.DEVNULL
        )
    except Exception as e:
        logger.warning(f"exit-kiosk failed: {e}")
    return web.json_response({"status": "ok"})

def _imu_reader_loop():
    """独立线程：以最小开销读取 ZED IMU（VGA+无深度，~20Hz）"""
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).parent.parent / 'camera_driver'))
    try:
        import pyzed.sl as sl
    except ImportError:
        logger.warning('ZED SDK not available, standalone IMU disabled')
        return
    zed = sl.Camera()
    params = sl.InitParameters()
    params.camera_resolution = sl.RESOLUTION.VGA
    params.camera_fps = 15
    params.depth_mode = sl.DEPTH_MODE.NONE
    params.sensors_required = True
    err = zed.open(params)
    if err != sl.ERROR_CODE.SUCCESS:
        logger.warning(f'ZED open for IMU-only failed: {err}')
        return
    logger.info('ZED IMU-only reader started')
    sensors = sl.SensorsData()
    runtime = sl.RuntimeParameters()
    while not _imu_stop.is_set():
        try:
            if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
                if zed.get_sensors_data(sensors, sl.TIME_REFERENCE.IMAGE) == sl.ERROR_CODE.SUCCESS:
                    imu_sensor = sensors.get_imu_data()
                    if imu_sensor.is_available:
                        import math
                        acc = imu_sensor.get_linear_acceleration()
                        gyro = imu_sensor.get_angular_velocity()
                        mag_data = sensors.get_magnetometer_data()
                        baro = sensors.get_barometer_data()
                        temp_data = sensors.get_temperature_data()
                        mag = mag_data.get_magnetic_field_calibrated() if mag_data.is_available else (0,0,0)
                        imu_temp_raw = temp_data.get(sl.SENSOR_LOCATION.IMU)
                        imu_temp = imu_temp_raw * 0.01 if imu_temp_raw and imu_temp_raw > 0 else 0.0
                        baro_temp_raw = temp_data.get(sl.SENSOR_LOCATION.BAROMETER)
                        env_temp = baro_temp_raw * 0.01 if baro_temp_raw and baro_temp_raw > 0 else 0.0
                        ts_ns = imu_sensor.timestamp.get_nanoseconds()
                        imu_data = {
                            'accel': [acc[0], acc[1], acc[2]],
                            'gyro_dps': [gyro[0], gyro[1], gyro[2]],
                            'gyro_rad': [math.radians(gyro[0]), math.radians(gyro[1]), math.radians(gyro[2])],
                            'mag': [mag[0], mag[1], mag[2]],
                            'mag_valid': 1 if mag_data.is_available else 0,
                            'imu_temp': imu_temp,
                            'pressure': baro.pressure if baro.is_available else 0.0,
                            'env_temp': env_temp,
                            'timestamp_ns': ts_ns,
                            'timestamp_s': ts_ns / 1e9,
                            'server_ts': time.time(),
                        }
                        with imu_cache['lock']:
                            imu_cache['data'] = imu_data
                        _atomic_write_json('/dev/shm/zed_imu.json', imu_data)
            _imu_stop.wait(0.05)
        except Exception as e:
            logger.debug(f'IMU reader error: {e}')
            _imu_stop.wait(0.5)
    zed.close()
    logger.info('ZED IMU-only reader stopped')


def start_imu_reader():
    """启动独立 IMU 读取线程（ZED 未被相机管理器启动时调用）"""
    global _imu_thread
    if _imu_thread and _imu_thread.is_alive():
        return
    _imu_stop.clear()
    _imu_thread = threading.Thread(target=_imu_reader_loop, daemon=True)
    _imu_thread.start()


def stop_imu_reader():
    """停止独立 IMU 读取线程"""
    global _imu_thread
    _imu_stop.set()
    if _imu_thread:
        _imu_thread.join(timeout=3)
        _imu_thread = None


async def imu_data_handler(request):
    """Return latest IMU data from ZED camera."""
    with imu_cache['lock']:
        data = imu_cache['data']
    if data is None:
        return web.json_response({'ok': False, 'message': 'IMU data not available. Start ZED camera first.'})
    return web.json_response({'ok': True, 'data': data})


async def distance_sensor_handler(request):
    """Return latest distance sensor readings (SEN0492 laser range finders)."""
    with distance_cache['lock']:
        data = dict(distance_cache['data'])
        ok = distance_cache['ok']
        ts = distance_cache['timestamp']
    return web.json_response({
        'ok': ok,
        'data': {
            'front': data.get('front'),
            'right': data.get('right'),
            'rear': data.get('rear'),
            'left': data.get('left'),
            'unit': 'mm',
        },
        'timestamp': ts,
    })




async def imu_ws_handler(request):
    """WebSocket endpoint to push IMU data at ~20Hz."""
    ws = web.WebSocketResponse()
    await ws.prepare(request)

    logger.info("IMU WebSocket client connected")

    try:
        while not ws.closed:
            with imu_cache['lock']:
                data = imu_cache['data']
            if data is not None:
                payload = json.dumps({'ok': True, 'data': data})
                try:
                    await ws.send_str(payload)
                except Exception:
                    break
            await asyncio.sleep(0.05)  # 20Hz push rate
    except Exception as e:
        logger.debug(f"IMU WebSocket error: {e}")
    finally:
        logger.info("IMU WebSocket client disconnected")

    return ws


async def distance_ws_handler(request):
    """WebSocket endpoint to push distance sensor data at ~10Hz."""
    ws = web.WebSocketResponse()
    await ws.prepare(request)

    logger.info("Distance WebSocket client connected")

    try:
        while not ws.closed:
            with distance_cache['lock']:
                data = dict(distance_cache['data'])
                ok = distance_cache['ok']
                ts = distance_cache['timestamp']
            payload = json.dumps({
                'ok': ok,
                'data': {
                    'front': data.get('front'),
                    'right': data.get('right'),
                    'rear': data.get('rear'),
                    'left': data.get('left'),
                    'unit': 'mm',
                },
                'timestamp': ts,
            })
            try:
                await ws.send_str(payload)
            except Exception:
                break
            await asyncio.sleep(0.1)  # 10Hz push rate
    except Exception as e:
        logger.debug(f"Distance WebSocket error: {e}")
    finally:
        logger.info("Distance WebSocket client disconnected")

    return ws


async def on_shutdown(app):
    visual_reloc_mgr.stop()
    f710_mgr.stop()
    recording_mgr.stop()
    camera_mgr.stop_all()
    # Close all ws proxy connections
    for ws in app.get('ws_proxies', set()):
        await ws.close()


async def rosbridge_proxy_handler(request):
    """Proxy WebSocket connections to rosbridge_server on localhost:9090."""
    ws_client = web.WebSocketResponse()
    await ws_client.prepare(request)

    # Connect to rosbridge
    rosbridge_url = 'http://localhost:9090'
    try:
        async with aiohttp.ClientSession() as session:
            async with session.ws_connect(rosbridge_url) as ws_ros:
                # Store for cleanup on shutdown
                ws_proxies = request.app.setdefault('ws_proxies', set())
                ws_proxies.add(ws_client)

                async def forward_to_ros():
                    """Forward messages from browser → rosbridge."""
                    try:
                        async for msg in ws_client:
                            if msg.type == WSMsgType.TEXT:
                                await ws_ros.send_str(msg.data)
                            elif msg.type == WSMsgType.BINARY:
                                await ws_ros.send_bytes(msg.data)
                            elif msg.type == WSMsgType.ERROR:
                                break
                    except Exception:
                        pass

                async def forward_to_client():
                    """Forward messages from rosbridge → browser."""
                    try:
                        async for msg in ws_ros:
                            if msg.type == WSMsgType.TEXT:
                                await ws_client.send_str(msg.data)
                            elif msg.type == WSMsgType.BINARY:
                                await ws_client.send_bytes(msg.data)
                            elif msg.type == WSMsgType.ERROR:
                                break
                    except Exception:
                        pass

                # Run both directions concurrently
                await asyncio.gather(
                    forward_to_ros(),
                    forward_to_client(),
                    return_exceptions=True,
                )

                ws_proxies.discard(ws_client)
    except Exception as e:
        logger.warning(f"rosbridge proxy connection failed: {e}")
        if not ws_client.closed:
            await ws_client.close()

    return ws_client


# ── Master Lock: only one page can control at a time ──
_master_lock = {
    'session_id': None,   # unique ID assigned to each tab
    'last_seen': 0,       # timestamp of last heartbeat
    'addr': '',           # client IP for display
}
import uuid, time

async def master_request_handler(request):
    """Request master control. Returns {master: true/false, holder: addr}.
    With steal=true: force-grab the lock (used when a new tab connects).
    With steal=false: normal heartbeat, don't grab from others.
    """
    global _master_lock
    try:
        data = await request.json()
    except Exception:
        data = {}
    sid = data.get('session_id', '')
    steal = data.get('steal', False)
    addr = request.remote or ''

    now = time.time()
    # Auto-release if master hasn't heartbeated in 15s
    if _master_lock['session_id'] and (now - _master_lock['last_seen']) > 15:
        logger.info(f"Master lock auto-released (timeout from {_master_lock['addr']})")
        _master_lock['session_id'] = None

    if not sid:
        sid = str(uuid.uuid4())

    if _master_lock['session_id'] is None:
        # Lock is free, take it
        _master_lock['session_id'] = sid
        _master_lock['last_seen'] = now
        _master_lock['addr'] = addr
        logger.info(f"Master lock acquired by {addr} (sid={sid[:8]})")
        return web.json_response({'master': True, 'session_id': sid})
    elif _master_lock['session_id'] == sid:
        # Already master, refresh heartbeat
        _master_lock['last_seen'] = now
        _master_lock['addr'] = addr
        return web.json_response({'master': True, 'session_id': sid})
    elif steal:
        # Steal mode: new tab forcefully takes the lock
        old_addr = _master_lock['addr']
        logger.info(f"Master lock stolen by {addr} (sid={sid[:8]}), previous holder: {old_addr}")
        _master_lock['session_id'] = sid
        _master_lock['last_seen'] = now
        _master_lock['addr'] = addr
        return web.json_response({'master': True, 'session_id': sid})
    else:
        # Someone else is master
        return web.json_response({'master': False, 'session_id': sid, 'holder': _master_lock['addr']})

async def master_release_handler(request):
    """Release master control."""
    global _master_lock
    try:
        data = await request.json()
    except Exception:
        data = {}
    sid = data.get('session_id', '')
    if _master_lock['session_id'] == sid:
        _master_lock['session_id'] = None
        logger.info(f"Master lock released by {_master_lock['addr']}")
    return web.json_response({'ok': True})

async def master_status_handler(request):
    """Check who is master."""
    now = time.time()
    if _master_lock['session_id'] and (now - _master_lock['last_seen']) > 15:
        _master_lock['session_id'] = None
    is_master = _master_lock['session_id'] is not None
    return web.json_response({
        'locked': is_master,
        'holder': _master_lock['addr'] if is_master else None,
    })


def create_app():
    app = web.Application()
    app.on_shutdown.append(on_shutdown)

    # Routes
    app.router.add_get('/', index_handler)
    app.router.add_static('/static', STATIC_DIR, name='static')
    app.router.add_get('/camera/{name}', camera_stream_handler)
    app.router.add_post('/camera/start', camera_start_handler)
    app.router.add_post('/camera/stop', camera_stop_handler)
    app.router.add_get('/camera/status', camera_status_handler)
    app.router.add_post('/recording/start', recording_start_handler)
    app.router.add_post('/recording/stop', recording_stop_handler)
    app.router.add_get('/recording/status', recording_status_handler)
    app.router.add_post('/f710/start', f710_start_handler)
    app.router.add_post('/f710/stop', f710_stop_handler)
    app.router.add_get('/f710/status', f710_status_handler)
    app.router.add_get('/api/visual_relocalization/status', visual_relocalization_status_handler)
    app.router.add_post('/api/visual_relocalization/record', visual_relocalization_record_handler)
    app.router.add_post('/api/visual_relocalization/start', visual_relocalization_start_handler)
    app.router.add_post('/api/visual_relocalization/stop', visual_relocalization_stop_handler)
    app.router.add_get('/api/imu', imu_data_handler)
    app.router.add_get('/api/sensors/distance', distance_sensor_handler)
    app.router.add_post("/api/exit-kiosk", exit_kiosk_handler)
    # Start standalone IMU reader (lightweight, no video/depth)
    start_imu_reader()
    app.router.add_post('/api/master/request', master_request_handler)
    app.router.add_post('/api/master/release', master_release_handler)
    app.router.add_get('/api/master/status', master_status_handler)
    app.router.add_get('/ws', rosbridge_proxy_handler)
    app.router.add_get('/ws/imu', imu_ws_handler)
    app.router.add_get('/ws/distance', distance_ws_handler)

    return app


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='SVTROBO Web Control Server')
    parser.add_argument('--host', default='0.0.0.0', help='Host to bind (default: 0.0.0.0)')
    parser.add_argument('--port', type=int, default=8080, help='Port to bind (default: 8080)')
    args = parser.parse_args()

    logger.info(f"Starting server at http://{args.host}:{args.port}")
    web.run_app(create_app(), host=args.host, port=args.port, print=logger.info)
