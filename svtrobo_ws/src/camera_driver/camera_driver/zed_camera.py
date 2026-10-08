"""
ZED 2i 相机采集模块

双模式: SDK (完整功能) / OpenCV (降级, 无精确深度)

用法:
    from camera_driver import ZEDCamera

    # SDK 模式
    zed = ZEDCamera(resolution='HD720', depth_mode='NEURAL')
    zed.start()
    left, depth = zed.capture()   # BGR uint8, float32 depth (mm)
    zed.stop()

    # 上下文管理器
    with ZEDCamera() as zed:
        left, depth = zed.capture()
"""

import os
import numpy as np
import cv2
import time
from datetime import datetime

try:
    import pyzed.sl as sl
    HAS_ZED_SDK = True
except ImportError:
    HAS_ZED_SDK = False


class ZEDCamera:
    """ZED 2i 相机管理类"""

    # ZED 2i baseline (mm)
    BASELINE = 120.0

    def __init__(self, resolution='HD720', fps=30, depth_mode='NEURAL',
                 min_depth=100.0, force_opencv=False, color_only=True):
        """
        Args:
            resolution: HD2K / HD1080 / HD720 / VGA
            fps: 帧率
            depth_mode: NEURAL / ULTRA / QUALITY / PERFORMANCE (仅 color_only=False 时生效)
            min_depth: 最小深度 (mm)
            force_opencv: 强制使用 OpenCV 模式
            color_only: 只采集彩色图，不计算深度（默认 True）
        """
        self.resolution = resolution
        self.fps = fps
        self.depth_mode = depth_mode
        self.min_depth = min_depth
        self.color_only = color_only

        self.use_sdk = HAS_ZED_SDK and not force_opencv
        self.is_running = False

        # SDK 模式对象
        self.zed = None
        self.runtime_params = None
        # Reusable sl.Mat objects (avoid per-frame allocation)
        self._img_mat = None
        self._right_mat = None
        self._depth_mat = None
        self._pc_mat = None
        self._latest_pc = None
        self._svo_recording = False
        self._svo_path = None
        # OpenCV 模式对象
        self.cap = None
        self.stereo_matcher = None
        self._target_size = None

    # ---- 启动/停止 ----

    def start(self):
        """启动相机"""
        if self.is_running:
            return
        if self.use_sdk:
            if not self._start_sdk():
                self.use_sdk = False
                self._start_opencv()
        else:
            self._start_opencv()
        self.is_running = True

    def stop(self):
        """停止相机"""
        self.stop_svo_recording()
        if self.use_sdk and self.zed:
            try:
                if self.zed.is_opened():
                    self.zed.close()
            except Exception:
                pass
        if self.cap and self.cap.isOpened():
            self.cap.release()
        self.cap = None
        self.zed = None
        self.is_running = False

    def _start_sdk(self):
        """SDK 模式启动"""
        try:
            self.zed = sl.Camera()
            params = sl.InitParameters()
            params.camera_resolution = self._res_map(self.resolution)
            params.camera_fps = self.fps
            if self.color_only:
                params.depth_mode = sl.DEPTH_MODE.NONE
            else:
                params.depth_mode = self._depth_map(self.depth_mode)
                params.coordinate_units = sl.UNIT.MILLIMETER
                params.depth_minimum_distance = self.min_depth

            err = self.zed.open(params)
            if err != sl.ERROR_CODE.SUCCESS:
                print(f'[ZED] SDK 打开失败: {err}, 降级到 OpenCV')
                return False

            self.runtime_params = sl.RuntimeParameters()

            # 预分配 sl.Mat (复用，避免每帧分配)
            self._img_mat = sl.Mat()
            self._right_mat = sl.Mat()
            if not self.color_only:
                self._depth_mat = sl.Mat()
                self._pc_mat = sl.Mat()

            # 预热
            warmup_frames = 5 if self.color_only else 15
            for _ in range(warmup_frames):
                if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                    self.zed.retrieve_image(self._img_mat, sl.VIEW.LEFT)

            mode = 'color-only' if self.color_only else self.depth_mode
            cam_info = self.zed.get_camera_information()
            print(f'[ZED] SDK 模式启动 ({mode}), SN: {cam_info.serial_number}')
            return True

        except Exception as e:
            print(f'[ZED] SDK 异常: {e}, 降级到 OpenCV')
            return False

    def _start_opencv(self):
        """OpenCV V4L2 模式启动（ZED 无 SDK 时的降级方案）"""
        size_map = {
            'HD2K': (4416, 1242), 'HD1080': (3840, 1080),
            'HD720': (2560, 720), 'VGA': (1344, 376),
        }
        self._target_size = size_map.get(self.resolution, (1344, 376))

        # 优先使用 V4L2 后端（ZED 的 GStreamer 兼容性差）
        for dev_id in range(20):
            cap = cv2.VideoCapture(dev_id, cv2.CAP_V4L2)
            if not cap.isOpened():
                continue
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._target_size[0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._target_size[1])
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

            # 验证是 ZED: side-by-side 格式 (宽是高的 3~4 倍)
            if w > 0 and h > 0 and w / h >= 3.0:
                self.cap = cap
                self._target_size = (w, h)
                break
            cap.release()

        if self.cap is None:
            raise RuntimeError('未找到 ZED 相机设备')

        # 仅在需要深度时初始化 SGBM
        if not self.color_only:
            self.stereo_matcher = cv2.StereoSGBM_create(
                minDisparity=0, numDisparities=128, blockSize=5,
                P1=8 * 3 * 25, P2=32 * 3 * 25,
                disp12MaxDiff=1, uniquenessRatio=10,
                speckleWindowSize=100, speckleRange=32,
            )

        # 预热
        warmup_frames = 5 if self.color_only else 15
        for _ in range(warmup_frames):
            self.cap.read()

        mode = 'color-only' if self.color_only else 'depth'
        print(f'[ZED] OpenCV 模式启动 ({mode}), {self._target_size[0]}x{self._target_size[1]}')

    # ---- 采集 ----

    def capture(self):
        """
        采集一帧

        Returns:
            SDK 模式: (left_bgr, depth_float32_mm)  — 兼容旧接口，右眼丢弃
            OpenCV 模式: (left_bgr, depth_uint16_mm)
            失败: (None, None)
        """
        if not self.is_running:
            raise RuntimeError('相机未启动，请先调用 start()')
        if self.use_sdk:
            left, right, depth = self._capture_sdk()
            return left, depth
        return self._capture_opencv()

    def start_svo_recording(self, output_path, compression='H265', target_framerate=0, bitrate=0):
        """Start recording the opened SDK camera stream to an SVO/SVO2 file."""
        if not self.use_sdk or not self.zed or not self.is_running:
            raise RuntimeError('SVO recording requires a running ZED SDK camera')
        if self._svo_recording:
            return self._svo_path

        output_path = os.fspath(output_path)
        os.makedirs(os.path.dirname(output_path), exist_ok=True)

        mode_map = {
            'H264': sl.SVO_COMPRESSION_MODE.H264,
            'H264_LOSSLESS': sl.SVO_COMPRESSION_MODE.H264_LOSSLESS,
            'H265': sl.SVO_COMPRESSION_MODE.H265,
            'H265_LOSSLESS': sl.SVO_COMPRESSION_MODE.H265_LOSSLESS,
            'LOSSLESS': sl.SVO_COMPRESSION_MODE.LOSSLESS,
        }
        comp = mode_map.get(str(compression).upper(), sl.SVO_COMPRESSION_MODE.H265)
        params = sl.RecordingParameters(output_path, comp)
        if target_framerate:
            params.target_framerate = int(target_framerate)
        if bitrate:
            params.bitrate = int(bitrate)

        err = self.zed.enable_recording(params)
        if err != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f'ZED SVO recording failed: {err}')
        self._svo_recording = True
        self._svo_path = output_path
        print(f'[ZED] SVO recording started: {output_path}')
        return output_path

    def stop_svo_recording(self):
        """Stop SVO recording if it is active."""
        if self._svo_recording and self.zed:
            try:
                self.zed.disable_recording()
                print(f'[ZED] SVO recording stopped: {self._svo_path}')
            finally:
                path = self._svo_path
                self._svo_recording = False
                self._svo_path = None
                return path
        return None

    def _capture_sdk(self):
        err = self.zed.grab(self.runtime_params)
        if err != sl.ERROR_CODE.SUCCESS:
            return None, None, None

        self.zed.retrieve_image(self._img_mat, sl.VIEW.LEFT)
        bgra = self._img_mat.get_data()
        if bgra is None:
            return None, None, None
        left = bgra[:, :, :3]  # BGRA -> BGR (drop alpha)

        # Retrieve right eye (always, for stereo recording)
        self.zed.retrieve_image(self._right_mat, sl.VIEW.RIGHT)
        right_bgra = self._right_mat.get_data()
        right = right_bgra[:, :, :3] if right_bgra is not None else None

        if self.color_only:
            return left, right, None

        self.zed.retrieve_measure(self._depth_mat, sl.MEASURE.DEPTH)
        depth = self._depth_mat.get_data()

        # Retrieve point cloud in same grab (shares GPU data, minimal overhead)
        if self._pc_mat is not None:
            try:
                self.zed.retrieve_measure(self._pc_mat, sl.MEASURE.XYZRGBA)
                pc_data = self._pc_mat.get_data()
                self._latest_pc = pc_data if pc_data is not None else self._latest_pc
            except Exception:
                pass

        return left, right, depth

    def _capture_opencv(self):
        ret, frame = self.cap.read()
        if not ret:
            return None, None

        h, w = frame.shape[:2]
        half = w // 2
        left = frame[:, :half, :]

        if self.color_only:
            return left, None

        right = frame[:, half:, :]

        # SGBM 深度估算
        lg = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        rg = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        disp = self.stereo_matcher.compute(lg, rg).astype(np.float32) / 16.0

        focal = 700.0 if w <= 3000 else 1400.0
        with np.errstate(divide='ignore'):
            depth = (focal * self.BASELINE) / disp
        depth[disp <= 0] = 0
        depth = np.clip(depth, 0, 65535).astype(np.uint16)
        return left, depth

    def capture_and_save(self, save_dir=None, name='zed'):
        """采集并保存到磁盘

        Args:
            save_dir: 保存目录，默认 camera_driver/captures/
            name: 文件名前缀

        Returns:
            (left_path, depth_path) 或 (None, None)
        """
        left, depth = self.capture()
        if left is None:
            return None, None

        if save_dir is None:
            save_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'captures')
        os.makedirs(save_dir, exist_ok=True)

        ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')

        left_path = os.path.join(save_dir, f'{name}_{ts}_left.png')
        depth_path = os.path.join(save_dir, f'{name}_{ts}_depth.png')

        cv2.imwrite(left_path, left)
        cv2.imwrite(depth_path, depth)
        return left_path, depth_path

    def capture_stereo(self):
        """采集左右眼图像 + 深度 (复用预分配 sl.Mat)

        Returns:
            (left, right, depth)
        """
        if self.use_sdk:
            return self._capture_sdk()  # 已返回 (left, right, depth)，复用 _img_mat/_right_mat/_depth_mat
        else:
            ret, frame = self.cap.read()
            if not ret:
                return None, None, None
            h, w = frame.shape[:2]
            half = w // 2
            left = frame[:, :half, :]
            right = frame[:, half:, :]
            _, depth = self._capture_opencv()
            return left, right, depth

    def get_imu_data(self):
        """获取最新 IMU 数据（仅 SDK 模式可用）

        Returns:
            dict with keys: accel, gyro_dps, gyro_rad, mag, imu_temp, pressure, env_temp, timestamp_ns
            None if not available
        """
        if not self.use_sdk or not self.zed or not self.is_running:
            return None
        try:
            import math
            sensors = sl.SensorsData()
            if self.zed.get_sensors_data(sensors, sl.TIME_REFERENCE.IMAGE) != sl.ERROR_CODE.SUCCESS:
                return None
            imu = sensors.get_imu_data()
            if not imu.is_available:
                return None
            acc = imu.get_linear_acceleration()
            gyro = imu.get_angular_velocity()
            mag_data = sensors.get_magnetometer_data()
            baro = sensors.get_barometer_data()
            temp_data = sensors.get_temperature_data()
            mag = mag_data.get_magnetic_field_calibrated() if mag_data.is_available else (0,0,0)
            imu_temp_raw = temp_data.get(sl.SENSOR_LOCATION.IMU)
            imu_temp = imu_temp_raw * 0.01 if imu_temp_raw and imu_temp_raw > 0 else 0.0
            baro_temp_raw = temp_data.get(sl.SENSOR_LOCATION.BAROMETER)
            env_temp = baro_temp_raw * 0.01 if baro_temp_raw and baro_temp_raw > 0 else 0.0
            ts_ns = imu.timestamp.get_nanoseconds()
            return {
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
            }
        except Exception:
            return None

    def get_intrinsics(self):
        """获取左眼相机内参

        Returns:
            dict: {fx, fy, cx, cy, width, height} 或 None
        """
        if not self.use_sdk or not self.zed:
            return None
        try:
            calib = self.zed.get_camera_information().camera_configuration.calibration_parameters
            cam = calib.left_cam
            return {
                'fx': cam.fx, 'fy': cam.fy,
                'cx': cam.cx, 'cy': cam.cy,
                'width': int(cam.image_size.width),
                'height': int(cam.image_size.height),
            }
        except Exception:
            return None

    def capture_pointcloud(self):
        """获取最近一帧 XYZRGBA 点云 (仅 SDK 模式 + depth 开启时可用)

        点云在每次 _capture_sdk() grab 时自动更新到 _latest_pc。
        此方法直接返回缓存，不再触发独立 retrieve。

        Returns:
            numpy (H, W, 4) float32 — X,Y,Z (mm) + RGBA 打包
            None if not available (color_only mode or no SDK)
        """
        return self._latest_pc

    # ---- 工具 ----

    def _res_map(self, name):
        return {'HD2K': sl.RESOLUTION.HD2K, 'HD1080': sl.RESOLUTION.HD1080,
                'HD720': sl.RESOLUTION.HD720, 'VGA': sl.RESOLUTION.VGA,
                }.get(name, sl.RESOLUTION.HD720)

    def _depth_map(self, name):
        return {'NEURAL': sl.DEPTH_MODE.NEURAL, 'ULTRA': sl.DEPTH_MODE.ULTRA,
                'QUALITY': sl.DEPTH_MODE.QUALITY, 'PERFORMANCE': sl.DEPTH_MODE.PERFORMANCE,
                }.get(name, sl.DEPTH_MODE.NEURAL)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.stop()
        return False


# 快速测试
if __name__ == '__main__':
    mode = 'SDK' if HAS_ZED_SDK else 'OpenCV'
    print(f'ZED 模式: {mode}')

    with ZEDCamera(resolution='HD720') as zed:
        for i in range(5):
            left, depth = zed.capture()
            if left is not None:
                print(f'  帧 {i}: left {left.shape}, depth {depth.shape} '
                      f'dtype={depth.dtype}')
            else:
                print(f'  帧 {i}: 采集失败')