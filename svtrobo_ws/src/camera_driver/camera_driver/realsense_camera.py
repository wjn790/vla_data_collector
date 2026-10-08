"""
RealSense D405 相机采集模块

用法:
    from camera_driver import RealSenseCamera

    cam = RealSenseCamera(serial='409122272399')
    cam.start()
    color, depth = cam.capture()    # (H,W,3) BGR, (H,W) uint16 depth
    cam.stop()

    # 上下文管理器
    with RealSenseCamera(serial='409122272399') as cam:
        color, depth = cam.capture()
"""

import os
import time
from datetime import datetime
import numpy as np
import cv2

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None


class RealSenseCamera:
    """RealSense D405 相机管理类"""

    def __init__(self, serial='', color_size=(1280, 720), depth_size=(1280, 720), fps=5,
                 color_only=True, warmup_frames=None):
        """
        Args:
            serial: 设备序列号，空字符串则自动选择第一个设备
            color_size: (width, height) 彩色图分辨率
            depth_size: (width, height) 深度图分辨率 (仅 color_only=False 时生效)
            fps: 帧率
            color_only: 只采集彩色图，不计算深度（默认 True）
            warmup_frames: 预热帧数（None=自动: depth模式15, color_only模式5）
        """
        if rs is None:
            raise ImportError('pyrealsense2 未安装: pip install pyrealsense2')

        self.serial = serial
        self.color_w, self.color_h = color_size
        self.depth_w, self.depth_h = depth_size
        self.fps = fps
        self.color_only = color_only
        # 预热帧数: 显式传入用传入值，否则 depth 模式 15, color_only 模式 5
        self.warmup_frames = warmup_frames if warmup_frames is not None else (5 if color_only else 15)

        self.pipeline = None
        self.align = None
        self.profile = None
        self.depth_scale = 0.001
        self.is_running = False

    def _check_device_connected(self):
        """快速检查指定序列号的设备是否存在（<0.1秒）"""
        ctx = rs.context()
        devices = ctx.query_devices()
        if not self.serial:
            return len(devices) > 0
        for d in devices:
            try:
                sn = d.get_info(rs.camera_info.serial_number)
                if sn == self.serial:
                    return True
            except Exception:
                continue
        return False

    def start(self):
        """启动相机并预热"""
        if self.is_running:
            return

        # 快速检查设备是否存在，避免 pipeline.start 长时间阻塞
        if self.serial and not self._check_device_connected():
            raise RuntimeError(f'Device {self.serial} not found (quick check)')

        self.pipeline = rs.pipeline()
        config = rs.config()

        if self.serial:
            config.enable_device(self.serial)

        config.enable_stream(rs.stream.color, self.color_w, self.color_h, rs.format.bgr8, self.fps)
        if not self.color_only:
            config.enable_stream(rs.stream.depth, self.depth_w, self.depth_h, rs.format.z16, self.fps)

        self.profile = self.pipeline.start(config)

        if not self.color_only:
            depth_sensor = self.profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            self.align = rs.align(rs.stream.color)

        # 预热
        for _ in range(self.warmup_frames):
            self.pipeline.wait_for_frames()

        mode = 'color-only' if self.color_only else 'color+depth'
        print(f'[RealSense] 启动完成 ({mode}), SN: {self.serial or "auto"}')
        self.is_running = True

    def stop(self):
        """停止相机"""
        if self.pipeline:
            try:
                self.pipeline.stop()
            except Exception:
                pass
        self.pipeline = None
        self.align = None
        self.profile = None
        self.is_running = False

    def capture(self):
        """
        采集一帧

        Returns:
            color_only=True:  (color, None)
            color_only=False: (color, depth) - 对齐后的 BGR + uint16 深度
            失败: (None, None)
        """
        if not self.is_running:
            raise RuntimeError('相机未启动，请先调用 start()')

        frames = self.pipeline.wait_for_frames(5000)

        if self.color_only:
            color_frame = frames.get_color_frame()
            if not color_frame:
                return None, None
            color = np.asanyarray(color_frame.get_data())
            return color, None

        aligned = self.align.process(frames)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()

        if not color_frame or not depth_frame:
            return None, None

        color = np.asanyarray(color_frame.get_data())
        depth = np.asanyarray(depth_frame.get_data())
        return color, depth

    def get_intrinsics(self):
        """获取彩色相机内参

        Returns:
            dict: {fx, fy, ppx, ppy, width, height, coeffs}
        """
        if not self.profile:
            return None

        color_stream = self.profile.get_stream(rs.stream.color)
        intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        return {
            'fx': intrinsics.fx,
            'fy': intrinsics.fy,
            'ppx': intrinsics.ppx,
            'ppy': intrinsics.ppy,
            'width': intrinsics.width,
            'height': intrinsics.height,
            'coeffs': list(intrinsics.coeffs),
        }

    def get_depth_scale(self):
        """获取深度比例因子 (深度像素值 * scale = 米)"""
        return self.depth_scale

    def capture_and_save(self, save_dir=None, name=None):
        """采集并保存到磁盘

        Args:
            save_dir: 保存目录，默认 camera_driver/captures/
            name: 文件名前缀（如 d405_1），默认用 SN

        Returns:
            (color_path, depth_path) 或 (None, None)
        """
        color, depth = self.capture()
        if color is None:
            return None, None

        if save_dir is None:
            save_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'captures')
        os.makedirs(save_dir, exist_ok=True)

        tag = name or self.serial or 'd405'
        ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')

        color_path = os.path.join(save_dir, f'{tag}_{ts}_color.png')
        depth_path = os.path.join(save_dir, f'{tag}_{ts}_depth.png')

        cv2.imwrite(color_path, color)
        cv2.imwrite(depth_path, depth)
        return color_path, depth_path

    def capture_pointcloud(self, max_depth=1.0):
        """采集彩色点云 (mm 精度)

        Args:
            max_depth: 最大深度(米)，超过此距离的点过滤掉

        Returns:
            numpy (N, 6): [x, y, z, r, g, b] 每行一个点
        """
        color, depth = self.capture()
        if color is None:
            return None

        intrinsics = self.get_intrinsics()
        fx, fy = intrinsics['fx'], intrinsics['fy']
        cx, cy = intrinsics['ppx'], intrinsics['ppy']

        h, w = depth.shape
        # 深度转米
        depth_m = depth.astype(np.float32) * self.depth_scale

        mask = (depth_m > 0) & (depth_m < max_depth)
        ys, xs = np.where(mask)
        zs = depth_m[mask]

        xs_3d = (xs - cx) * zs / fx
        ys_3d = (ys - cy) * zs / fy

        colors = color[mask]
        points = np.column_stack([xs_3d, ys_3d, zs, colors.astype(np.float32)])
        return points

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.stop()
        return False

    @staticmethod
    def list_devices():
        """列出所有连接的 RealSense 设备

        Returns:
            list of dict: [{name, serial, firmware}]
        """
        if rs is None:
            return []
        ctx = rs.context()
        devices = []
        for dev in ctx.query_devices():
            devices.append({
                'name': dev.get_info(rs.camera_info.name),
                'serial': dev.get_info(rs.camera_info.serial_number),
                'firmware': dev.get_info(rs.camera_info.firmware_version),
            })
        return devices


# 快速测试
if __name__ == '__main__':
    devices = RealSenseCamera.list_devices()
    print(f'检测到 {len(devices)} 个 RealSense 设备:')
    for d in devices:
        print(f"  {d['name']} SN:{d['serial']} FW:{d['firmware']}")

    if devices:
        print(f'\n使用设备 {devices[0]["serial"]} 测试采集...')
        with RealSenseCamera(serial=devices[0]['serial']) as cam:
            for i in range(5):
                color, depth = cam.capture()
                if color is not None:
                    print(f'  帧 {i}: color {color.shape}, depth {depth.shape}, '
                          f'深度范围 [{depth.min()}, {depth.max()}]')
                else:
                    print(f'  帧 {i}: 采集失败')
