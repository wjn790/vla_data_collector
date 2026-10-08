"""
相机采集模块

用法:
    from camera_driver import RealSenseCamera, ZEDCamera

    # RealSense D405
    cam = RealSenseCamera(serial='409122272399')
    cam.start()
    color, depth = cam.capture()      # numpy arrays
    cam.stop()

    # ZED 2i
    zed = ZEDCamera(resolution='HD720')
    zed.start()
    left, depth = zed.capture()       # numpy arrays
    zed.stop()
"""

from .realsense_camera import RealSenseCamera
from .zed_camera import ZEDCamera
from .zed_imu import ZEDIMU

__all__ = ['RealSenseCamera', 'ZEDCamera', 'ZEDIMU']
