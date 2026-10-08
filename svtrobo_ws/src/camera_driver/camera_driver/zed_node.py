"""
ZED 2i ROS2 相机节点

发布 Topic:
  - ~/zed/left/image_raw   (sensor_msgs/Image)
  - ~/zed/right/image_raw  (sensor_msgs/Image)
  - ~/zed/depth/image_raw  (sensor_msgs/Image)
  - ~/zed/left/camera_info (sensor_msgs/CameraInfo)

双模式: SDK 优先, OpenCV 降级(无深度)

用法:
  ros2 run camera_driver zed_node --ros-args \
    -p resolution:=HD720 -p depth_mode:=NEURAL
"""

import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

try:
    import pyzed.sl as sl
    HAS_ZED_SDK = True
except ImportError:
    HAS_ZED_SDK = False

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None


class ZEDNode(Node):
    """ZED 2i ROS2 发布节点"""

    def __init__(self):
        super().__init__('zed_node')

        # 参数
        self.declare_parameter('resolution', 'HD720')
        self.declare_parameter('fps', 30)
        self.declare_parameter('depth_mode', 'NEURAL')
        self.declare_parameter('min_depth', 100.0)
        self.declare_parameter('frame_id', 'zed_link')
        self.declare_parameter('force_opencv', False)

        self.bridge = CvBridge() if CvBridge else None
        self.frame_id = self.get_parameter('frame_id').value
        fps = self.get_parameter('fps').value
        self.use_sdk = HAS_ZED_SDK and not self.get_parameter('force_opencv').value

        # ZED SDK 对象
        self.zed = None
        self.runtime_params = None
        # OpenCV 对象
        self.cap = None
        self.stereo_matcher = None

        # 发布者
        self.left_pub = self.create_publisher(Image, 'zed/left/image_raw', 10)
        self.right_pub = self.create_publisher(Image, 'zed/right/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, 'zed/depth/image_raw', 10)
        self.left_info_pub = self.create_publisher(CameraInfo, 'zed/left/camera_info', 10)

        # 启动相机
        if self.use_sdk:
            success = self._start_sdk()
        else:
            success = self._start_opencv()

        if not success:
            raise RuntimeError('Failed to start ZED camera')

        # 定时器
        period = 1.0 / fps if fps > 0 else 0.033
        self.timer = self.create_timer(period, self._timer_callback)

        self.get_logger().info(
            f'ZED node started: mode={"SDK" if self.use_sdk else "OpenCV"}, '
            f'resolution={self.get_parameter("resolution").value}'
        )

    # ---- SDK 模式 ----

    def _resolution_map(self, name):
        return {
            'HD2K': sl.RESOLUTION.HD2K,
            'HD1080': sl.RESOLUTION.HD1080,
            'HD720': sl.RESOLUTION.HD720,
            'VGA': sl.RESOLUTION.VGA,
        }.get(name, sl.RESOLUTION.HD720)

    def _depth_mode_map(self, name):
        return {
            'NEURAL': sl.DEPTH_MODE.NEURAL,
            'ULTRA': sl.DEPTH_MODE.ULTRA,
            'QUALITY': sl.DEPTH_MODE.QUALITY,
            'PERFORMANCE': sl.DEPTH_MODE.PERFORMANCE,
        }.get(name, sl.DEPTH_MODE.NEURAL)

    def _start_sdk(self):
        try:
            self.zed = sl.Camera()
            params = sl.InitParameters()
            params.camera_resolution = self._resolution_map(
                self.get_parameter('resolution').value)
            params.camera_fps = self.get_parameter('fps').value
            params.depth_mode = self._depth_mode_map(
                self.get_parameter('depth_mode').value)
            params.coordinate_units = sl.UNIT.MILLIMETER
            params.depth_minimum_distance = self.get_parameter('min_depth').value

            err = self.zed.open(params)
            if err != sl.ERROR_CODE.SUCCESS:
                self.get_logger().warn(f'SDK 打开失败: {err}, 降级到 OpenCV')
                self.use_sdk = False
                return self._start_opencv()

            self.runtime_params = sl.RuntimeParameters()

            # 预热
            image = sl.Mat()
            for _ in range(30):
                if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                    self.zed.retrieve_image(image, sl.VIEW.LEFT)

            return True

        except Exception as e:
            self.get_logger().warn(f'SDK 异常: {e}, 降级到 OpenCV')
            self.use_sdk = False
            return self._start_opencv()

    # ---- OpenCV 模式 ----

    def _start_opencv(self):
        res_map = {
            'HD2K': (4416, 1242),
            'HD1080': (3840, 1080),
            'HD720': (2560, 720),
            'VGA': (1344, 376),
        }
        target = res_map.get(self.get_parameter('resolution').value, (2560, 720))

        self.get_logger().info(f'OpenCV 模式: 搜索 ZED ({target[0]}x{target[1]})...')

        for dev_id in range(10):
            cap = cv2.VideoCapture(dev_id)
            if not cap.isOpened():
                continue
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, target[0])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, target[1])
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if w == target[0] and h == target[1]:
                self.cap = cap
                self.get_logger().info(f'找到 ZED: /dev/video{dev_id}')
                break
            cap.release()

        if self.cap is None:
            self.get_logger().error('未找到 ZED 设备')
            return False

        # 初始化 SGBM 立体匹配
        self.stereo_matcher = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=128,
            blockSize=5,
            P1=8 * 3 * 25,
            P2=32 * 3 * 25,
            disp12MaxDiff=1,
            uniquenessRatio=10,
            speckleWindowSize=100,
            speckleRange=32,
        )

        # 预热
        for _ in range(30):
            self.cap.read()

        return True

    # ---- 定时回调 ----

    def _timer_callback(self):
        now = self.get_clock().now().to_msg()
        if self.use_sdk:
            self._publish_sdk(now)
        else:
            self._publish_opencv(now)

    def _publish_sdk(self, stamp):
        err = self.zed.grab(self.runtime_params)
        if err != sl.ERROR_CODE.SUCCESS:
            return

        image_sl = sl.Mat()
        depth_sl = sl.Mat()

        self.zed.retrieve_image(image_sl, sl.VIEW.LEFT)
        self.zed.retrieve_image(depth_sl, sl.MEASURE.DEPTH)

        # 左眼
        left_bgra = image_sl.get_data()
        if left_bgra is None:
            return
        left_bgr = left_bgra[:, :, :3]
        self.left_pub.publish(self._to_image(left_bgr, 'bgr8', stamp))

        # 深度 (float32 mm)
        depth_data = depth_sl.get_data()
        if depth_data is not None:
            depth_uint16 = np.clip(depth_data, 0, 65535).astype(np.uint16)
            self.depth_pub.publish(self._to_image(depth_uint16, '16UC1', stamp))

        # Camera info
        cam_info = self.zed.get_camera_information().camera_configuration
        calib = cam_info.calibration_parameters
        left_ci = self._make_camera_info(
            calib.left_cam, int(left_bgr.shape[1]), int(left_bgr.shape[0]), stamp)
        self.left_info_pub.publish(left_ci)

    def _publish_opencv(self, stamp):
        ret, frame = self.cap.read()
        if not ret:
            return

        h, w = frame.shape[:2]
        half_w = w // 2
        left = frame[:, :half_w, :]
        right = frame[:, half_w:, :]

        self.left_pub.publish(self._to_image(left, 'bgr8', stamp))
        self.right_pub.publish(self._to_image(right, 'bgr8', stamp))

        # SGBM 深度估算
        left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)
        disp = self.stereo_matcher.compute(left_gray, right_gray).astype(np.float32) / 16.0
        baseline = 120.0
        focal = 700.0 if w <= 3000 else 1400.0
        with np.errstate(divide='ignore'):
            depth = (focal * baseline) / disp
        depth[disp <= 0] = 0
        depth = np.clip(depth, 0, 65535).astype(np.uint16)
        self.depth_pub.publish(self._to_image(depth, '16UC1', stamp))

    # ---- 工具方法 ----

    def _to_image(self, img, encoding, stamp):
        if self.bridge:
            msg = self.bridge.cv2_to_imgmsg(img, encoding)
        else:
            msg = Image()
            msg.height, msg.width = img.shape[:2]
            msg.encoding = encoding
            msg.step = msg.width * (img.dtype.itemsize * img.shape[2] if img.ndim == 3 else img.dtype.itemsize)
            msg.data = img.tobytes()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        return msg

    def _make_camera_info(self, cam, width, height, stamp):
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width = width
        info.height = height
        info.distortion_model = 'plumb_bob'
        fx = cam.fx if hasattr(cam, 'fx') else cam.get('fx', 0)
        fy = cam.fy if hasattr(cam, 'fy') else cam.get('fy', 0)
        cx = cam.cx if hasattr(cam, 'cx') else cam.get('cx', 0)
        cy = cam.cy if hasattr(cam, 'cy') else cam.get('cy', 0)
        info.k = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        info.d = list(cam.disto) if hasattr(cam, 'disto') else [0.0] * 5
        info.p = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]
        return info

    def destroy_node(self):
        if self.use_sdk and self.zed:
            try:
                if self.zed.is_opened():
                    self.zed.close()
            except Exception:
                pass
        if self.cap and self.cap.isOpened():
            self.cap.release()
        self.get_logger().info('ZED node destroyed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZEDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
