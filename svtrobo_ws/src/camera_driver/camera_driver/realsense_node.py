"""
RealSense D405 ROS2 相机节点

发布 Topic:
  - ~/<namespace>/color/image_raw  (sensor_msgs/Image)
  - ~/<namespace>/depth/image_raw  (sensor_msgs/Image)
  - ~/<namespace>/color/camera_info (sensor_msgs/CameraInfo)
  - ~/<namespace>/depth/camera_info (sensor_msgs/CameraInfo)

用法:
  ros2 run camera_driver realsense_node --ros-args \
    -p serial_number:=409122272399 \
    -p namespace:=d405_1
"""

import time
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None


class RealSenseNode(Node):
    """RealSense D405 ROS2 发布节点"""

    def __init__(self):
        super().__init__('realsense_node')

        # 声明参数
        self.declare_parameter('serial_number', '')
        self.declare_parameter('namespace', 'd405_1')
        self.declare_parameter('color_width', 640)
        self.declare_parameter('color_height', 480)
        self.declare_parameter('depth_width', 640)
        self.declare_parameter('depth_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('frame_id', 'camera_link')

        if rs is None:
            self.get_logger().error('pyrealsense2 未安装，无法启动')
            raise RuntimeError('pyrealsense2 not available')

        self.bridge = CvBridge() if CvBridge else None
        self.pipeline = None
        self.align = None
        self.profile = None
        self.depth_scale = 0.001

        # 获取参数
        serial = self.get_parameter('serial_number').value
        ns = self.get_parameter('namespace').value
        color_w = self.get_parameter('color_width').value
        color_h = self.get_parameter('color_height').value
        depth_w = self.get_parameter('depth_width').value
        depth_h = self.get_parameter('depth_height').value
        fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value

        # 创建发布者
        self.color_pub = self.create_publisher(Image, f'{ns}/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, f'{ns}/depth/image_raw', 10)
        self.color_info_pub = self.create_publisher(CameraInfo, f'{ns}/color/camera_info', 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, f'{ns}/depth/camera_info', 10)

        # 启动相机
        if not self._start_camera(serial, color_w, color_h, depth_w, depth_h, fps):
            raise RuntimeError('Failed to start RealSense camera')

        # 定时器：按 fps 发布
        period = 1.0 / fps if fps > 0 else 0.033
        self.timer = self.create_timer(period, self._timer_callback)

        self.get_logger().info(
            f'RealSense D405 [{ns}] started: serial={serial}, '
            f'{color_w}x{color_h} @ {fps}fps'
        )

    def _start_camera(self, serial, color_w, color_h, depth_w, depth_h, fps):
        """启动 RealSense pipeline"""
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()

            if serial:
                config.enable_device(serial)

            config.enable_stream(rs.stream.color, color_w, color_h, rs.format.bgr8, fps)
            config.enable_stream(rs.stream.depth, depth_w, depth_h, rs.format.z16, fps)

            self.profile = self.pipeline.start(config)

            # 获取深度比例
            depth_sensor = self.profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()

            self.align = rs.align(rs.stream.color)

            # 预热
            for _ in range(30):
                self.pipeline.wait_for_frames()

            return True

        except Exception as e:
            self.get_logger().error(f'相机启动失败: {e}')
            return False

    def _get_camera_info(self, intrinsics):
        """从 rs.intrinsics 生成 CameraInfo"""
        info = CameraInfo()
        info.header.frame_id = self.frame_id
        info.width = intrinsics.width
        info.height = intrinsics.height
        info.k = [
            intrinsics.fx, 0.0, intrinsics.ppx,
            0.0, intrinsics.fy, intrinsics.ppy,
            0.0, 0.0, 1.0,
        ]
        info.d = list(intrinsics.coeffs)
        info.distortion_model = 'brown_conrady'
        info.p = [
            intrinsics.fx, 0.0, intrinsics.ppx, 0.0,
            0.0, intrinsics.fy, intrinsics.ppy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return info

    def _timer_callback(self):
        """定时采集并发布"""
        try:
            frames = self.pipeline.wait_for_frames(1000)
            aligned = self.align.process(frames)

            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()

            if not color_frame or not depth_frame:
                return

            now = self.get_clock().now().to_msg()

            # 发布彩色图
            color_img = np.asanyarray(color_frame.get_data())
            color_msg = self._numpy_to_image(color_img, 'bgr8', now)
            self.color_pub.publish(color_msg)

            # 发布彩色相机信息
            color_intrinsics = color_frame.get_profile().as_video_stream_profile().get_intrinsics()
            color_info = self._get_camera_info(color_intrinsics)
            color_info.header.stamp = now
            self.color_info_pub.publish(color_info)

            # 发布深度图
            depth_img = np.asanyarray(depth_frame.get_data())
            depth_msg = self._numpy_to_image(depth_img, '16UC1', now)
            self.depth_pub.publish(depth_msg)

            # 发布深度相机信息
            depth_intrinsics = depth_frame.get_profile().as_video_stream_profile().get_intrinsics()
            depth_info = self._get_camera_info(depth_intrinsics)
            depth_info.header.stamp = now
            self.depth_info_pub.publish(depth_info)

        except Exception as e:
            self.get_logger().warn(f'采集异常: {e}')

    def _numpy_to_image(self, img, encoding, stamp):
        """numpy -> sensor_msgs/Image"""
        if self.bridge:
            msg = self.bridge.cv2_to_imgmsg(img, encoding)
        else:
            msg = Image()
            msg.height = img.shape[0]
            msg.width = img.shape[1]
            if encoding == 'bgr8':
                msg.encoding = 'bgr8'
                msg.step = img.shape[1] * 3
            elif encoding == '16UC1':
                msg.encoding = '16UC1'
                msg.step = img.shape[1] * 2
            msg.data = img.tobytes()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        return msg

    def destroy_node(self):
        """清理资源"""
        if self.pipeline:
            try:
                self.pipeline.stop()
            except Exception:
                pass
        self.get_logger().info('RealSense node destroyed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
