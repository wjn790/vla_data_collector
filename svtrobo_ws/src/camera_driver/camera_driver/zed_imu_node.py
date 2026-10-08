"""
ZED 2i IMU ROS2 节点

双模式: SDK 优先, USB HID 备用

发布 Topic:
  - ~/zed/imu/data       (sensor_msgs/Imu)        - 加速度计 + 陀螺仪
  - ~/zed/imu/mag        (sensor_msgs/MagneticField) - 磁力计
  - ~/zed/imu/temperature (sensor_msgs/Temperature)  - IMU 温度

用法:
  ros2 run camera_driver zed_imu_node --ros-args \
    -p frame_id:=zed_imu_link \
    -p force_hid:=false
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField, Temperature, FluidPressure
from std_msgs.msg import Header

from .zed_imu import ZEDIMU


class ZEDIMUNode(Node):
    """ZED 2i IMU ROS2 发布节点 (SDK 优先, USB HID 备用)"""

    def __init__(self):
        super().__init__('zed_imu_node')

        # 参数
        self.declare_parameter('frame_id', 'zed_imu_link')
        self.declare_parameter('publish_mag', True)
        self.declare_parameter('publish_temp', True)
        self.declare_parameter('publish_env', False)
        self.declare_parameter('force_hid', False)

        self.frame_id = self.get_parameter('frame_id').value
        publish_mag = self.get_parameter('publish_mag').value
        publish_temp = self.get_parameter('publish_temp').value
        publish_env = self.get_parameter('publish_env').value
        force_hid = self.get_parameter('force_hid').value

        # IMU 驱动 (自动选择 SDK/HID)
        self.imu = ZEDIMU(force_hid=force_hid)

        # 发布者
        self.imu_pub = self.create_publisher(Imu, 'zed/imu/data', 200)
        if publish_mag:
            self.mag_pub = self.create_publisher(MagneticField, 'zed/imu/mag', 50)
        else:
            self.mag_pub = None
        if publish_temp:
            self.temp_pub = self.create_publisher(Temperature, 'zed/imu/temperature', 10)
        else:
            self.temp_pub = None
        if publish_env:
            self.press_pub = self.create_publisher(FluidPressure, 'zed/imu/pressure', 10)
        else:
            self.press_pub = None

        # 启动 IMU
        try:
            self.imu.start()
        except RuntimeError as e:
            self.get_logger().error(str(e))
            raise

        # 定时发布 (100Hz 足够覆盖 IMU 数据率)
        self.timer = self.create_timer(0.01, self._timer_callback)

        self._last_ts = 0
        self._seq = 0

        self.get_logger().info(
            'ZED IMU node started (mode: {})'.format(self.imu.mode))

    def _timer_callback(self):
        data = self.imu.read()
        if data is None:
            return

        # 跳过重复时间戳
        if data['timestamp_ns'] == self._last_ts:
            return
        self._last_ts = data['timestamp_ns']

        if not data['valid']:
            return

        stamp = self.get_clock().now().to_msg()
        self._seq += 1

        # ---- Imu 消息 ----
        imu_msg = Imu()
        imu_msg.header = Header(
            stamp=stamp,
            frame_id=self.frame_id,
        )

        # 四元数姿态 (未知，不填)
        imu_msg.orientation_covariance[0] = -1.0

        # 角速度 (rad/s)
        gx, gy, gz = data['gyro_rad']
        imu_msg.angular_velocity.x = float(gx)
        imu_msg.angular_velocity.y = float(gy)
        imu_msg.angular_velocity.z = float(gz)
        imu_msg.angular_velocity_covariance = [
            0.0001, 0.0, 0.0,
            0.0, 0.0001, 0.0,
            0.0, 0.0, 0.0001,
        ]

        # 线加速度 (m/s^2)
        ax, ay, az = data['accel']
        imu_msg.linear_acceleration.x = float(ax)
        imu_msg.linear_acceleration.y = float(ay)
        imu_msg.linear_acceleration.z = float(az)
        imu_msg.linear_acceleration_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001,
        ]

        self.imu_pub.publish(imu_msg)

        # ---- 磁力计 ----
        if self.mag_pub and data['mag_valid'] >= 1:
            mag_msg = MagneticField()
            mag_msg.header = Header(stamp=stamp, frame_id=self.frame_id)
            mx, my, mz = data['mag']
            mag_msg.magnetic_field.x = float(mx)
            mag_msg.magnetic_field.y = float(my)
            mag_msg.magnetic_field.z = float(mz)
            mag_msg.magnetic_field_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01,
            ]
            self.mag_pub.publish(mag_msg)

        # ---- 温度 ----
        if self.temp_pub:
            temp_msg = Temperature()
            temp_msg.header = Header(stamp=stamp, frame_id=self.frame_id)
            temp_msg.temperature = float(data['imu_temp'])
            temp_msg.variance = 0.1
            self.temp_pub.publish(temp_msg)

        # ---- 气压 ----
        if self.press_pub and data.get('env_valid'):
            press_msg = FluidPressure()
            press_msg.header = Header(stamp=stamp, frame_id=self.frame_id)
            press_msg.fluid_pressure = float(data['pressure'])
            press_msg.variance = 0.01
            self.press_pub.publish(press_msg)

    def destroy_node(self):
        self.imu.stop()
        self.get_logger().info('ZED IMU node destroyed (mode was: {})'.format(self.imu.mode))
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZEDIMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
