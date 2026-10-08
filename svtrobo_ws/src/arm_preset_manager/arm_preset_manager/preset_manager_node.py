#!/usr/bin/env python3
"""
机械臂预设姿态管理节点
通过手柄 D-pad（方向键）切换预设姿态，与底盘控制完全解耦
订阅 /f710/joy 原始手柄数据，仅读取 axes[6](左右) 和 axes[7](上下)
"""

import yaml
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray, String


# F710 D-pad 轴索引
DPAD_H_AXIS = 6   # 左右: -1=左, 1=右
DPAD_V_AXIS = 7   # 上下: -1=上, 1=下


class ArmPresetManager(Node):
    def __init__(self):
        super().__init__('arm_preset_manager')

        # 参数
        self.declare_parameter('config_file', '')
        self.declare_parameter('joy_topic', '/f710/joy')
        self.declare_parameter('arm_cmd_topic', '/forward_position_controller/commands')
        self.declare_parameter('gripper_topic', '/gripper_controller/commands')
        self.declare_parameter('gripper_open', 0.044)
        self.declare_parameter('gripper_closed', 0.0)

        config_file = self.get_parameter('config_file').value
        self.arm_cmd_topic = self.get_parameter('arm_cmd_topic').value
        self.gripper_topic = self.get_parameter('gripper_topic').value
        self.gripper_open = self.get_parameter('gripper_open').value
        self.gripper_closed = self.get_parameter('gripper_closed').value
        joy_topic = self.get_parameter('joy_topic').value

        # 加载预设配置
        if not config_file or not os.path.exists(config_file):
            self.get_logger().fatal(f'配置文件不存在: {config_file}')
            raise FileNotFoundError(config_file)

        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)

        self.presets = self.config.get('presets', {})
        # D-pad 方向键映射
        self.dpad_map = self.config.get('dpad_map', {})

        self.preset_names = list(self.presets.keys())
        self.current_idx = 0
        self.current_joints = self.presets[self.preset_names[0]]['joints']
        self.gripper_is_open = False
        self.stopped = False

        # D-pad 边沿检测
        self.prev_dpad_h = 0.0
        self.prev_dpad_v = 0.0
        self.dpad_deadzone = 0.5

        # 发布器
        self.arm_pub = self.create_publisher(Float64MultiArray, self.arm_cmd_topic, 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, self.gripper_topic, 10)

        # 订阅手柄原始数据（只读 D-pad，不碰底盘按键）
        self.joy_sub = self.create_subscription(Joy, joy_topic, self.joy_callback, 10)

        # 订阅文本指令（备用）
        self.cmd_sub = self.create_subscription(
            String, '/arm/preset_cmd', self.cmd_callback, 10)

        self.get_logger().info(f'加载 {len(self.preset_names)} 个预设: {self.preset_names}')
        self.get_logger().info(f'D-pad 映射: {self.dpad_map}')
        self.get_logger().info(f'监听手柄: {joy_topic} (D-pad axes[{DPAD_H_AXIS}] / axes[{DPAD_V_AXIS}])')
        self.get_logger().info(f'指令话题: /arm/preset_cmd')

        # 延迟发布 home（等控制器就绪）
        self._home_timer = self.create_timer(3.0, self.publish_home)

    def publish_home(self):
        """启动后自动回 home"""
        self.publish_preset('home')
        self.destroy_timer(self._home_timer)
        self.get_logger().info('已回 home 位')

    def joy_callback(self, msg: Joy):
        """手柄回调，检测 D-pad 边沿"""
        axes = list(msg.axes)
        if len(axes) <= max(DPAD_H_AXIS, DPAD_V_AXIS):
            return

        h = axes[DPAD_H_AXIS]
        v = axes[DPAD_V_AXIS]

        # 检测水平边沿（左/右）
        if abs(h) > self.dpad_deadzone and abs(self.prev_dpad_h) <= self.dpad_deadzone:
            direction = 'right' if h > 0 else 'left'
            self.on_dpad(direction)

        # 检测垂直边沿（上/下）
        if abs(v) > self.dpad_deadzone and abs(self.prev_dpad_v) <= self.dpad_deadzone:
            direction = 'down' if v > 0 else 'up'
            self.on_dpad(direction)

        self.prev_dpad_h = h
        self.prev_dpad_v = v

    def cmd_callback(self, msg: String):
        """文本指令回调"""
        cmd = msg.data.strip()
        if cmd == 'gripper_open':
            self.toggle_gripper(open_gripper=True)
        elif cmd == 'gripper_close':
            self.toggle_gripper(open_gripper=False)
        elif cmd == 'emergency':
            self.emergency_stop()
        elif cmd == 'emergency_reset':
            self.stopped = False
            self.get_logger().info('急停已解除')
        else:
            self.publish_preset(cmd)

    def on_dpad(self, direction: str):
        """处理 D-pad 方向"""
        action = self.dpad_map.get(direction)
        if not action:
            return

        if action == 'gripper_toggle':
            self.toggle_gripper()
        elif action == 'emergency_stop':
            self.emergency_stop()
        elif action in self.presets:
            self.publish_preset(action)
            # 同步索引
            if action in self.preset_names:
                self.current_idx = self.preset_names.index(action)
        else:
            self.get_logger().warn(f'未知 D-pad 动作: {action}')

    def publish_preset(self, name: str):
        """发布预设姿态"""
        if self.stopped:
            self.get_logger().warn('急停中，发 /arm/preset_cmd emergency_reset 解除')
            return

        preset = self.presets.get(name)
        if not preset:
            self.get_logger().warn(f'未知预设: {name}')
            return

        joints = preset['joints']
        self.current_joints = joints

        # 发布关节位置
        msg = Float64MultiArray()
        msg.data = list(joints)
        self.arm_pub.publish(msg)

        # 发布夹爪
        gripper_val = preset.get('gripper', self.gripper_closed)
        self.gripper_is_open = (gripper_val > 0.01)
        gmsg = Float64MultiArray()
        gmsg.data = [gripper_val]
        self.gripper_pub.publish(gmsg)

        self.get_logger().info(f'预设 {name}: joints={joints}, gripper={gripper_val:.3f}')

    def toggle_gripper(self, open_gripper: bool = None):
        """切换夹爪"""
        if open_gripper is None:
            open_gripper = not self.gripper_is_open
        self.gripper_is_open = open_gripper
        val = self.gripper_open if open_gripper else self.gripper_closed
        msg = Float64MultiArray()
        msg.data = [val]
        self.gripper_pub.publish(msg)
        self.get_logger().info(f'夹爪: {"张开" if open_gripper else "闭合"} ({val:.3f})')

    def emergency_stop(self):
        """急停"""
        self.stopped = True
        self.get_logger().warn('急停! 发 /arm/preset_cmd emergency_reset 解除')


def main():
    rclpy.init()
    node = ArmPresetManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
