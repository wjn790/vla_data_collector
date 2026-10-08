#!/usr/bin/env python3
import csv
import math
import os
import threading
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray, String

DPAD_H_AXIS = 0
DPAD_V_AXIS = 1
BUTTON_A = 1

LEFT_ORDER = [
    'openarm_left_joint1',
    'openarm_left_joint2',
    'openarm_left_joint3',
    'openarm_left_joint4',
    'openarm_left_joint5',
    'openarm_left_joint6',
    'openarm_left_joint7',
]
RIGHT_ORDER = [
    'openarm_right_joint1',
    'openarm_right_joint2',
    'openarm_right_joint3',
    'openarm_right_joint4',
    'openarm_right_joint5',
    'openarm_right_joint6',
    'openarm_right_joint7',
]


def smoothstep(t: float) -> float:
    """Hermite smoothstep: 3t^2 - 2t^3, maps [0,1] -> [0,1] with zero derivative at endpoints."""
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def lerp_joints(a: List[float], b: List[float], alpha: float) -> List[float]:
    return [x + (y - x) * alpha for x, y in zip(a, b)]


class ArmMotionPlayer(Node):
    def __init__(self):
        super().__init__('arm_motion_player')

        # --- 参数声明 ---
        self.declare_parameter('joy_topic', '/f710/joy')
        self.declare_parameter('command_topic', '/arm/motion_cmd')
        self.declare_parameter('status_topic', '/arm/motion_status')
        self.declare_parameter('left_arm_topic', '/left_forward_position_controller/commands')
        self.declare_parameter('right_arm_topic', '/right_forward_position_controller/commands')
        self.declare_parameter('data_dir', '/home/svt/svtrobo_ws/src/arm_motion_data')
        self.declare_parameter('dpad_deadzone', 0.5)
        self.declare_parameter('playback_speed', 1.0)
        self.declare_parameter('require_a_to_arm', True)
        self.declare_parameter('button_a', BUTTON_A)
        self.declare_parameter('transition_duration', 0.8)
        self.declare_parameter('transition_steps', 30)
        self.declare_parameter('home_joints', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('stop_double_press_window', 1.0)
        self.declare_parameter('loop', True)
        self.declare_parameter('dpad_map.up', 'walk')
        self.declare_parameter('dpad_map.down', 'grasp')
        self.declare_parameter('dpad_map.left', 'turn')
        self.declare_parameter('dpad_map.right', 'stop')
        self.declare_parameter('motion_files.walk', 'walk.csv')
        self.declare_parameter('motion_files.grasp', 'grasp.csv')
        self.declare_parameter('motion_files.turn', 'turn.csv')

        # --- 参数读取 ---
        joy_topic = self.get_parameter('joy_topic').value
        self.command_topic = self.get_parameter('command_topic').value
        status_topic = self.get_parameter('status_topic').value
        self.left_arm_topic = self.get_parameter('left_arm_topic').value
        self.right_arm_topic = self.get_parameter('right_arm_topic').value
        self.data_dir = self.get_parameter('data_dir').value
        self.dpad_deadzone = float(self.get_parameter('dpad_deadzone').value)
        self.playback_speed = max(0.05, float(self.get_parameter('playback_speed').value))
        self.require_a_to_arm = bool(self.get_parameter('require_a_to_arm').value)
        self.button_a = int(self.get_parameter('button_a').value)
        self.transition_duration = float(self.get_parameter('transition_duration').value)
        self.transition_steps = int(self.get_parameter('transition_steps').value)
        self.home_joints = [float(v) for v in self.get_parameter('home_joints').value]
        self.stop_double_press_window = float(self.get_parameter('stop_double_press_window').value)
        self.loop_enabled = bool(self.get_parameter('loop').value)

        self.dpad_map = {
            'up': self.get_parameter('dpad_map.up').value,
            'down': self.get_parameter('dpad_map.down').value,
            'left': self.get_parameter('dpad_map.left').value,
            'right': self.get_parameter('dpad_map.right').value,
        }
        motion_files = {
            'walk': self.get_parameter('motion_files.walk').value,
            'grasp': self.get_parameter('motion_files.grasp').value,
            'turn': self.get_parameter('motion_files.turn').value,
        }

        # --- ROS 接口 ---
        self.left_pub = self.create_publisher(Float64MultiArray, self.left_arm_topic, 10)
        self.right_pub = self.create_publisher(Float64MultiArray, self.right_arm_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.cmd_sub = self.create_subscription(String, self.command_topic, self.cmd_callback, 10)
        self.joy_sub = self.create_subscription(Joy, joy_topic, self.joy_callback, 10)

        # --- 状态 ---
        self.prev_dpad_h = 0.0
        self.prev_dpad_v = 0.0
        self.prev_a = 0
        self.motion_armed = not self.require_a_to_arm
        self._play_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.current_motion: Optional[str] = None
        self.last_left: Optional[List[float]] = None
        self.last_right: Optional[List[float]] = None
        self.frames: Dict[str, List[Tuple[int, List[float], List[float]]]] = {}

        # 双击停止状态机
        # 'idle' = 无动作/已回 home → 下一次 stop 是"第一下"（暂停）
        # 'stopped' = 已暂停 → 下一次 stop 是"第二下"（回 home）
        self._stop_state = 'idle'
        self._last_stop_time: float = 0.0

        # 加载动作 CSV
        for name, rel in motion_files.items():
            path = os.path.join(self.data_dir, rel)
            self.frames[name] = self._load_motion_csv(path)
            self.get_logger().info(f"加载动作 {name}: {len(self.frames[name])} 帧 <- {path}")

        self.publish_status('armed' if self.motion_armed else 'locked')
        self.get_logger().info(f"D-pad 动作映射: {self.dpad_map}")
        self.get_logger().info(f"动作指令话题: {self.command_topic}")
        self.get_logger().info(f"播放速度倍率: {self.playback_speed:.2f}x")
        self.get_logger().info(f"过渡插值: {self.transition_duration:.2f}s / {self.transition_steps} 步 (smoothstep)")
        self.get_logger().info(f"双击停止窗口: {self.stop_double_press_window:.1f}s")
        self.get_logger().info(f"循环播放: {'开启' if self.loop_enabled else '关闭'}")
        self.get_logger().info(f"Home 姿态: {self.home_joints}")
        if self.require_a_to_arm:
            self.get_logger().info('动作播放默认锁定，需先按 A 解锁')

    # ==================== CSV 加载 ====================

    def _load_motion_csv(self, path: str) -> List[Tuple[int, List[float], List[float]]]:
        if not os.path.exists(path):
            raise FileNotFoundError(path)
        frames: List[Tuple[int, List[float], List[float]]] = []
        with open(path, 'r', encoding='utf-8', errors='replace') as f:
            reader = csv.DictReader(f)
            missing = [c for c in LEFT_ORDER + RIGHT_ORDER if c not in reader.fieldnames]
            if missing:
                raise ValueError(f"{path} 缺少列: {missing}")
            for row in reader:
                ts = int(row.get('ros_timestamp_ns') or row.get('system_timestamp_ns') or 0)
                left = [float(row[name]) for name in LEFT_ORDER]
                right = [float(row[name]) for name in RIGHT_ORDER]
                frames.append((ts, left, right))
        if not frames:
            raise ValueError(f"{path} 无有效帧")
        return frames

    # ==================== 状态发布 ====================

    def publish_status(self, text: str):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(f"发布状态: {text}")

    # ==================== 手柄 & 指令回调 ====================

    def joy_callback(self, msg: Joy):
        buttons = list(msg.buttons)
        a_now = buttons[self.button_a] if 0 <= self.button_a < len(buttons) else 0
        if a_now and not self.prev_a and not self.motion_armed:
            self.motion_armed = True
            self.get_logger().info('A 按下：动作播放已解锁')
            self.publish_status('armed')
        self.prev_a = a_now

        axes = list(msg.axes)
        if len(axes) <= max(DPAD_H_AXIS, DPAD_V_AXIS):
            return
        h = axes[DPAD_H_AXIS]
        v = axes[DPAD_V_AXIS]
        if abs(h) > self.dpad_deadzone and abs(self.prev_dpad_h) <= self.dpad_deadzone:
            self._handle_direction('right' if h > 0 else 'left')
        if abs(v) > self.dpad_deadzone and abs(self.prev_dpad_v) <= self.dpad_deadzone:
            self._handle_direction('down' if v > 0 else 'up')
        self.prev_dpad_h = h
        self.prev_dpad_v = v

    def _handle_direction(self, direction: str):
        cmd = self.dpad_map.get(direction)
        if not cmd:
            return
        if cmd != 'stop' and not self.motion_armed:
            self.get_logger().warn('动作未解锁，按 A 后再用 D-pad 触发')
            self.publish_status('locked')
            return
        self.handle_motion_command(cmd)

    def cmd_callback(self, msg: String):
        cmd = msg.data.strip()
        if cmd == 'arm':
            self.motion_armed = True
            self.get_logger().info('收到 arm 命令：动作播放已解锁')
            self.publish_status('armed')
            return
        self.handle_motion_command(cmd)

    # ==================== 命令分发 ====================

    def handle_motion_command(self, cmd: str):
        if not cmd:
            return
        if cmd == 'stop':
            self._handle_stop()
            return
        if cmd not in self.frames:
            self.get_logger().warn(f"未知动作: {cmd}")
            self.publish_status(f"error:unknown:{cmd}")
            return
        if not self.motion_armed:
            self.get_logger().warn('动作未解锁，忽略播放请求')
            self.publish_status('locked')
            return
        self._stop_state = 'idle'  # 开始新动作时重置停止状态
        self.start_motion(cmd)

    def _handle_stop(self):
        """双击停止逻辑：
        第一下 → 停止动作，保持当前姿态
        第二下（间隔 > stop_double_press_window 秒）→ smoothstep 回 home
        """
        now = time.monotonic()

        if not self.motion_armed:
            self.get_logger().warn('动作未解锁，忽略 stop')
            self.publish_status('locked')
            return

        if self._stop_state == 'stopped':
            # 已经在 stopped 状态，检查是否过了防抖窗口
            elapsed = now - self._last_stop_time
            if elapsed < self.stop_double_press_window:
                self.get_logger().info(
                    f"第二下停止过快 ({elapsed:.2f}s < {self.stop_double_press_window:.1f}s)，忽略"
                )
                return
            # 第二下：回 home
            self.get_logger().info("第二下停止 → 回 home 姿态")
            self._stop_state = 'idle'
            self._go_home()
            return

        # 第一下：停止动作，保持当前姿态
        self._last_stop_time = now
        self._stop_state = 'stopped'
        self.stop_motion('第一下停止，保持当前姿态')
        self.get_logger().info(
            f"第一下停止已执行，{self.stop_double_press_window:.1f}s 后再按回 home"
        )

    # ==================== 回 Home ====================

    def _go_home(self):
        """smoothstep 从当前姿态过渡到 home"""
        start_left = self.last_left if self.last_left else self.home_joints[:]
        start_right = self.last_right if self.last_right else self.home_joints[:]
        target_left = self.home_joints[:]
        target_right = self.home_joints[:]

        self.stop_motion('go home', log_if_idle=False)

        self._stop_event = threading.Event()
        with self._play_lock:
            self.current_motion = '_go_home'
            self._thread = threading.Thread(
                target=self._transition_and_hold,
                args=(start_left, start_right, target_left, target_right, 'home'),
                daemon=True,
                name='arm-motion-go-home',
            )
            self._thread.start()
        self.publish_status('going_home')
        self.get_logger().info("开始回 home ...")

    # ==================== 动作播放 ====================

    def start_motion(self, motion_name: str):
        self.stop_motion(f"switch to {motion_name}", log_if_idle=False)
        with self._play_lock:
            self._stop_event = threading.Event()
            self.current_motion = motion_name
            self._thread = threading.Thread(
                target=self._play_motion,
                args=(motion_name, self._stop_event),
                daemon=True,
                name=f"arm-motion-{motion_name}",
            )
            self._thread.start()
        self.get_logger().info(f"开始播放动作: {motion_name}")
        self.publish_status(f"playing:{motion_name}")

    def stop_motion(self, reason: str, log_if_idle: bool = True):
        thread = None
        stop_event = None
        with self._play_lock:
            thread = self._thread
            stop_event = self._stop_event if self._thread else None
            self._thread = None
            self.current_motion = None
        if stop_event is not None:
            stop_event.set()
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
            if self.last_left is not None and self.last_right is not None:
                self._publish_frame(self.last_left, self.last_right)
            self.get_logger().info(f"动作已停止并保持当前姿态: {reason}")
        elif log_if_idle:
            self.get_logger().info('当前无动作在播放')

    # ==================== 帧发布 ====================

    def _publish_frame(self, left: List[float], right: List[float]):
        self.last_left = list(left)
        self.last_right = list(right)
        lmsg = Float64MultiArray()
        lmsg.data = list(left)
        rmsg = Float64MultiArray()
        rmsg.data = list(right)
        self.left_pub.publish(lmsg)
        self.right_pub.publish(rmsg)

    # ==================== 过渡插值（通用） ====================

    def _run_transition(
        self,
        start_left: List[float],
        start_right: List[float],
        target_left: List[float],
        target_right: List[float],
        stop_event: threading.Event,
        label: str = 'transition',
    ):
        """smoothstep 过渡：从 start_* 过渡到 target_*"""
        steps = self.transition_steps
        step_dt = self.transition_duration / steps
        for i in range(1, steps + 1):
            if stop_event.is_set():
                return
            alpha = smoothstep(i / steps)
            blend_left = lerp_joints(start_left, target_left, alpha)
            blend_right = lerp_joints(start_right, target_right, alpha)
            self._publish_frame(blend_left, blend_right)
            time.sleep(step_dt)
        self.get_logger().info(f"过渡完成: {label}")

    # ==================== 过渡 + 保持（用于回 home） ====================

    def _transition_and_hold(
        self,
        start_left: List[float],
        start_right: List[float],
        target_left: List[float],
        target_right: List[float],
        label: str,
    ):
        """过渡到目标后停止（用于 go_home 场景）"""
        try:
            self._run_transition(
                start_left, start_right, target_left, target_right,
                self._stop_event, label,
            )
        finally:
            with self._play_lock:
                if self._thread is threading.current_thread():
                    self._thread = None
                    self.current_motion = None
            self.publish_status('armed' if self.motion_armed else 'locked')

    # ==================== 动作播放主循环（含过渡） ====================

    def _play_motion(self, motion_name: str, stop_event: threading.Event):
        frames = self.frames[motion_name]
        target_left = frames[0][1]
        target_right = frames[0][2]

        # --- 阶段1：过渡插值（从当前姿态到新动作首帧） ---
        # 第一次播放时 last_left/right 为 None，用 home_joints 作为起点以触发减速过渡
        start_left = self.last_left if self.last_left else self.home_joints[:]
        start_right = self.last_right if self.last_right else self.home_joints[:]

        # 判断是否需要过渡：当前姿态和新动作首帧差距较大时才做过渡
        max_diff = 0.0
        for a, b in zip(start_left + start_right, target_left + target_right):
            max_diff = max(max_diff, abs(a - b))

        if max_diff > 0.05:  # 超过 0.05 rad 才做过渡
            self.get_logger().info(
                f"过渡到 {motion_name} 首帧 (最大关节差: {max_diff:.3f} rad, "
                f"{self.transition_duration:.2f}s smoothstep)"
            )
            self._run_transition(
                start_left, start_right, target_left, target_right,
                stop_event, f"{motion_name} 首帧过渡",
            )
            if stop_event.is_set():
                return
        else:
            self.get_logger().info(
                f"跳过过渡（最大关节差 {max_diff:.3f} rad < 0.05），直接播放 {motion_name}"
            )

        # --- 阶段2：轨迹回放（支持循环） ---
        try:
            loop_count = 0
            while not stop_event.is_set():
                # 循环时，尾帧→首帧做过渡插值
                if loop_count > 0 and self.loop_enabled:
                    tail_left = self.last_left if self.last_left else frames[-1][1]
                    tail_right = self.last_right if self.last_right else frames[-1][2]
                    head_left = frames[0][1]
                    head_right = frames[0][2]
                    # 检查尾帧和首帧差距
                    max_diff = 0.0
                    for a, b in zip(tail_left + tail_right, head_left + head_right):
                        max_diff = max(max_diff, abs(a - b))
                    if max_diff > 0.05:
                        # 用首帧和第二帧的 dt 来估算过渡时长（取短的避免卡顿）
                        transition_steps = min(self.transition_steps, 15)
                        step_dt = self.transition_duration / transition_steps
                        for i in range(1, transition_steps + 1):
                            if stop_event.is_set():
                                return
                            alpha = smoothstep(i / transition_steps)
                            blend_left = lerp_joints(tail_left, head_left, alpha)
                            blend_right = lerp_joints(tail_right, head_right, alpha)
                            self._publish_frame(blend_left, blend_right)
                            time.sleep(step_dt)

                prev_ts = frames[0][0]
                for idx, (ts, left, right) in enumerate(frames):
                    if stop_event.is_set():
                        return
                    if idx > 0:
                        dt_ns = max(0, ts - prev_ts)
                        time.sleep((dt_ns / 1e9) / self.playback_speed)
                    self._publish_frame(left, right)
                    prev_ts = ts

                loop_count += 1
                if not self.loop_enabled:
                    break

            self.get_logger().info(f"动作播放完成: {motion_name} (循环 {loop_count} 次)")
        finally:
            with self._play_lock:
                if self._thread is threading.current_thread():
                    self._thread = None
                    self.current_motion = None
            self.publish_status('armed' if self.motion_armed else 'locked')


def main():
    rclpy.init()
    node = ArmMotionPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motion('shutdown', log_if_idle=False)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
