#!/usr/bin/env python3
"""
SVTROBO 底盘 Python 控制与状态获取 API

提供完整的底盘控制、状态读取、深度学习集成接口。

使用示例:
    # 基础控制
    with SVTROBOController() as robot:
        robot.move_forward(0.5)
        time.sleep(2)
        robot.stop()

    # 深度学习推理
    with SVTROBOController() as robot:
        obs = robot.get_observation()
        action = model.predict(obs)
        robot.publish_action(action)

    # 数据采集
    robot = SVTROBOController()
    robot.start_state_listener()
    state = robot.wait_for_state(timeout=1.0)
    robot.shutdown()
"""

import threading
import time
from typing import Dict, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray


class SVTROBOController:
    """SVTROBO 底盘控制器 — 提供控制和状态获取的完整 API。

    支持两种使用方式:
    1. 上下文管理器 (with 语句): 自动初始化/清理，退出时自动停止
    2. 手动管理: 调用 start/shutdown

    线程安全: 状态回调在 ROS 线程中执行，读取在用户线程中，内部使用锁保护。
    """

    # Topic 名称
    CMD_TOPIC = "/svtrobot_cmd"
    STATE_TOPIC = "/chassis/joint_states"
    CMD_FEEDBACK_TOPIC = "/chassis/cmd_feedback"
    LIFT_TOPIC = "/lift_control_cmd"

    # 关节名称映射 (与 C++ 端一致)
    STEER_JOINTS = ["fl_steer", "fr_steer", "rl_steer", "rr_steer"]
    WHEEL_JOINTS = ["fl_wheel", "fr_wheel", "rl_wheel", "rr_wheel"]

    def __init__(
        self,
        node_name: str = "svtrobo_controller",
        auto_stop_timeout: float = 0.0,
    ):
        """初始化控制器。

        Args:
            node_name: ROS2 节点名。
            auto_stop_timeout: 状态反馈中断超过该秒数后自动发送零速。
                               0 表示禁用。
        """
        self._auto_stop_timeout = auto_stop_timeout
        self._lock = threading.Lock()

        # 状态缓存
        self._joint_state: Optional[JointState] = None
        self._cmd_feedback: Optional[Twist] = None
        self._state_event = threading.Event()
        self._last_state_time: float = 0.0

        # ROS2 初始化
        if not rclpy.ok():
            rclpy.init()
        self._node = Node(node_name)

        # QoS: 使用 reliable + keep_last 保证数据可靠性
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publisher
        self._cmd_pub = self._node.create_publisher(Twist, self.CMD_TOPIC, 10)
        self._lift_pub = self._node.create_publisher(
            Int32MultiArray, self.LIFT_TOPIC, 10
        )

        # Subscriber
        self._state_sub = self._node.create_subscription(
            JointState, self.STATE_TOPIC, self._state_callback, qos
        )
        self._cmd_fb_sub = self._node.create_subscription(
            Twist, self.CMD_FEEDBACK_TOPIC, self._cmd_feedback_callback, qos
        )

        # ROS spin 线程
        self._spin_thread: Optional[threading.Thread] = None
        self._running = False

    # ──────────────────── 生命周期 ────────────────────

    def start(self) -> "SVTROBOController":
        """启动 ROS2 spin 线程，开始接收状态数据。"""
        if self._running:
            return self
        self._running = True
        self._spin_thread = threading.Thread(
            target=self._spin, daemon=True, name="svtrobo_spin"
        )
        self._spin_thread.start()
        return self

    def shutdown(self) -> None:
        """停止控制器，关闭 ROS2 节点。"""
        if not self._running:
            return
        self._running = False
        self.stop()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=3.0)
            self._spin_thread = None
        self._node.destroy_node()

    def _spin(self) -> None:
        """后台 spin 循环。"""
        while self._running and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.05)

    def __enter__(self) -> "SVTROBOController":
        return self.start()

    def __exit__(self, *args) -> None:
        self.shutdown()

    # ──────────────────── 控制 API ────────────────────

    def move(self, vx: float, vy: float, wz: float) -> None:
        """发送全向移动指令。

        Args:
            vx: 前进速度 (m/s)，正值前进，负值后退。
            vy: 横移速度 (m/s)，正值左移，负值右移。
            wz: 旋转角速度 (rad/s)，正值逆时针。
        """
        self._check_auto_stop()
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        msg.angular.z = float(wz)
        self._cmd_pub.publish(msg)

    def move_forward(self, speed: float = 0.1) -> None:
        """前进。"""
        self.move(vx=speed, vy=0.0, wz=0.0)

    def move_backward(self, speed: float = 0.1) -> None:
        """后退。"""
        self.move(vx=-speed, vy=0.0, wz=0.0)

    def move_left(self, speed: float = 0.1) -> None:
        """左移。"""
        self.move(vx=0.0, vy=speed, wz=0.0)

    def move_right(self, speed: float = 0.1) -> None:
        """右移。"""
        self.move(vx=0.0, vy=-speed, wz=0.0)

    def rotate(self, speed: float = 0.2) -> None:
        """原地旋转。

        Args:
            speed: 角速度 (rad/s)，正值逆时针，负值顺时针。
        """
        self.move(vx=0.0, vy=0.0, wz=speed)

    def stop(self) -> None:
        """停止底盘运动。"""
        msg = Twist()
        self._cmd_pub.publish(msg)

    def control_lift(self, direction: int, speed: int) -> None:
        """控制升降机构。

        Args:
            direction: 1=正转, -1=反转, 0=停止。
            speed: RPM (1~6000)。
        """
        msg = Int32MultiArray()
        msg.data = [int(direction), int(speed)]
        self._lift_pub.publish(msg)

    def stop_lift(self) -> None:
        """停止升降机构。"""
        self.control_lift(0, 0)

    # ──────────────────── 状态获取 API ────────────────────

    def get_state(self) -> Optional[Dict]:
        """获取完整的底盘状态（非阻塞）。

        Returns:
            包含所有状态数据的字典，如果尚未收到数据返回 None。

            字段:
                stamp            : 时间戳 (ROS Time)
                steer_angles     : [FL, FR, RL, RR] 舵向实际角度 (rad)
                steer_velocities : [FL, FR, RL, RR] 舵向速度 (rad/s)
                steer_torques    : [FL, FR, RL, RR] 舵向力矩 (Nm)
                wheel_speeds     : [FL, FR, RL, RR] 轮子目标转速 (RPM)
                vx_set           : 当前指令 vx (m/s)
                vy_set           : 当前指令 vy (m/s)
                wz_set           : 当前指令 wz (rad/s)
        """
        with self._lock:
            if self._joint_state is None:
                return None
            js = self._joint_state
            cmd = self._cmd_feedback

        result = {
            "stamp": js.header.stamp,
            "steer_angles": list(js.position[:4]),
            "steer_velocities": list(js.velocity[:4]),
            "steer_torques": list(js.effort[:4]),
            "wheel_speeds": list(js.velocity[4:8]),
        }
        if cmd is not None:
            result["vx_set"] = cmd.linear.x
            result["vy_set"] = cmd.linear.y
            result["wz_set"] = cmd.angular.z
        return result

    def wait_for_state(self, timeout: float = 5.0) -> Optional[Dict]:
        """阻塞等待直到收到第一条状态数据或超时。

        Args:
            timeout: 超时秒数。

        Returns:
            状态字典，超时返回 None。
        """
        self._state_event.clear()
        if self._state_event.wait(timeout=timeout):
            return self.get_state()
        return None

    @property
    def is_connected(self) -> bool:
        """是否已收到底盘状态数据。"""
        with self._lock:
            return self._joint_state is not None

    @property
    def steer_angles(self) -> Optional[List[float]]:
        """4 个舵向电机的实际角度 [FL, FR, RL, RR] (rad)。"""
        with self._lock:
            if self._joint_state is None:
                return None
            return list(self._joint_state.position[:4])

    @property
    def steer_velocities(self) -> Optional[List[float]]:
        """4 个舵向电机的速度 [FL, FR, RL, RR] (rad/s)。"""
        with self._lock:
            if self._joint_state is None:
                return None
            return list(self._joint_state.velocity[:4])

    @property
    def steer_torques(self) -> Optional[List[float]]:
        """4 个舵向电机的力矩 [FL, FR, RL, RR] (Nm)。"""
        with self._lock:
            if self._joint_state is None:
                return None
            return list(self._joint_state.effort[:4])

    @property
    def wheel_speeds(self) -> Optional[List[float]]:
        """4 个轮子的目标转速 [FL, FR, RL, RR] (RPM)。"""
        with self._lock:
            if self._joint_state is None:
                return None
            return list(self._joint_state.velocity[4:8])

    @property
    def last_cmd(self) -> Optional[Dict[str, float]]:
        """最近收到的指令反馈 {vx, vy, wz}。"""
        with self._lock:
            if self._cmd_feedback is None:
                return None
            return {
                "vx": self._cmd_feedback.linear.x,
                "vy": self._cmd_feedback.linear.y,
                "wz": self._cmd_feedback.angular.z,
            }

    # ──────────────────── 深度学习接口 ────────────────────

    def get_observation(self) -> Optional[np.ndarray]:
        """获取模型观测向量（numpy 数组）。

        观测向量布局 (16 维):
            [0:4]  舵向角度 (rad)
            [4:8]  舵向速度 (rad/s)
            [8:12] 舵向力矩 (Nm)
            [12:16] 轮子目标转速 (RPM)

        Returns:
            shape (16,) 的 numpy float32 数组，无数据时返回 None。
        """
        state = self.get_state()
        if state is None:
            return None
        obs = np.array(
            state["steer_angles"]
            + state["steer_velocities"]
            + state["steer_torques"]
            + state["wheel_speeds"],
            dtype=np.float32,
        )
        return obs

    def publish_action(self, action: np.ndarray) -> None:
        """将模型输出发布为底盘控制指令。

        Args:
            action: 长度 3 的数组 [vx, vy, wz]。
        """
        action = np.asarray(action).flatten()
        if len(action) < 3:
            raise ValueError(f"action 需要至少 3 个元素, 收到 {len(action)}")
        self.move(float(action[0]), float(action[1]), float(action[2]))

    # ──────────────────── 内部回调 ────────────────────

    def _state_callback(self, msg: JointState) -> None:
        """接收底盘状态回调。"""
        with self._lock:
            self._joint_state = msg
            self._last_state_time = time.monotonic()
        self._state_event.set()

    def _cmd_feedback_callback(self, msg: Twist) -> None:
        """接收指令反馈回调。"""
        with self._lock:
            self._cmd_feedback = msg

    def _check_auto_stop(self) -> None:
        """检查状态反馈是否超时，超时则自动停止。"""
        if self._auto_stop_timeout <= 0:
            return
        with self._lock:
            if self._last_state_time == 0.0:
                return
            elapsed = time.monotonic() - self._last_state_time
        if elapsed > self._auto_stop_timeout:
            self.stop()


# ══════════════════════════════════════════════════════════
#  使用示例
# ══════════════════════════════════════════════════════════

if __name__ == "__main__":
    print("=" * 50)
    print("  SVTROBO 底盘控制器 — 使用示例")
    print("=" * 50)

    with SVTROBOController(auto_stop_timeout=1.0) as robot:
        # 等待底盘连接
        print("\n等待底盘状态...")
        state = robot.wait_for_state(timeout=10.0)
        if state is None:
            print("超时：未收到底盘状态，请确认底盘节点已启动。")
            exit(1)

        print("底盘已连接！")
        print(f"  舵向角度: {state['steer_angles']}")
        print(f"  舵向速度: {state['steer_velocities']}")
        print(f"  舵向力矩: {state['steer_torques']}")
        print(f"  轮速:     {state['wheel_speeds']}")

        # --- 示例 1: 基础运动控制 ---
        print("\n--- 示例 1: 前进 0.3 m/s，持续 2 秒 ---")
        robot.move_forward(0.3)
        time.sleep(2)
        state = robot.get_state()
        print(f"  运动中状态: {state['steer_angles']}")
        robot.stop()
        print("  已停止")

        time.sleep(1)

        # --- 示例 2: 使用 get_observation ---
        print("\n--- 示例 2: 获取观测向量 ---")
        obs = robot.get_observation()
        if obs is not None:
            print(f"  观测向量 shape: {obs.shape}")
            print(f"  观测向量: {obs}")

        # --- 示例 3: 属性访问 ---
        print("\n--- 示例 3: 属性访问 ---")
        print(f"  steer_angles:     {robot.steer_angles}")
        print(f"  steer_velocities: {robot.steer_velocities}")
        print(f"  steer_torques:    {robot.steer_torques}")
        print(f"  wheel_speeds:     {robot.wheel_speeds}")
        print(f"  last_cmd:         {robot.last_cmd}")
        print(f"  is_connected:     {robot.is_connected}")

        # --- 示例 4: publish_action (深度学习推理) ---
        print("\n--- 示例 4: publish_action 模拟 ---")
        dummy_action = np.array([0.1, 0.0, 0.2])
        print(f"  发送 action: {dummy_action}")
        robot.publish_action(dummy_action)
        time.sleep(1)
        robot.stop()

    print("\n控制器已关闭。")
