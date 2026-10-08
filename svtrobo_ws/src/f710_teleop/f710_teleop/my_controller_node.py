import re
import struct
import threading
import time
import urllib.error
import urllib.request
from typing import List

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, Int32MultiArray, String


JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80


class LinuxJoystickReader(threading.Thread):
    """简单的 Linux /dev/input/jsX 手柄读取线程."""

    def __init__(self, device_path: str, axes_count: int = 8, buttons_count: int = 12):
        super().__init__(daemon=True)
        self.device_path = device_path
        self.axes: List[float] = [0.0] * axes_count
        self.buttons: List[int] = [0] * buttons_count
        self._stop_event = threading.Event()
        self._device_fd = None
        self._last_event_mono = 0.0

    def run(self) -> None:
        while not self._stop_event.is_set():
            if self._device_fd is None:
                try:
                    self._device_fd = open(self.device_path, "rb", buffering=0)
                except OSError:
                    # 设备暂时不可用，稍后重试
                    time.sleep(1.0)
                    continue

            try:
                data = self._device_fd.read(8)
                if not data or len(data) < 8:
                    # 读到 EOF 或者短包，重连
                    self._device_fd.close()
                    self._device_fd = None
                    time.sleep(0.5)
                    continue

                _, value, etype, number = struct.unpack("IhBB", data)
                self._last_event_mono = time.monotonic()

                if etype & JS_EVENT_INIT:
                    etype &= ~JS_EVENT_INIT

                if etype == JS_EVENT_AXIS and 0 <= number < len(self.axes):
                    # 将 int16 [-32768, 32767] 映射到 [-1.0, 1.0]
                    self.axes[number] = float(value) / 32767.0
                elif etype == JS_EVENT_BUTTON and 0 <= number < len(self.buttons):
                    self.buttons[number] = 1 if value else 0
            except OSError:
                if self._device_fd is not None:
                    try:
                        self._device_fd.close()
                    except OSError:
                        pass
                    self._device_fd = None
                time.sleep(0.5)

    def stop(self) -> None:
        self._stop_event.set()
        if self._device_fd is not None:
            try:
                self._device_fd.close()
            except OSError:
                pass
            self._device_fd = None

    def has_fresh_input(self, timeout_sec: float) -> bool:
        if timeout_sec <= 0.0:
            return True
        if self._last_event_mono <= 0.0:
            return False
        return (time.monotonic() - self._last_event_mono) <= timeout_sec

    def is_alive(self) -> bool:
        """设备 fd 是否处于打开状态（USB 接收器还活着、可读）。

        Linux joy 驱动在接收器拔出/掉电时会触发 read 返回 EOF 或 EIO，
        read 线程会把 _device_fd 置 None 并尝试重连；所以"fd 不是 None"
        等价于"设备当前能正常收数据"。这是判断手柄是否真正失联的可靠信号，
        不会像"事件时间"那样被摇杆静止误判。
        """
        return self._device_fd is not None


class F710TeleopNode(Node):
    """根据 docs/方案.md 与 手柄数据说明 实现的控制节点."""

    def __init__(self) -> None:
        super().__init__("my_controller_node")

        # 参数配置（默认为 /dev/input/js1，可被 YAML / 命令行覆盖）
        self.declare_parameter("device_path", "/dev/input/js1")
        self.declare_parameter("deadzone", 0.1)
        # true：死区外线性拉伸到 [-1,1]，避免刚过死区就接近满速
        self.declare_parameter("deadzone.remap", True)
        self.declare_parameter("publish_rate", 25.0)

        # 速度配置（底盘线速度 / 角速度基准）
        self.declare_parameter("velocity.max_linear", 1.0)
        self.declare_parameter("velocity.max_angular", 1.0)

        # 通过 LT / RT 调整的缩放系数：
        # speed_scale.max 作为“默认上限”，speed_scale.hard_max 作为“绝对上限”
        # 最终范围: speed_scale ∈ [min, hard_max]
        self.declare_parameter("speed_scale.min", 0.01)
        self.declare_parameter("speed_scale.max", 1.0)
        self.declare_parameter("speed_scale.hard_max", 1.5)
        self.declare_parameter("speed_scale.initial", 1.0)
        self.declare_parameter("speed_scale.step", 0.05)

        # 升降配置
        self.declare_parameter("lift.speed", 500)

        # 轴与按钮索引（可根据实际手柄映射调整）
        self.declare_parameter("axis.left_x", 0)   # 左摇杆 左右 (X)
        self.declare_parameter("axis.left_y", 1)   # 左摇杆 前后 (Y)
        # 对于当前手柄：Z / Rz 多数情况下对应右摇杆两个方向
        # 这里默认使用 2 作为“右摇杆左右”轴，如果不符可通过参数 axis.right_x 覆盖
        self.declare_parameter("axis.right_x", 2)  # 右摇杆 左右 (Z)
        # 左手柄左右：发送的 linear.y 是否取反（true=发送时取反）
        self.declare_parameter("invert.left_x", False)
        # 右手柄左右：发送的 angular.z 是否取反（true=发送时取反）
        self.declare_parameter("invert.right_x", False)
        # 右手柄有输入时是否禁用左手柄左右（true=用右摇杆时 linear.y 不发）
        self.declare_parameter("disable_left_x_when_right_active", False)
        # 右手柄角速度缩放（>1 加大转向）
        self.declare_parameter("velocity.angular_scale", 1.0)
        # true=定时器一直发 cmd_vel（即使摇杆回中）；false=仅在有速度或松杆瞬间发一次 0（不操作时不刷屏）
        self.declare_parameter("cmd_vel.publish_always", True)
        # 开机自启时：前若干秒只发全 0，避免手柄未就绪/扳机轴误映射导致乱动
        self.declare_parameter("startup.zero_cmd_sec", 0.0)
        # 0=关闭；0~1 越大越跟手、越小越稳（抑制摇杆噪声导致的偶发抖动）
        self.declare_parameter("axis.smoothing_alpha", 0.35)
        # 减速斜坡（仅减速/停止阶段生效；加速不限速保持响应性）
        self.declare_parameter("ramp.enabled", True)
        self.declare_parameter("ramp.linear_decel", 1.0)   # m/s²，vx/vy 减速度上限
        self.declare_parameter("ramp.angular_decel", 2.0)  # rad/s²，wz 减速度上限
        self.declare_parameter("axis.lt", 2)       # 左扳机
        self.declare_parameter("axis.rt", 5)       # 右扳机

        # true：首次按 A（上升沿）之前不发任何速度/升降（默认推荐，防开机乱动）
        self.declare_parameter("safety.gate_until_first_a", True)
        # true：必须按住 A 才允许速度/升降（松手即停）。与 gate 可同时开；仅 gate 时建议 false
        self.declare_parameter("safety.require_deadman", False)
        # true：B 上升沿急停并锁存，短按即可；再按 A（上升沿）解除。false：仅当帧发 0，不锁存
        self.declare_parameter("safety.estop_latch", True)
        # >0 时启用输入超时：超过该秒数未收到手柄事件则停止向话题发布
        self.declare_parameter("safety.input_timeout_sec", 1.0)

        # 1 是 A，2 是 B
        self.declare_parameter("button.a", 1)
        self.declare_parameter("button.b", 2)
        self.declare_parameter("button.lb", 4)
        self.declare_parameter("button.rb", 5)
        # LT / RT 用作加减速度缩放因子
        self.declare_parameter("button.lt", 8)
        self.declare_parameter("button.rt", 9)
        # X / Y 按钮用于采集控制（X=开始, Y=结束）
        self.declare_parameter("button.x", 0)
        self.declare_parameter("button.y", 3)

        # 采集（录制）功能：通过 HTTP 调用 web_control 的 /recording API
        self.declare_parameter("recording.enabled", True)
        self.declare_parameter("recording.server_url", "http://127.0.0.1:8080")

        device_path = self.get_parameter("device_path").get_parameter_value().string_value
        self.deadzone = float(self.get_parameter("deadzone").value)
        self.deadzone_remap = bool(self.get_parameter("deadzone.remap").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        self.startup_zero_cmd_sec = float(self.get_parameter("startup.zero_cmd_sec").value)
        self.axis_smoothing_alpha = float(self.get_parameter("axis.smoothing_alpha").value)
        self.ramp_enabled = bool(self.get_parameter("ramp.enabled").value)
        self.ramp_linear_decel = float(self.get_parameter("ramp.linear_decel").value)
        self.ramp_angular_decel = float(self.get_parameter("ramp.angular_decel").value)
        # 参数热调回调：ros2 param set 时同步更新缓存的 self.* 值
        self.add_on_set_parameters_callback(self._on_param_change)
        self.gate_until_first_a = bool(self.get_parameter("safety.gate_until_first_a").value)
        self.require_deadman = bool(self.get_parameter("safety.require_deadman").value)
        self.estop_latch = bool(self.get_parameter("safety.estop_latch").value)
        self.input_timeout_sec = float(self.get_parameter("safety.input_timeout_sec").value)

        self.max_linear = float(self.get_parameter("velocity.max_linear").value)
        self.max_angular = float(self.get_parameter("velocity.max_angular").value)

        # LT / RT 控制的缩放系数
        self.speed_min = float(self.get_parameter("speed_scale.min").value)
        self.speed_max = float(self.get_parameter("speed_scale.max").value)  # 默认上限
        self.speed_hard_max = float(self.get_parameter("speed_scale.hard_max").value)  # 绝对上限
        self.speed_scale = float(self.get_parameter("speed_scale.initial").value)
        self.speed_step = float(self.get_parameter("speed_scale.step").value)
        if self.speed_hard_max < self.speed_max:
            self.get_logger().warn(
                "speed_scale.hard_max < speed_scale.max, 已自动使用 speed_scale.max 作为 hard_max"
            )
            self.speed_hard_max = self.speed_max
        # 实际上限：取 max 与 hard_max 中较小者（此前未使用 speed_max，导致 yaml 里 max 不生效）
        self._speed_scale_upper = min(self.speed_max, self.speed_hard_max)
        initial_raw = self.speed_scale
        self.speed_scale = max(
            self.speed_min, min(self._speed_scale_upper, self.speed_scale)
        )
        if abs(initial_raw - self.speed_scale) > 1e-6:
            self.get_logger().info(
                f"speed_scale.initial ({initial_raw:.3f}) 已夹紧到 [min, max∩hard_max] -> {self.speed_scale:.3f}"
            )

        self.lift_speed = int(self.get_parameter("lift.speed").value)

        self.axis_left_x = int(self.get_parameter("axis.left_x").value)
        self.axis_left_y = int(self.get_parameter("axis.left_y").value)
        self.axis_right_x = int(self.get_parameter("axis.right_x").value)
        self.invert_left_x = bool(self.get_parameter("invert.left_x").value)
        self.invert_right_x = bool(self.get_parameter("invert.right_x").value)
        self.disable_left_x_when_right_active = bool(
            self.get_parameter("disable_left_x_when_right_active").value
        )
        self.angular_scale = float(self.get_parameter("velocity.angular_scale").value)
        self.cmd_vel_publish_always = bool(
            self.get_parameter("cmd_vel.publish_always").value
        )
        self.axis_lt = int(self.get_parameter("axis.lt").value)
        self.axis_rt = int(self.get_parameter("axis.rt").value)

        self.btn_a = int(self.get_parameter("button.a").value)
        self.btn_b = int(self.get_parameter("button.b").value)
        self.btn_lb = int(self.get_parameter("button.lb").value)
        self.btn_rb = int(self.get_parameter("button.rb").value)
        self.btn_lt = int(self.get_parameter("button.lt").value)
        self.btn_rt = int(self.get_parameter("button.rt").value)
        self.btn_x = int(self.get_parameter("button.x").value)
        self.btn_y = int(self.get_parameter("button.y").value)

        # 采集（录制）状态
        self.recording_enabled = bool(self.get_parameter("recording.enabled").value)
        self.recording_server_url = self.get_parameter("recording.server_url").get_parameter_value().string_value
        self._recording_active = False
        self._prev_x = 0
        self._prev_y = 0

        # 手柄读取线程
        self.joy = LinuxJoystickReader(device_path=device_path)
        self.joy.start()

        # 发布者
        self.cmd_vel_pub = self.create_publisher(Twist, "/svtrobot_cmd", 10)
        self.lift_pub = self.create_publisher(Int32MultiArray, "/lift_control_cmd", 10)

        # /f710/mode (String): "X" | "D" | "unknown" - 供 Web 前端做模式检测
        # /f710/status (Bool): 当前是否使能（由 /f710/enable 控制）
        self.mode_pub = self.create_publisher(String, "/f710/mode", 10)
        self.status_pub = self.create_publisher(Bool, "/f710/status", 10)
        self.debug_pub = self.create_publisher(String, "/f710/debug", 10)
        self.enable_sub = self.create_subscription(
            Bool, "/f710/enable", self._enable_callback, 10
        )
        self._enabled = True  # 默认使能；外部可通过 /f710/enable=false 暂停

        # 探测一次手柄 X/D 模式（基于 USB product ID，不依赖事件流）
        self._gamepad_mode = self._detect_gamepad_mode(device_path)

        # 发布频率
        self.timer = self.create_timer(1.0 / publish_rate, self._on_timer)
        # 慢速发布 mode/status（1Hz 足够前端刷新）
        self._mode_status_timer = self.create_timer(1.0, self._publish_mode_status)
        self._t_mono_start = time.monotonic()

        # 升降状态，仅在变化时发布
        self._last_lift_direction = 0
        self._last_lift_speed = 0

        # LT/RT 触发沿检测
        self._prev_lt_pressed = 0
        self._prev_rt_pressed = 0
        # 上一周期是否发过非零 cmd_vel（用于松杆时补发一次停车）
        self._prev_cmd_vel_active = False
        self._filt_axis = {"lx": 0.0, "ly": 0.0, "rx": 0.0}
        # 减速斜坡状态：保存上一周期实际发布的 cmd_vel
        self._ramp_state = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        self._ramp_last_mono = time.monotonic()
        self._prev_a = 0
        self._prev_b = 0
        self._estop_latched = False
        self._armed = False  # 已按过至少一次 A，允许发速度/升降
        self._input_timed_out = False

        self.get_logger().info(
            f"F710TeleopNode 已启动，设备: {device_path}, 频率: {publish_rate} Hz, "
            f"gate_until_first_a={self.gate_until_first_a}, require_deadman={self.require_deadman}, "
            f"estop_latch={self.estop_latch}"
        )
        self.get_logger().info(f"F710 手柄模式检测: {self._gamepad_mode}")
        if self._gamepad_mode == "X":
            self.get_logger().warn(
                "F710 处于 X 模式！控制已禁止。请将手柄背面开关拨到 D 模式后重新插上接收器。"
            )

    def _recording_request(self, endpoint: str) -> bool:
        """向 web_control 发送录制请求（/recording/start 或 /recording/stop）。"""
        if not self.recording_enabled:
            return False

        def _do() -> None:
            try:
                url = self.recording_server_url + endpoint
                req = urllib.request.Request(
                    url,
                    method="POST",
                    headers={"Content-Type": "application/json"},
                    data=b"{}",
                )
                with urllib.request.urlopen(req, timeout=5) as resp:
                    result = resp.read().decode()
                    self.get_logger().info(f"录制 {endpoint}: {result.strip()}")
            except urllib.error.URLError as exc:
                self.get_logger().warn(f"录制 {endpoint} 请求失败: {exc.reason}")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"录制 {endpoint} 异常: {exc}")

        threading.Thread(target=_do, daemon=True, name="recording-req").start()
        return True

    def _handle_recording_buttons(self, x_now: int, y_now: int) -> None:
        """X 上升沿开始采集，Y 上升沿停止采集。"""
        if not self.recording_enabled:
            return

        if x_now and not self._prev_x and not self._recording_active:
            self._recording_active = True
            self._recording_request('/recording/start')
            self.get_logger().info("手柄 X 按钮：开始采集")

        if y_now and not self._prev_y and self._recording_active:
            self._recording_active = False
            self._recording_request('/recording/stop')
            self.get_logger().info("手柄 Y 按钮：停止采集")

        self._prev_x = x_now
        self._prev_y = y_now

    @staticmethod
    def _detect_gamepad_mode(device_path: str) -> str:
        """根据 /sys/class/input/jsN/device/id/{vendor,product} 判断 F710 的 X/D 模式。

        Logitech F710:
          - vendor=046d, product=c21f → XInput (X 模式)
          - vendor=046d, product=c218/c219 → DirectInput (D 模式)
          - 其他 → unknown
        """
        try:
            m = re.search(r"/js(\d+)(?:\D|$)", device_path)
            if not m:
                return "unknown"
            base = f"/sys/class/input/js{m.group(1)}/device/id"
            with open(f"{base}/vendor") as f:
                vendor = f.read().strip().lower()
            with open(f"{base}/product") as f:
                product = f.read().strip().lower()
            if vendor != "046d":
                return "unknown"
            if product == "c21f":
                return "X"
            if product in ("c218", "c219"):
                return "D"
            return "unknown"
        except OSError:
            return "unknown"

    def _enable_callback(self, msg: Bool) -> None:
        """处理外部 /f710/enable 指令，True=使能手柄输出，False=暂停。"""
        new_state = bool(msg.data)
        if new_state != self._enabled:
            self.get_logger().warn(
                f"F710 手柄控制已由外部{'使能' if new_state else '禁用'}（/f710/enable → {new_state}）"
            )
            if not new_state:
                # 禁用瞬间补发一次全停，避免残留指令
                self._publish_all_stop()
        self._enabled = new_state

    def _publish_mode_status(self) -> None:
        """1Hz 慢速发布 mode/status，供 Web 前端订阅。

        同时重新探测 F710 X/D 模式：用户拨动背面物理开关时，sysfs 里的 product ID
        会变化（c21f ↔ c218/c219），这里读到后自动更新 _gamepad_mode，避免每次
        切换都要重启节点。
        """
        new_mode = self._detect_gamepad_mode(self.joy.device_path)
        # "unknown" 通常是 sysfs 暂时不可读（USB 重枚举中），保留上次的稳定值更安全
        if new_mode != "unknown" and new_mode != self._gamepad_mode:
            old_mode = self._gamepad_mode
            self._gamepad_mode = new_mode
            # 模式切换瞬间补发一次全停，防止轴/按钮映射突变导致乱动
            self._publish_all_stop()
            # 强制再按一次 A 解除保险（X↔D 切换时按键索引可能不同）
            self._armed = False
            self._estop_latched = False
            self.get_logger().warn(
                f"F710 模式自动切换：{old_mode} → {new_mode}，已补发全停并要求重新按 A 解锁"
            )
        self.mode_pub.publish(String(data=self._gamepad_mode))
        self.status_pub.publish(Bool(data=self._enabled))
        axes = [round(float(v), 3) for v in self.joy.axes[:6]]
        buttons = [int(v) for v in self.joy.buttons[:8]]
        debug = (
            f"mode={self._gamepad_mode} enabled={self._enabled} "
            f"armed={self._armed} estop={self._estop_latched} "
            f"alive={self.joy.is_alive()} speed_scale={self.speed_scale:.3f} "
            f"axes0_5={axes} buttons0_7={buttons}"
        )
        self.debug_pub.publish(String(data=debug))

    def _apply_deadzone(self, value: float) -> float:
        a = abs(value)
        if a < self.deadzone:
            return 0.0
        if not self.deadzone_remap or self.deadzone >= 1.0 - 1e-9:
            return value
        # 将 (deadzone, 1.0] 线性映射到 (0, 1.0]，避免刚过死区就接近满指令
        sign = 1.0 if value > 0 else -1.0
        return sign * (a - self.deadzone) / (1.0 - self.deadzone)

    def _twist_is_idle(self, twist: Twist) -> bool:
        eps = 1e-9
        return (
            abs(twist.linear.x) < eps
            and abs(twist.linear.y) < eps
            and abs(twist.linear.z) < eps
            and abs(twist.angular.x) < eps
            and abs(twist.angular.y) < eps
            and abs(twist.angular.z) < eps
        )

    def _sanitize_twist(self, twist: Twist) -> None:
        """静止时打成真正 0.0，避免取反后出现 -0.0。"""
        eps = 1e-9
        for attr in ("x", "y", "z"):
            v = getattr(twist.linear, attr)
            setattr(twist.linear, attr, 0.0 if abs(v) < eps else float(v))
            v = getattr(twist.angular, attr)
            setattr(twist.angular, attr, 0.0 if abs(v) < eps else float(v))

    def _get_axis(self, index: int) -> float:
        if 0 <= index < len(self.joy.axes):
            return self.joy.axes[index]
        return 0.0

    def _smooth_axis(self, key: str, raw: float) -> float:
        a = min(max(self.axis_smoothing_alpha, 0.0), 1.0)
        if a <= 1e-9:
            return raw
        prev = self._filt_axis[key]
        out = prev + a * (raw - prev)
        self._filt_axis[key] = out
        return out

    def _get_button(self, index: int) -> int:
        if 0 <= index < len(self.joy.buttons):
            return self.joy.buttons[index]
        return 0

    def _handle_speed_scale_change(self) -> None:
        """
        使用 LT / RT 按钮加减底盘速度缩放因子:
        - RT：增加 speed_scale
        - LT：减小 speed_scale
        范围: [speed_min, min(speed_max, speed_hard_max)]
        """
        lt_pressed = self._get_button(self.btn_lt)
        rt_pressed = self._get_button(self.btn_rt)

        # RT 上升沿：加速
        if rt_pressed and not self._prev_rt_pressed:
            old = self.speed_scale
            self.speed_scale = min(
                self._speed_scale_upper, self.speed_scale + self.speed_step
            )
            self.get_logger().info(
                f"RT 加速: {old:.2f} -> {self.speed_scale:.2f}"
            )

        # LT 上升沿：减速
        if lt_pressed and not self._prev_lt_pressed:
            old = self.speed_scale
            self.speed_scale = max(self.speed_min, self.speed_scale - self.speed_step)
            self.get_logger().info(
                f"LT 减速: {old:.2f} -> {self.speed_scale:.2f}"
            )

        self._prev_lt_pressed = lt_pressed
        self._prev_rt_pressed = rt_pressed

    def _compute_cmd_vel(self) -> Twist:
        left_y = self._apply_deadzone(
            self._smooth_axis("ly", self._get_axis(self.axis_left_y))
        )
        left_x = self._apply_deadzone(
            self._smooth_axis("lx", self._get_axis(self.axis_left_x))
        )
        right_x = self._apply_deadzone(
            self._smooth_axis("rx", self._get_axis(self.axis_right_x))
        )

        # 底盘速度根据当前 speed_scale 进行缩放
        # 这里对前后方向取反，使“推前为正”（根据当前手柄坐标系调整）
        vx = -left_y * self.max_linear * self.speed_scale
        vy = left_x * self.max_linear * self.speed_scale
        wz = right_x * self.max_angular * self.speed_scale * self.angular_scale

        # 右手柄有输入时禁用左手柄左右（不发 linear.y）
        if self.disable_left_x_when_right_active and abs(right_x) > 0:
            vy = 0.0

        # 右手柄取反：左手柄往后（left_y > 0）时不取反，其余情况按 invert.right_x
        apply_right_invert = self.invert_right_x and (left_y <= 0)

        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = -vy if self.invert_left_x else vy  # 发送时对 vy 取反
        msg.angular.z = -wz if apply_right_invert else wz  # 发送时对 angular.z 取反（往后时不取反）
        return msg

    def _on_param_change(self, params: list) -> SetParametersResult:
        """ros2 param set 的回调：把新值同步到缓存的实例变量，使 ramp.* 真正支持热调。"""
        result = SetParametersResult(successful=True)
        for p in params:
            if p.name == "ramp.enabled":
                self.ramp_enabled = bool(p.value)
                self.get_logger().info(f"ramp.enabled 热调 -> {self.ramp_enabled}")
            elif p.name == "ramp.linear_decel":
                v = float(p.value)
                if v < 0.0:
                    result.successful = False
                    result.reason = "ramp.linear_decel 不能为负"
                    return result
                self.ramp_linear_decel = v
                self.get_logger().info(f"ramp.linear_decel 热调 -> {v:.3f} m/s²")
            elif p.name == "ramp.angular_decel":
                v = float(p.value)
                if v < 0.0:
                    result.successful = False
                    result.reason = "ramp.angular_decel 不能为负"
                    return result
                self.ramp_angular_decel = v
                self.get_logger().info(f"ramp.angular_decel 热调 -> {v:.3f} rad/s²")
        return result

    def _apply_decel_ramp(self, twist: Twist) -> Twist:
        """对 cmd_vel 应用减速斜坡：减速阶段受 decel 限制，加速阶段不限速。"""
        if not self.ramp_enabled:
            # 即使禁用也要同步 state，避免重新启用时跳变
            self._ramp_state["vx"] = twist.linear.x
            self._ramp_state["vy"] = twist.linear.y
            self._ramp_state["wz"] = twist.angular.z
            return twist

        now = time.monotonic()
        dt = max(min(now - self._ramp_last_mono, 0.1), 0.001)  # 钳到 [1ms, 100ms]
        self._ramp_last_mono = now

        out = Twist()
        for axis, target, max_rate in (
            ("vx", twist.linear.x,  self.ramp_linear_decel),
            ("vy", twist.linear.y,  self.ramp_linear_decel),
            ("wz", twist.angular.z, self.ramp_angular_decel),
        ):
            current = self._ramp_state[axis]
            # 判断是否为"减速/反向"：|target| 变小 或 符号翻转
            is_decel = (abs(target) < abs(current)) or (
                (target * current) < 0.0  # 异号 = 反向
            )
            if is_decel and max_rate > 0.0:
                max_step = max_rate * dt
                delta = target - current
                if delta >  max_step: delta =  max_step
                if delta < -max_step: delta = -max_step
                v_new = current + delta
            else:
                v_new = target  # 加速阶段直接跟手
            self._ramp_state[axis] = v_new

            if axis == "vx": out.linear.x  = v_new
            if axis == "vy": out.linear.y  = v_new
            if axis == "wz": out.angular.z = v_new
        return out

    def _reset_ramp_state(self) -> None:
        """安全事件（急停/禁用/模式切换）后清零 ramp state，避免下次起步被旧值拖住。"""
        self._ramp_state = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
        self._ramp_last_mono = time.monotonic()

    def _compute_lift_command(self) -> (int, int):
        lb = self._get_button(self.btn_lb)
        rb = self._get_button(self.btn_rb)

        if rb and not lb:
            direction = 1
        elif lb and not rb:
            direction = -1
        else:
            direction = 0

        speed = self.lift_speed if direction != 0 else 0
        return direction, speed

    def _publish_all_stop(self) -> None:
        self._reset_ramp_state()
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        lift_msg = Int32MultiArray()
        lift_msg.data = [0, 0]
        self.lift_pub.publish(lift_msg)
        self._last_lift_direction = 0
        self._last_lift_speed = 0
        self._prev_cmd_vel_active = False

    def _on_timer(self) -> None:
        # 安全保护：X 模式下轴/按钮映射全部错乱，禁止发布任何控制指令
        if self._gamepad_mode == "X":
            return
        # 外部禁用（/f710/enable=false）：不发任何控制话题
        # _enable_callback 已在切换瞬间补发过全停
        if not self._enabled:
            return
        # 设备失联保护：仅在 USB 接收器真正掉线（fd 断）时才停止发布。
        # 注意：不能用"事件时间"判断失联——Linux joy 驱动在摇杆静止时本来就不发事件，
        # 那样会把"摇杆推到固定位置保持不动"误判为失联，导致底盘周期性卡顿。
        if not self.joy.is_alive():
            if not self._input_timed_out:
                self.get_logger().warn(
                    "手柄设备失联（fd 断开/接收器掉电），暂停发布控制话题"
                )
                self._input_timed_out = True
            self._prev_lt_pressed = 0
            self._prev_rt_pressed = 0
            self._prev_a = 0
            self._prev_b = 0
            self._prev_cmd_vel_active = False
            return
        elif self._input_timed_out:
            self.get_logger().info("手柄输入恢复，继续发布控制话题")
            self._input_timed_out = False

        a_now = self._get_button(self.btn_a)
        b_now = self._get_button(self.btn_b)
        x_now = self._get_button(self.btn_x)
        y_now = self._get_button(self.btn_y)
        self._handle_recording_buttons(x_now, y_now)
        try:
            # B 上升沿：急停（短按即可；锁存后需按 A 上升沿解除）
            if b_now and not self._prev_b:
                if self.estop_latch:
                    self._estop_latched = True
                self._publish_all_stop()
                self.get_logger().warn(
                    "急停(B)：已清零"
                    + ("并锁存，请再按一次 A 解除" if self.estop_latch else "")
                )
                return

            if self._estop_latched and a_now and not self._prev_a:
                self._estop_latched = False
                self.get_logger().info("急停已解除（A）")

            if self._estop_latched:
                self._publish_all_stop()
                return

            in_startup = (
                self.startup_zero_cmd_sec > 0.0
                and (time.monotonic() - self._t_mono_start) < self.startup_zero_cmd_sec
            )
            if in_startup:
                self._reset_ramp_state()
                twist = Twist()
                self.cmd_vel_pub.publish(twist)
                self._prev_cmd_vel_active = False
                return

            # 未按过 A 前：不发速度/升降（仅发全 0，与急停类似）
            if self.gate_until_first_a and not self._armed:
                if a_now and not self._prev_a:
                    self._armed = True
                    self.get_logger().info("已使能：检测到 A，开始允许发速度/升降指令")
                if not self._armed:
                    # 关键约束：首次 A 之前不发布任何控制话题
                    self._reset_ramp_state()
                    return

            if self.require_deadman and not a_now:
                self._reset_ramp_state()
                twist = Twist()
                self._sanitize_twist(twist)
                if self.cmd_vel_publish_always:
                    self.cmd_vel_pub.publish(twist)
                elif self._prev_cmd_vel_active:
                    self.cmd_vel_pub.publish(twist)
                    self._prev_cmd_vel_active = False
                if self._last_lift_direction != 0 or self._last_lift_speed != 0:
                    lift_msg = Int32MultiArray()
                    lift_msg.data = [0, 0]
                    self.lift_pub.publish(lift_msg)
                    self._last_lift_direction = 0
                    self._last_lift_speed = 0
                return

            # LT / RT 调整底盘速度缩放（仅使能时）
            self._handle_speed_scale_change()

            # 发布底盘速度（静止时数值归零，避免 linear.x: -0.0 等）
            twist = self._compute_cmd_vel()
            self._sanitize_twist(twist)
            # 减速斜坡：让松杆/减速过程平缓（仅减速阶段限速，起步仍跟手）
            twist = self._apply_decel_ramp(twist)
            self._sanitize_twist(twist)  # ramp 后再 sanitize 一次，防止产生 -0.0
            idle = self._twist_is_idle(twist)
            if self.cmd_vel_publish_always:
                self.cmd_vel_pub.publish(twist)
                self._prev_cmd_vel_active = not idle
            else:
                if not idle:
                    self.cmd_vel_pub.publish(twist)
                    self._prev_cmd_vel_active = True
                elif self._prev_cmd_vel_active:
                    self.cmd_vel_pub.publish(twist)
                    self._prev_cmd_vel_active = False

            direction, speed = self._compute_lift_command()
            if direction != self._last_lift_direction or speed != self._last_lift_speed:
                lift_msg = Int32MultiArray()
                lift_msg.data = [direction, speed]
                self.lift_pub.publish(lift_msg)
                self._last_lift_direction = direction
                self._last_lift_speed = speed
        finally:
            self._prev_a = a_now
            self._prev_b = b_now

    def destroy_node(self) -> bool:
        self.joy.stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = F710TeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

