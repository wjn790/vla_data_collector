# SVTROBO Python 控制与状态获取 API 使用文档

> 文件位置：`src/chassis_control/scripts/svtrobo_controller.py`

---

## 目录

1. [前置条件](#1-前置条件)
2. [快速开始](#2-快速开始)
3. [控制器初始化](#3-控制器初始化)
4. [底盘控制 API](#4-底盘控制-api)
5. [状态获取 API](#5-状态获取-api)
6. [升降机构控制 API](#6-升降机构控制-api)
7. [深度学习集成 API](#7-深度学习集成-api)
8. [生命周期管理](#8-生命周期管理)
9. [完整代码示例](#9-完整代码示例)
10 [ROS Topic 说明](#10-ros-topic-说明)
11. [注意事项与常见问题](#11-注意事项与常见问题)

---

## 1. 前置条件

### 依赖

```bash
pip install numpy
```

ROS2 环境（`rclpy`、`geometry_msgs`、`sensor_msgs`、`std_msgs`）需要已安装并 source：

```bash
source /home/openarm/svtrobo_ws/install/setup.bash
```

### 启动底盘节点

Python 脚本依赖底盘 C++ 节点发布的状态数据，必须先启动底盘：

```bash
# 终端 1：启动底盘
cd /home/openarm/svtrobo_ws
colcon build --packages-select chassis_control
source install/setup.bash
ros2 launch chassis_control svtrobo_bringup.launch.py
```

### 导入模块

```python
import sys
sys.path.append("/home/openarm/svtrobo_ws/src/chassis_control/scripts")

from svtrobo_controller import SVTROBOController
```

或者直接把 `svtrobo_controller.py` 复制到你的项目目录中导入。

---

## 2. 快速开始

最简单的使用方式——用 `with` 语句自动管理生命周期：

```python
import time
from svtrobo_controller import SVTROBOController

with SVTROBOController() as robot:
    # 等待底盘状态就绪
    state = robot.wait_for_state(timeout=10.0)
    if state is None:
        print("底盘未连接")
        exit(1)

    # 前进 2 秒
    robot.move_forward(0.3)
    time.sleep(2)

    # 停止（with 退出时也会自动 stop）
    robot.stop()
```

> `with` 块退出时，控制器会自动调用 `stop()` 停止底盘并关闭 ROS2 节点。

---

## 3. 控制器初始化

### 构造函数

```python
SVTROBOController(
    node_name="svtrobo_controller",  # ROS2 节点名称
    auto_stop_timeout=0.0,           # 自动停止超时（秒），0 = 禁用
)
```

**参数说明：**

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `node_name` | str | `"svtrobo_controller"` | 在 ROS2 中注册的节点名，同一进程内不可重复 |
| `auto_stop_timeout` | float | `0.0` | 状态反馈中断超过该秒数后，每次调用 `move()` 前会自动发送零速。设为 0 禁用此功能 |

### 启动方式

有两种方式启动控制器：

#### 方式一：`with` 语句（推荐）

```python
with SVTROBOController() as robot:
    # 此处 robot 已自动启动 spin 线程
    robot.move_forward(0.3)
# 退出 with 块时自动 stop() + shutdown()
```

#### 方式二：手动管理

```python
robot = SVTROBOController()
robot.start()                    # 启动后台 spin 线程

robot.move_forward(0.3)
time.sleep(2)
robot.stop()

robot.shutdown()                 # 手动关闭
```

---

## 4. 底盘控制 API

所有控制方法调用后立即通过 ROS2 Topic 发送指令到底盘 C++ 节点。这些方法是**非阻塞的**，调用后立即返回。

### 4.1 `move(vx, vy, wz)` — 全向移动

核心控制方法，其他方向方法都是它的封装。

```python
robot.move(vx=0.5, vy=0.0, wz=0.0)
```

| 参数 | 类型 | 单位 | 说明 |
|------|------|------|------|
| `vx` | float | m/s | 前进速度，正值前进，负值后退 |
| `vy` | float | m/s | 横移速度，正值左移，负值右移 |
| `wz` | float | rad/s | 旋转角速度，正值逆时针，负值顺时针 |

> 底盘内部有速度限制机制：任何轮子超过 100 RPM 时所有轮速等比缩放，不会损坏硬件。

### 4.2 `move_forward(speed)` — 前进

```python
robot.move_forward(0.1)    # 以 0.1 m/s 前进
```

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `speed` | float | 0.1 | 前进速度 (m/s)，必须为正值 |

### 4.3 `move_backward(speed)` — 后退

```python
robot.move_backward(0.1)   # 以 0.1 m/s 后退
```

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `speed` | float | 0.1 | 后退速度 (m/s)，必须为正值 |

### 4.4 `move_left(speed)` — 左移

```python
robot.move_left(0.1)       # 以 0.1 m/s 左移
```

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `speed` | float | 0.1 | 左移速度 (m/s) |

### 4.5 `move_right(speed)` — 右移

```python
robot.move_right(0.1)      # 以 0.1 m/s 右移
```

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `speed` | float | 0.1 | 右移速度 (m/s) |

### 4.6 `rotate(speed)` — 原地旋转

```python
robot.rotate(0.2)          # 以 0.2 rad/s 逆时针旋转
robot.rotate(-0.2)         # 以 0.2 rad/s 顺时针旋转
```

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `speed` | float | 0.2 | 旋转角速度 (rad/s)，正=逆时针，负=顺时针 |

### 4.7 `stop()` — 停止底盘

```python
robot.stop()
```

发送零速指令（vx=0, vy=0, wz=0）。无参数。

> **重要**：底盘节点不会自动停止。一旦发送了运动指令，底盘会持续执行该指令直到收到新的指令。在程序结束前务必调用 `stop()`。

### 4.8 组合运动示例

```python
# 边前进边左移（斜向运动）
robot.move(vx=0.3, vy=0.2, wz=0.0)

# 边前进边旋转（弧线运动）
robot.move(vx=0.5, vy=0.0, wz=0.3)

# 全向运动：前进 + 左移 + 旋转
robot.move(vx=0.3, vy=0.2, wz=0.5)
```

---

## 5. 状态获取 API

底盘 C++ 节点以 **100Hz** 频率发布状态数据到 `/chassis/joint_states` Topic。Python 端通过后台 spin 线程接收并缓存最新状态。

### 5.1 `get_state()` — 获取完整状态（非阻塞）

```python
state = robot.get_state()
if state is None:
    print("尚未收到状态数据")
else:
    print(state["steer_angles"])
```

**返回值**：字典或 `None`（未收到数据时）

| 字段 | 类型 | 说明 |
|------|------|------|
| `stamp` | ROS2 Time | 时间戳 |
| `steer_angles` | `List[float]` | 4 个舵向实际角度 `[FL, FR, RL, RR]`，单位 rad |
| `steer_velocities` | `List[float]` | 4 个舵向速度 `[FL, FR, RL, RR]`，单位 rad/s |
| `steer_torques` | `List[float]` | 4 个舵向力矩 `[FL, FR, RL, RR]`，单位 Nm |
| `wheel_speeds` | `List[float]` | 4 个轮子目标转速 `[FL, FR, RL, RR]`，单位 RPM |
| `vx_set` | float | 当前指令前进速度 (m/s) |
| `vy_set` | float | 当前指令横移速度 (m/s) |
| `wz_set` | float | 当前指令旋转角速度 (rad/s) |

> 数组索引对应：`[0]=FL(前左), [1]=FR(前右), [2]=RL(后左), [3]=RR(后右)`

### 5.2 `wait_for_state(timeout)` — 阻塞等待状态

```python
# 阻塞等待，最多 5 秒
state = robot.wait_for_state(timeout=5.0)
if state is None:
    print("等待超时，底盘未连接")
```

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `timeout` | float | 5.0 | 等待超时秒数 |

**返回值**：与 `get_state()` 相同的字典，超时返回 `None`。

> 首次调用会等待直到收到第一条状态数据。适合在程序启动时确认底盘连接。

### 5.3 属性访问 — 快捷读取单项数据

所有属性在未收到数据时返回 `None`。

```python
# 舵向角度 [FL, FR, RL, RR]，单位 rad
angles = robot.steer_angles
# 示例: [0.201, 4.897, 2.703, 3.501]

# 舵向速度 [FL, FR, RL, RR]，单位 rad/s
vels = robot.steer_velocities
# 示例: [0.012, -0.005, 0.003, 0.001]

# 舵向力矩 [FL, FR, RL, RR]，单位 Nm
torques = robot.steer_torques
# 示例: [0.15, 0.12, 0.13, 0.14]

# 轮子目标转速 [FL, FR, RL, RR]，单位 RPM
speeds = robot.wheel_speeds
# 示例: [42.5, 42.3, 41.8, 42.1]

# 最近发送的指令
cmd = robot.last_cmd
# 示例: {"vx": 0.3, "vy": 0.0, "wz": 0.0}

# 底盘是否已连接（收到过状态数据）
connected = robot.is_connected
# 示例: True
```

| 属性 | 类型 | 说明 |
|------|------|------|
| `steer_angles` | `Optional[List[float]]` | 舵向角度 `[FL, FR, RL, RR]` (rad) |
| `steer_velocities` | `Optional[List[float]]` | 舵向速度 `[FL, FR, RL, RR]` (rad/s) |
| `steer_torques` | `Optional[List[float]]` | 舵向力矩 `[FL, FR, RL, RR]` (Nm) |
| `wheel_speeds` | `Optional[List[float]]` | 轮子目标转速 `[FL, FR, RL, RR]` (RPM) |
| `last_cmd` | `Optional[Dict[str, float]]` | 最近指令 `{"vx": ..., "vy": ..., "wz": ...}` |
| `is_connected` | `bool` | 是否已收到状态数据 |

### 5.4 实时状态监控示例

```python
import time
from svtrobo_controller import SVTROBOController

with SVTROBOController() as robot:
    state = robot.wait_for_state(timeout=10.0)
    if state is None:
        print("底盘未连接")
        exit(1)

    # 以 10Hz 打印状态，持续 10 秒
    robot.move_forward(0.2)
    for i in range(100):
        state = robot.get_state()
        if state:
            angles = [f"{a:.3f}" for a in state["steer_angles"]]
            speeds = [f"{s:.1f}" for s in state["wheel_speeds"]]
            print(f"[{i:03d}] angles={angles}  speeds={speeds}  "
                  f"cmd=({state['vx_set']:.2f}, {state['vy_set']:.2f}, {state['wz_set']:.2f})")
        time.sleep(0.1)

    robot.stop()
```

---

## 6. 升降机构控制 API

### 6.1 `control_lift(direction, speed)` — 控制升降

```python
# 正转 300 RPM
robot.control_lift(1, 300)

# 反转 200 RPM
robot.control_lift(-1, 200)
```

| 参数 | 类型 | 说明 |
|------|------|------|
| `direction` | int | `1`=正转, `-1`=反转, `0`=停止 |
| `speed` | int | RPM，范围 1~6000 |

### 6.2 `stop_lift()` — 停止升降

```python
robot.stop_lift()
```

---

## 7. 深度学习集成 API

### 7.1 `get_observation()` — 获取观测向量

将底盘状态打包为一个 numpy 数组，可直接作为神经网络输入。

```python
obs = robot.get_observation()
if obs is not None:
    print(obs.shape)    # (16,)
    print(obs)
```

**观测向量布局（16 维 float32）：**

| 索引 | 数据 | 单位 |
|------|------|------|
| `[0]` | FL 舵向角度 | rad |
| `[1]` | FR 舵向角度 | rad |
| `[2]` | RL 舵向角度 | rad |
| `[3]` | RR 舵向角度 | rad |
| `[4]` | FL 舵向速度 | rad/s |
| `[5]` | FR 舵向速度 | rad/s |
| `[6]` | RL 舵向速度 | rad/s |
| `[7]` | RR 舵向速度 | rad/s |
| `[8]` | FL 舵向力矩 | Nm |
| `[9]` | FR 舵向力矩 | Nm |
| `[10]` | RL 舵向力矩 | Nm |
| `[11]` | RR 舵向力矩 | Nm |
| `[12]` | FL 轮子转速 | RPM |
| `[13]` | FR 轮子转速 | RPM |
| `[14]` | RL 轮子转速 | RPM |
| `[15]` | RR 轮子转速 | RPM |

> FL=前左, FR=前右, RL=后左, RR=后右

**返回值**：`np.ndarray`，shape `(16,)`，dtype `float32`。未收到数据时返回 `None`。

### 7.2 `publish_action(action)` — 发送模型输出

将模型推理结果直接发送为底盘控制指令。

```python
import numpy as np

# 模型输出 [vx, vy, wz]
action = np.array([0.5, 0.1, 0.2])
robot.publish_action(action)
```

| 参数 | 类型 | 说明 |
|------|------|------|
| `action` | array-like | 长度 3 的数组 `[vx, vy, wz]`，单位分别为 m/s, m/s, rad/s |

> `action` 支持 numpy 数组、Python 列表等任何可转换为 numpy 的格式。

### 7.3 推理循环示例

```python
import numpy as np
import time
from svtrobo_controller import SVTROBOController

# 假设你已训练好的模型
# model = load_your_model("model.onnx")

with SVTROBOController(auto_stop_timeout=0.5) as robot:
    state = robot.wait_for_state(timeout=10.0)
    if state is None:
        print("底盘未连接")
        exit(1)

    print("开始推理控制...")

    try:
        while True:
            # 1. 获取观测
            obs = robot.get_observation()
            if obs is None:
                continue

            # 2. 模型推理
            # action = model.predict(obs.reshape(1, -1))[0]  # 你的模型
            action = np.array([0.3, 0.0, 0.1])  # 示例：替换为实际推理

            # 3. 发送动作
            robot.publish_action(action)

            # 4. 控制推理频率
            time.sleep(0.05)  # 20Hz

    except KeyboardInterrupt:
        print("用户中断")

    robot.stop()
```

### 7.4 数据采集示例

采集 (观测, 动作) 对用于行为克隆训练：

```python
import csv
import time
import numpy as np
from svtrobo_controller import SVTROBOController

OUTPUT_FILE = "training_data.csv"

with SVTROBOController() as robot:
    state = robot.wait_for_state(timeout=10.0)
    if state is None:
        print("底盘未连接")
        exit(1)

    dataset = []
    print("开始采集数据，Ctrl+C 停止...")

    try:
        while True:
            obs = robot.get_observation()
            cmd = robot.last_cmd
            if obs is not None and cmd is not None:
                row = {
                    "timestamp": time.time(),
                    # 观测 (16维)
                    "fl_steer_angle": obs[0],
                    "fr_steer_angle": obs[1],
                    "rl_steer_angle": obs[2],
                    "rr_steer_angle": obs[3],
                    "fl_steer_vel": obs[4],
                    "fr_steer_vel": obs[5],
                    "rl_steer_vel": obs[6],
                    "rr_steer_vel": obs[7],
                    "fl_steer_torque": obs[8],
                    "fr_steer_torque": obs[9],
                    "rl_steer_torque": obs[10],
                    "rr_steer_torque": obs[11],
                    "fl_wheel_speed": obs[12],
                    "fr_wheel_speed": obs[13],
                    "rl_wheel_speed": obs[14],
                    "rr_wheel_speed": obs[15],
                    # 动作 (3维)
                    "vx": cmd["vx"],
                    "vy": cmd["vy"],
                    "wz": cmd["wz"],
                }
                dataset.append(row)
            time.sleep(0.1)  # 10Hz 采集

    except KeyboardInterrupt:
        print(f"\n采集结束，共 {len(dataset)} 条记录")

    # 保存到 CSV
    if dataset:
        keys = dataset[0].keys()
        with open(OUTPUT_FILE, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(dataset)
        print(f"已保存到 {OUTPUT_FILE}")
```

---

## 8. 生命周期管理

### 8.1 两种使用模式对比

#### 模式 A：`with` 语句（推荐，适合脚本和简单场景）

```python
with SVTROBOController() as robot:
    robot.move_forward(0.3)
    time.sleep(2)
    robot.stop()
# 自动 shutdown()
```

- 进入 `with` 时自动调用 `start()`
- 退出 `with` 时自动调用 `stop()` + `shutdown()`
- 即使发生异常也会正确清理

#### 模式 B：手动管理（适合长期运行或嵌入其他系统）

```python
robot = SVTROBOController()
robot.start()

try:
    robot.move_forward(0.3)
    time.sleep(2)
    robot.stop()
finally:
    robot.shutdown()  # 确保清理
```

### 8.2 `start()` — 启动后台线程

```python
robot.start()  # 返回 self，支持链式调用
```

启动 ROS2 spin 后台线程，开始接收状态数据。调用 `start()` 后才能收到状态。

### 8.3 `shutdown()` — 关闭控制器

```python
robot.shutdown()
```

执行以下操作：
1. 调用 `stop()` 停止底盘
2. 停止后台 spin 线程
3. 销毁 ROS2 节点

### 8.4 自动停止超时

通过 `auto_stop_timeout` 参数启用安全保护：

```python
# 如果状态反馈中断超过 1 秒，每次 move 前会自动发送零速
robot = SVTROBOController(auto_stop_timeout=1.0)
```

工作原理：每次调用 `move()` 时检查距最后一次收到状态数据的时间差。如果超过 `auto_stop_timeout`，先发送零速再发送新指令。

---

## 9. 完整代码示例

### 示例 1：基础运动测试

```python
#!/usr/bin/env python3
"""测试底盘所有运动方向"""

import time
from svtrobo_controller import SVTROBOController

MOTION_TESTS = [
    ("前进",   lambda r: r.move_forward(0.2),   2.0),
    ("后退",   lambda r: r.move_backward(0.2),  2.0),
    ("左移",   lambda r: r.move_left(0.2),      2.0),
    ("右移",   lambda r: r.move_right(0.2),     2.0),
    ("逆时针旋转", lambda r: r.rotate(0.3),     2.0),
    ("顺时针旋转", lambda r: r.rotate(-0.3),    2.0),
    ("斜向运动",   lambda r: r.move(0.2, 0.2, 0.0), 2.0),
    ("弧线运动",   lambda r: r.move(0.3, 0.0, 0.3), 2.0),
]

with SVTROBOController() as robot:
    state = robot.wait_for_state(timeout=10.0)
    if state is None:
        print("底盘未连接")
        exit(1)

    for name, motion_fn, duration in MOTION_TESTS:
        print(f"执行: {name} ({duration}秒)")
        motion_fn(robot)
        time.sleep(duration)
        robot.stop()
        time.sleep(1.0)  # 间隔

    print("测试完成")
```

### 示例 2：定时状态记录

```python
#!/usr/bin/env python3
"""以固定频率采集底盘状态数据"""

import json
import time
from svtrobo_controller import SVTROBOController

RECORD_HZ = 20       # 采集频率
RECORD_SEC = 30      # 采集时长

with SVTROBOController() as robot:
    state = robot.wait_for_state(timeout=10.0)
    if state is None:
        print("底盘未连接")
        exit(1)

    records = []
    start = time.time()

    while time.time() - start < RECORD_SEC:
        state = robot.get_state()
        if state:
            state["time"] = time.time() - start
            # ROS Time 对象不能直接 JSON 序列化，移除
            state.pop("stamp", None)
            records.append(state)
        time.sleep(1.0 / RECORD_HZ)

    with open("state_record.json", "w") as f:
        json.dump(records, f, indent=2)

    print(f"采集 {len(records)} 条记录，已保存到 state_record.json")
```

### 示例 3：键盘遥控

```python
#!/usr/bin/env python3
"""WASD 键盘遥控底盘"""

import sys
import termios
import tty
import time
from svtrobo_controller import SVTROBOController

SPEED = 0.3
ROT_SPEED = 0.5

def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

print("键盘遥控 (WASD + Q/E 旋转, 空格停止, ESC 退出)")

with SVTROBOController() as robot:
    state = robot.wait_for_state(timeout=10.0)
    if state is None:
        print("底盘未连接")
        exit(1)

    while True:
        key = get_key()
        if key == "\x1b":  # ESC
            break
        elif key == "w":
            robot.move_forward(SPEED)
        elif key == "s":
            robot.move_backward(SPEED)
        elif key == "a":
            robot.move_left(SPEED)
        elif key == "d":
            robot.move_right(SPEED)
        elif key == "q":
            robot.rotate(ROT_SPEED)
        elif key == "e":
            robot.rotate(-ROT_SPEED)
        elif key == " ":
            robot.stop()

    robot.stop()
    print("退出")
```

---

## 10. ROS Topic 说明

Python 脚本通过以下 ROS2 Topic 与底盘 C++ 节点通信：

```
Python 节点 (svtrobo_controller)          C++ 底盘节点 (chassis_control_node)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━          ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

                    ── Twist ──→
    /svtrobot_cmd                  底盘接收速度指令
    (控制指令)

                    ←─ JointState ──
    /chassis/joint_states          Python 接收底盘状态
    (状态反馈, 100Hz)

                    ←─ Twist ──
    /chassis/cmd_feedback          Python 接收当前指令回显
    (指令回显, 100Hz)

                    ←─ ChassisDiagnostics ──
    /chassis/diagnostics           诊断数据（电压/温度/错误码/实际轮速）
    (诊断数据, 100Hz)

                    ── Int32MultiArray ──→
    /lift_control_cmd              升降机构控制
    (升降控制)
```

**Topic 详细说明：**

| Topic | 消息类型 | 方向 | 频率 | 说明 |
|-------|---------|------|------|------|
| `/svtrobot_cmd` | `geometry_msgs/msg/Twist` | Python → C++ | 按需 | 控制指令 (vx, vy, wz) |
| `/chassis/joint_states` | `sensor_msgs/msg/JointState` | C++ → Python | 100Hz | 底盘关节状态 |
| `/chassis/cmd_feedback` | `geometry_msgs/msg/Twist` | C++ → Python | 100Hz | 当前指令回显 |
| `/chassis/diagnostics` | `chassis_control/msg/ChassisDiagnostics` | C++ → Python | 100Hz | 诊断数据（电压/温度/错误码/实际轮速） |
| `/lift_control_cmd` | `std_msgs/msg/Int32MultiArray` | Python → C++ | 按需 | 升降控制指令 |

**JointState 消息字段：**

| 字段 | name[0..3] | name[4..7] |
|------|-----------|-----------|
| `name` | `fl_steer`, `fr_steer`, `rl_steer`, `rr_steer` | `fl_wheel`, `fr_wheel`, `rl_wheel`, `rr_wheel` |
| `position` | 4 个舵向电机实际角度 (rad) | 0.0（轮子无位置反馈） |
| `velocity` | 4 个舵向电机速度 (rad/s) | 4 个轮子目标转速 (RPM) |
| `effort` | 4 个舵向电机力矩 (Nm) | 0.0（轮子无力矩反馈） |

**ChassisDiagnostics 消息字段：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `header` | `std_msgs/Header` | 时间戳 |
| `vbus` | `float32` | 电机驱动器总线电压 (V) |
| `motor_temperatures` | `float32[4]` | 4 个舵向电机温度 (°C)：FL, FR, RL, RR |
| `motor_error_codes` | `uint8[4]` | 4 个舵向电机错误码：FL, FR, RL, RR |
| `wheel_speeds_actual` | `float32[4]` | ZLAC8015D 实际轮速 (RPM)：FL, FR, RL, RR |

---

## 11. 注意事项与常见问题

### Q1: 运行脚本报错 `ModuleNotFoundError: No module named 'rclpy'`

ROS2 环境未 source。执行：

```bash
source /home/openarm/svtrobo_ws/install/setup.bash
```

### Q2: `wait_for_state()` 一直超时

1. 确认底盘 C++ 节点已启动：`ros2 node list` 应看到 `chassis_control_node`
2. 确认状态 Topic 存在：`ros2 topic list` 应看到 `/chassis/joint_states`
3. 确认 C++ 代码已重新编译：`colcon build --packages-select chassis_control`

### Q3: 发送运动指令后底盘不动

1. 底盘 C++ 节点必须先启动并完成初始化（日志中出现 `chassis init finished`）
2. 检查底盘参数是否正确：`ros2 param dump /chassis_control_node`
3. 底盘内部有舵向到位保护：4 个舵向角到位（误差 ≤ 0.1 rad）后才驱动轮子，初始化时需要短暂等待

### Q4: 程序崩溃后底盘继续运动

底盘 C++ 节点不会自动检测 Python 端断开。解决方法：

```python
# 方案 1: 使用 with 语句确保清理
with SVTROBOController() as robot:
    ...

# 方案 2: 手动停止底盘
ros2 topic pub --once /svtrobot_cmd geometry_msgs/msg/Twist "{linear: {x: 0, y: 0, z: 0}, angular: {z: 0}}"
```

### Q5: 多个 Python 脚本能否同时控制底盘？

不建议。多个 Publisher 写同一个 Topic 会导致指令冲突。如果需要多个模块协作，建议设计一个中心控制节点。

### Q6: `auto_stop_timeout` 应该设多少？

建议设为状态发布周期的 3~5 倍。状态发布频率为 100Hz（10ms），建议 `auto_stop_timeout=0.5`（500ms）。

### Q7: 如何在自己的项目中导入？

```python
# 方式 1: 添加路径
import sys
sys.path.append("/home/openarm/svtrobo_ws/src/chassis_control/scripts")
from svtrobo_controller import SVTROBOController

# 方式 2: 复制文件到你的项目目录
# cp svtrobo_controller.py /your/project/
from svtrobo_controller import SVTROBOController
```

### Q8: 深度学习推理应该用什么频率？

取决于模型复杂度和任务需求。建议：
- 简单策略网络：20~50Hz
- 复杂视觉模型：5~10Hz
- 底盘控制循环频率为 1kHz，但舵向响应带宽有限，10Hz 以上的推理频率即可获得良好效果

---

## 12. 交互式测试工具

项目提供了终端交互式测试脚本 `scripts/test_chassis.py`，可以在不写代码的情况下测试所有功能。

### 12.1 启动

```bash
cd /home/openarm/svtrobo_ws
source install/setup.bash
python3 src/chassis_control/scripts/test_chassis.py
```

### 12.2 功能菜单

启动后自动连接底盘并显示主菜单：

```
============================================================
  SVTROBO 测试菜单    速度=0.1 m/s  角速度=0.2 rad/s
============================================================
  1) 查询状态（单次）
  2) 实时状态监控
  3) 运动控制（定时）
  4) 连续运动（WASDQE 键盘控制）
  5) 升降机构控制
  6) 观测向量测试（深度学习）
  7) 修改默认速度
  8) 紧急停止
  0) 退出
============================================================
```

### 12.3 各功能说明

| 选项 | 功能 | 说明 |
|------|------|------|
| **1** | 查询状态 | 单次打印完整状态（舵向角度/速度/力矩/轮速/当前指令） |
| **2** | 实时监控 | 以 10Hz 持续刷新状态，按回车停止 |
| **3** | 定时运动 | 选择方向（前进/后退/左移/右移/旋转/自定义），设定持续秒数 |
| **4** | 连续运动 | WASDQE 实时键盘控制，空格停止，X 退出 |
| **5** | 升降控制 | 控制升降机构正转/反转/停止 |
| **6** | 观测向量 | 显示深度学习 16 维观测向量的详细分解 |
| **7** | 修改速度 | 调整默认线速度和角速度 |
| **8** | 紧急停止 | 立即停止底盘和升降机构 |
| **0** | 退出 | 停止底盘并关闭 |

### 12.4 连续运动模式（选项 4）键盘映射

```
  W ── 前进
  S ── 后退
  A ── 左移
  D ── 右移
  Q ── 逆时针旋转
  E ── 顺时针旋转
  空格 ── 停止
  X/Esc ── 退出
```

按下方向键立即运动，按空格或切换方向时停止。

### 12.5 安全设计

- 默认速度为 0.1 m/s（线速度）和 0.2 rad/s（角速度），确保安全
- 所有定时运动到期后自动停止
- 退出程序时自动发送停止指令
- 启用了 `auto_stop_timeout=1.0`，状态反馈中断 1 秒后自动停
