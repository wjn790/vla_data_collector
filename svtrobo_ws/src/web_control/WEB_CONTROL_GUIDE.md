# SVTROBO Web 控制台使用文档

> 基于 roslibjs + aiohttp 的浏览器端机器人控制面板

---

## 目录

1. [系统概述](#1-系统概述)
2. [架构说明](#2-架构说明)
3. [前置条件](#3-前置条件)
4. [快速启动](#4-快速启动)
5. [功能模块](#5-功能模块)
6. [ROS Topic 通信映射](#6-ros-topic-通信映射)
7. [相机画面](#7-相机画面)
8. [底盘键盘控制](#8-底盘键盘控制)
9. [升降机构控制](#9-升降机构控制)
10. [电机状态面板](#10-电机状态面板)
11. [电池与诊断面板](#11-电池与诊断面板)
12. [文件结构](#12-文件结构)
13. [常见问题与排错](#13-常见问题与排错)

---

## 1. 系统概述

Web 控制台提供以下功能：

- **相机画面**：实时 MJPEG 视频流（D405 #1、D405 #2、ZED 2i）
- **底盘控制**：浏览器端 WASD/Q/E 键盘控制，支持速度调节
- **升降机构**：竖向滑条控制升降速度，上升/下降/停止按钮
- **电机状态**：实时显示 4 个舵向电机的角度/速度/力矩及轮速（目标 vs 实际）
- **测距传感器**：4 路 SEN0492 激光测距（前/右/后/左），REST API + WebSocket 实时推送
- **电池与诊断**：总线电压（VBUS）、电量估算、电机温度、错误码

所有底盘控制通过 **rosbridge WebSocket** 转发 ROS2 Topic 消息，无需在机器人上安装桌面环境。

---

## 2. 架构说明

```
┌──────────────────────────────────────────────────────────────┐
│                       浏览器                                 │
│                                                              │
│  index.html + JS 模块（app / chassis / chassis-status /      │
│  camera / diagnostics / lift / status）                      │
│                                                              │
│         │ ws://host:9090              │ http://host:8080     │
│         │ (roslibjs)                  │ (MJPEG / REST)      │
└─────────┼──────────────────────────────┼─────────────────────┘
          │                              │
          ▼                              ▼
┌─────────────────┐          ┌──────────────────────────┐
│ rosbridge_server │          │  aiohttp Web Server       │
│  (port 9090)     │          │  (server.py, port 8080)   │
│                  │          │                            │
│  ROS2 ↔ WebSocket│          │  - 静态文件托管             │
│  协议转换         │          │  - 相机 MJPEG 流           │
└────────┬─────────┘          │  - 相机启停 REST API       │
         │                     └────────────┬──────────────┘
         │ ROS2 DDS                        │ camera_driver
         ▼                                  ▼
┌──────────────────────────────────────────────────┐
│              ROS2 节点 (底盘 C++ 节点)              │
│                                                    │
│  /chassis/joint_states  (100Hz)                    │
│  /chassis/cmd_feedback  (100Hz)                    │
│  /chassis/diagnostics   (100Hz)                    │
│  /svtrobot_cmd         (控制指令)                   │
│  /lift_control_cmd     (升降控制)                   │
└──────────────────────────────────────────────────┘
```

**两个服务：**

| 服务 | 端口 | 功能 |
|------|------|------|
| `rosbridge_server` | 9090 | ROS2 ↔ WebSocket 协议转换，传输 Topic 消息 |
| `aiohttp Web Server` | 8080 | 静态文件托管、相机 MJPEG 流、相机启停 REST API |

---

## 3. 前置条件

### 依赖

```bash
# ROS2 环境
source /opt/ros/humble/setup.bash

# rosbridge_server
sudo apt install ros-humble-rosbridge-server

# Python 依赖
pip3 install aiohttp opencv-python numpy

# 相机驱动（如需使用相机功能）
pip3 install pyrealsense2
```

### 底盘节点

Web 控制台依赖底盘 C++ 节点发布的状态数据：

```bash
cd /home/svt/svtrobo_ws
colcon build --packages-select chassis_control
source install/setup.bash
ros2 launch chassis_control svtrobo_bringup.launch.py
```

---

## 4. 快速启动

### 方式一：一键启动（推荐）

```bash
cd /home/svt/svtrobo_ws/web_control
bash start_web.sh
```

该脚本自动启动 rosbridge_server 和 aiohttp Web 服务器。

### 方式二：手动启动

```bash
# 终端 1：启动 rosbridge_server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090

# 终端 2：启动 Web 服务器
cd /home/svt/svtrobo_ws/src/web_control
python3 server.py --host 0.0.0.0 --port 8080
```

### 访问控制台

在浏览器中打开：

```
http://<机器人IP>:8080
```

页面顶部输入 rosbridge 地址（如 `ws://localhost:9090`），点击 **连接** 按钮。

> 如果浏览器和机器人不在同一台机器上，将 `localhost` 替换为机器人的实际 IP 地址。

---

## 5. 功能模块

### 模块概览

| JS 模块 | 文件 | 功能 |
|---------|------|------|
| App | `app.js` | rosbridge 连接管理，模块初始化调度 |
| Chassis | `chassis.js` | WASD/Q/E 键盘控制，速度滑条，指令发布 |
| ChassisStatus | `chassis-status.js` | 舵向电机角度/速度/力矩 + 轮速实时显示 |
| Camera | `camera.js` | 相机启停控制，MJPEG 流显示 |
| Diagnostics | `diagnostics.js` | VBUS 电压、电量、电机温度、错误码 |
| Lift | `lift.js` | 升降机构速度滑条 + 方向按钮 |
| StatusMonitor | `status.js` | ROS Topic 列表显示 |
| ControlMode | `f710-toggle.js` | 手柄/Web 模式互斥切换、X/D 模式检测与警告 |
| DataRecord | `data-record.js` | 数据录制核心逻辑（bag + 摄像头帧 + JSONL）；常驻低频轮询 `/recording/status`，手柄/后端外部启动时也能同步前端 |
| RecPanel | `rec-panel.js` | 录制面板 v3c：按钮触发弹窗式面板，ROS topic 计数（SQLite），停止后保留面板，后端 running 时强制清理上一轮状态 |
| MasterLock | `master-lock.js` | 主控锁：手柄/Web 模式互斥切换 |
| HardwareStatus | `hardware-status.js` | 硬件在线检测（不依赖 ROS 连接） |
| ImuStatus | `imu-status.js` | IMU 实时数据 WebSocket 显示 |
| DistanceSensor | `distance-sensor.js` | 测距传感器 HTTP polling 显示 |
| JointDisplay | `joint-display.js` | 机械臂关节角度实时显示 |

---

### 5.1 数据录制状态同步约定

录制状态以后端 `/recording/status` 为准，不能依赖 rosbridge 连接或 MasterLock 主控状态：

- `DataRecord` 在页面初始化后常驻低频轮询 `/recording/status`（当前 5 秒一次），避免给 Jetson/Web 服务增加额外负载。
- 录制目录按日期分类，叶子目录命名为 `<三位序号>-<时间>`（例如 `2026-06-12/003-110530`）。三位序号放在时间前面，用来避免小车未联网时同一天内时间不准、联网校时后文件夹按时间排序混乱。测试录制位于 `_test/<日期>/` 下并使用独立序号。
- ROS 断开、页面只读、MasterLock 被其它页面抢占时，只禁止本页主动开始/停止录制；仍应继续显示后端正在录制的状态。
- 一旦后端返回 `running: true`，前端必须同步为“录制中”，并可弹出/刷新 `RecPanel`。
- `RecPanel` 发现后端 `running: true` 时要清理上一轮 `recordingDone` / `abortDetected` 状态，避免上一轮完成态阻塞新一轮显示。

---

## 6. ROS Topic 通信映射

Web 控制台通过 rosbridge WebSocket 订阅和发布以下 ROS2 Topic：

| Web 模块 | ROS Topic | 消息类型 | 方向 | 用途 |
|----------|-----------|---------|------|------|
| Chassis | `/svtrobot_cmd` | `geometry_msgs/Twist` | 发布 | 底盘速度指令 |
| Chassis | `/chassis/cmd_feedback` | `geometry_msgs/Twist` | 订阅 | 速度反馈显示 |
| ChassisStatus | `/chassis/joint_states` | `sensor_msgs/JointState` | 订阅 | 舵向角度/速度/力矩 + 轮速目标值 |
| ChassisStatus | `/chassis/diagnostics` | `chassis_control/msg/ChassisDiagnostics` | 订阅 | ZLAC8015D 实际轮速 |
| Diagnostics | `/chassis/diagnostics` | `chassis_control/msg/ChassisDiagnostics` | 订阅 | VBUS、温度、错误码 |
| Lift | `/lift_control_cmd` | `std_msgs/Int32MultiArray` | 发布 | 升降控制指令 |
| ControlMode | `/f710/enable` | `std_msgs/Bool` | 发布 | 启停手柄控制 |
| ControlMode | `/f710/status` | `std_msgs/Bool` | 订阅 | 手柄使能状态 |
| ControlMode | `/f710/mode` | `std_msgs/String` | 订阅 | 手柄模式检测 "X"/"D"/"unknown" |
| DataRecord | `/f710/joy` | `sensor_msgs/Joy` | - | 手柄数据录制（通过 ros2 bag） |

---

## 7. 相机画面

### 支持的相机

| 名称 | 类型 | 分辨率 | FPS |
|------|------|--------|-----|
| D405 #1 | RealSense | 1280x720 | 5 |
| D405 #2 | RealSense | 1280x720 | 5 |
| ZED 2i | ZED SDK | 1280x720 (HD720) | 15 |

### 操作方式

1. 在"相机画面"区域，点击对应相机的 **启动** 按钮
2. Web 后端通过 `camera_driver` 模块启动相机采集线程
3. 画面通过 MJPEG 流 (`/camera/{name}`) 实时推送到浏览器
4. 点击 **停止** 按钮关闭相机并释放资源

### REST API

| 端点 | 方法 | 说明 |
|------|------|------|
| `/camera/{name}` | GET | MJPEG 视频流（name: `d405_1` / `d405_2` / `zed`） |
| `/camera/start` | POST | 启动相机（body: `{"camera": "d405_1"}`） |
| `/camera/stop` | POST | 停止相机（body: `{"camera": "d405_1"}`） |
| `/camera/status` | GET | 获取所有相机状态 |

### 主控锁 API

Web 和手柄模式互斥，确保同一时间只有一个控制源。

| 端点 | 方法 | 说明 |
|------|------|------|
| `/api/master/request` | POST | 请求主控锁（body: `{"source": "web"}`） |
| `/api/master/release` | POST | 释放主控锁 |
| `/api/master/status` | GET | 查询当前主控锁状态 |

> 前端每次请求时自动附带 master request，确保控制权归属。

### 其他端点

| 端点 | 方法 | 说明 |
|------|------|------|
| `/api/exit-kiosk` | POST | 退出 Firefox kiosk 全屏模式 |
| `/ws` | WebSocket | rosbridge WebSocket 代理（转发到 localhost:9090） |

> 相机采集线程以后台守护线程运行，帧通过有界队列传递，MJPEG 编码质量为 95。

### 数据录制

| 端点 | 方法 | 说明 |
|------|------|------|
| `/recording/start` | POST | 开始录制（bag + 摄像头帧） |
| `/recording/stop` | POST | 停止录制并自动转换 JSONL |
| `/recording/status` | GET | 获取录制状态（running, elapsed, path） |

> **自动相机管理**：调用 `/recording/start` 开始录制时，系统会自动启动所有未运行的相机（d405_1/d405_2/zed）；调用 `/recording/stop` 停止录制时，系统会自动关闭由录制启动的相机（此前已运行的相机不会被关闭）。

#### F710 手柄采集控制

除了通过 Web 控制台按钮触发录制外，还可以使用 F710 手柄控制采集：

| 按钮 | 动作 | 实现方式 |
|------|------|----------|
| **X** 按钮 | 开始采集 | 通过 HTTP 调用 `/recording/start` |
| **Y** 按钮 | 停止采集 | 通过 HTTP 调用 `/recording/stop` |

手柄按钮由 `f710_teleop` 节点处理，当检测到 X/Y 按钮按下时，自动通过 HTTP 请求调用 Web 服务器的录制 API，实现与 Web 控制台按钮完全一致的采集流程（包括自动相机启停管理）。

### F710 手柄控制

| 端点 | 方法 | 说明 |
|------|------|------|
| `/f710/start` | POST | 启动 F710 手柄节点 |
| `/f710/stop` | POST | 停止 F710 手柄节点 |
| `/f710/status` | GET | 获取 F710 节点运行状态 |

---

## 8. 底盘键盘控制

### 操作方式

1. 鼠标点击"底盘与升降控制"面板区域获取焦点
2. 页面显示"点击此处启用键盘控制"提示消失后即可操控
3. 按下方向键立即运动，松开或按空格停止

### 键盘映射

```
  W ── 前进 (vx > 0)
  S ── 后退 (vx < 0)
  A ── 左移 (vy > 0)
  D ── 右移 (vy < 0)
  Q ── 逆时针旋转 (wz > 0)
  E ── 顺时针旋转 (wz < 0)
  空格 ── 停止
```

支持组合键（如 W+A 斜向前进左移、W+Q 弧线运动），合成速度会自动归一化。

### 速度控制

速度滑条范围：**0.05 ~ 0.50 m/s**，默认 0.30 m/s。旋转速度固定为 0.50 rad/s。

### 发布频率

键盘按下期间以 **20Hz**（50ms 间隔）持续发布速度指令，确保底盘持续运动。松开所有键后发送零速指令。

### 安全机制

- 面板失焦（点击其他区域）时自动停止底盘
- 键盘松开时发送零速指令
- 底盘内部有舵向到位保护和轮速上限机制

---

## 9. 升降机构控制

### 操作方式

控制面板位于底盘控制右侧，包含：

- **速度滑条**：0 ~ 500 RPM，默认 300 RPM
- **上升按钮**：正转，以滑条设定速度运行
- **下降按钮**：反转，以滑条设定速度运行
- **停止按钮**：断开使能

### ROS Topic

通过 `/lift_control_cmd`（`std_msgs/msg/Int32MultiArray`）发送控制指令：

```javascript
// 上升
msg.data = [1, speed]    // direction=1, speed=RPM

// 下降
msg.data = [-1, speed]   // direction=-1

// 停止
msg.data = [0, 0]        // direction=0
```

---

## 10. 电机状态面板

### 显示内容

**舵向电机表：**

| 列 | 数据来源 | 单位 |
|----|---------|------|
| 角度 | `/chassis/joint_states` position[0..3] | 度（从 rad 转换） |
| 速度 | `/chassis/joint_states` velocity[0..3] | rad/s |
| 力矩 | `/chassis/joint_states` effort[0..3] | Nm |

**轮速表：**

| 列 | 数据来源 | 单位 |
|----|---------|------|
| 目标 | `/chassis/joint_states` velocity[4..7] | RPM |
| 实际 | `/chassis/diagnostics` wheel_speeds_actual | RPM |

> 实际轮速从 ZLAC8015D 驱动器读取，约每 10 秒更新一次。当轮速 > 1 RPM 时，实际值显示为蓝色高亮。

---

## 11. 电池与诊断面板

### 电池状态

| 显示项 | 数据来源 | 说明 |
|--------|---------|------|
| 电压 | `diagnostics.vbus` | RobStride 电机 VBUS 读取（约每 20s 更新） |
| 电量 | `diagnostics.vbus` | 基于 6S LiPo (19.8V~25.2V) 线性估算 |
| 温度 | `diagnostics.motor_temperatures` | 4 个电机中的最高温度 |

电量颜色指示：
- > 50%：绿色
- 20%~50%：黄色
- < 20%：红色

### 电机诊断

每个舵向电机显示温度和错误码：

| 显示项 | 颜色规则 |
|--------|---------|
| 温度 > 70°C | 红色 |
| 温度 > 50°C | 黄色 |
| 温度 ≤ 50°C | 默认色 |
| 错误码 = 0 | 显示 "OK" |
| 错误码 ≠ 0 | 显示 "ERR:0xXX" |

> **注意**：rosbridge 将 `uint8[]` 类型编码为 base64 字符串，Web 端已做自动解码处理。

---

## 12. 文件结构

```
web_control/
├── server.py                  # aiohttp Web 服务器（静态文件 + 相机流 + REST API）
├── bag_converter.py            # bag (.db3) → JSONL 自动转换（录制结束后后台执行）
├── start_web.sh               # 一键启动脚本（rosbridge + aiohttp）
└── static/
    ├── index.html             # 控制台主页面
    ├── css/
    │   ├── style.css          # 主样式表
│   ├── distance-sensor.css # 测距传感器面板样式
│   ├── hardware-status.css # 硬件状态面板样式
│   ├── joint-display.css   # 关节显示样式
│   └── rec-panel.css       # 录制面板样式
    └── js/
        ├── roslib.min.js      # roslibjs 库（WebSocket ROS 通信）
        ├── app.js             # 主应用（rosbridge 连接管理、模块初始化）
        ├── chassis.js         # 底盘键盘控制 + 速度滑条
        ├── chassis-status.js  # 舵向电机角度/速度/力矩 + 轮速显示
        ├── camera.js          # 相机启停控制 + MJPEG 显示
        ├── diagnostics.js     # VBUS/温度/错误码诊断面板
        ├── f710-toggle.js     # 手柄/Web 模式互斥切换 + X/D 模式警告
        ├── data-record.js     # 数据录制核心逻辑（bag + 摄像头帧 + JSONL 转换）
        ├── rec-panel.js       # 录制面板 v3c（按钮触发弹窗，ROS topic 计数，SQLite 统计）
        ├── master-lock.js     # 主控锁（手柄/Web 模式互斥切换）
        ├── hardware-status.js # 硬件在线状态检测（ZED/D405/IMU/底盘/手柄）
        ├── imu-status.js      # IMU 实时数据 WebSocket 显示（ws/imu + HTTP 回退）
        ├── distance-sensor.js # 测距传感器 HTTP polling 显示（REST /api/sensors/distance）
        ├── joint-display.js   # 机械臂关节角度实时显示
        ├── lift.js            # 升降机构控制
        └── status.js          # ROS Topic 列表显示
```

---

## 13. 常见问题与排错

### IMU 数据接口

Web 控制台提供 IMU 数据的实时查看接口（ZED 2i 内置 IMU）：

| 端点 | 方法 | 说明 |
|------|------|------|
| `/api/imu` | GET | IMU 当前数据 JSON（accel, gyro_dps, gyro_rad, mag, imu_temp, pressure, env_temp） |
| `/ws/imu` | WebSocket | IMU 实时推送 ~20Hz（前端优先使用 WebSocket，自动回退 HTTP polling） |

### 测距传感器接口

4 路 SEN0492 激光测距传感器，通过 RS485/Modbus RTU 接入 RainbowLink（/dev/ttyACM1），传感器地址 0x51~0x54。

**硬件参数：**
- 型号：SEN0492 激光测距传感器
- 接口：RS485 → RainbowLink USB 转换器（/dev/ttyACM1）
- 协议：Modbus RTU，波特率 115200
- 地址：前(0x51)、右(0x52)、后(0x53)、左(0x54)
- 寄存器：0x34（距离值，单位 mm）
- 采样率：约5Hz（后台 daemon 线程轮询）
- 开机自启：随 svtrobo-web 服务启动，串口打开失败自动重试（每 5 秒，最多 30 次）

**API 端点：**

| 端点 | 方法 | 说明 |
|------|------|------|
| `/api/sensors/distance` | GET | 测距当前数据 JSON |
| `/ws/distance` | WebSocket | 测距实时推送 约10Hz |

**REST 响应格式：**

```json
{
  "ok": true,
  "data": {
    "front": 169,
    "right": 154,
    "rear": 432,
    "left": 235,
    "unit": "mm"
  },
  "timestamp": 1779934254.321
}
```

- `ok`: true 表示至少一个传感器在线
- 各方向值：距离（mm），null 表示该传感器离线
- `timestamp`: Unix 时间戳（秒）

**WebSocket 推送格式与 REST 响应一致**，以约10Hz 频率推送，前端优先使用 WebSocket，可回退到 HTTP polling。

> 传感器线程随 svtrobo-web 服务自动启动（After=svtrobo-can.service），无需单独配置。

### 录制数据内容

录制时自动采集以下数据：

| 数据 | 格式 | 频率 | 说明 |
|------|------|------|------|
| ZED 左眼彩色 | JPEG q95 (images/zed/) | 2 Hz | 1280x720, deadline-based |
| ZED 右眼彩色 | JPEG q95 (images/zed_right/) | 2 Hz | 1280x720, 与左眼同步 |
| D405 彩色图 | JPEG q95 (images/d405_1/, d405_2/) | 2 Hz | 1280x720 |
| ZED 深度图 | JPEG q95 JET colormap (depth/zed/) | 2 Hz | 1280x720, 0-20m归一化 |
| D405 深度图 | JPEG q95 JET colormap (depth/d405_*/) | 2 Hz | 1280x720 |
| ZED 点云 | npz float16 (pointcloud/zed/) | 2 Hz | XYZRGBA, 全分辨率1280x720, ~7MB/帧 |
| IMU | imu.jsonl | ~15 Hz (native grab rate) | accel/gyro/mag/pressure/temp |
| ROS2 bag | rosbag/*.db3 | 原始频率 | 5个话题 |
| 录制摘要 | summary.json | - | 时长/帧数/大小 |
| JSONL传感器 | *.jsonl | 10-50 Hz | bag自动转换 |

> 详细数据格式见 `recordings/README.md`。

### Q1: 连接 rosbridge 失败

1. 确认 rosbridge_server 已启动：`ros2 node list` 应看到 `/rosbridge_websocket_server`
2. 检查端口是否正确（默认 9090）：`netstat -tlnp | grep 9090`
3. 如果浏览器不在机器人本机，将 `localhost` 替换为机器人 IP
4. 检查防火墙是否放行了 9090 端口

### Q2: 底盘控制无响应

1. 确认底盘 C++ 节点已启动并完成初始化
2. 点击控制面板获取焦点（提示文字消失）
3. 检查 rosbridge 连接状态是否为"已连接"
4. 终端运行 `ros2 topic echo /svtrobot_cmd` 确认指令是否发出

### Q3: 相机画面无法显示

1. 确认相机已连接：`rs-enumerate-devices --short`（RealSense）或 `ls /dev/video*`（ZED）
2. 确认 `pyrealsense2` 已安装：`pip3 show pyrealsense2`
3. 点击"启动"按钮后等待 1~2 秒（相机预热需要时间）
4. 检查 Web 服务器日志中的错误信息

### Q4: 电机状态面板显示 "--"

1. 确认底盘 C++ 节点正在运行
2. 确认 rosbridge 已连接
3. 检查 Topic 是否发布：`ros2 topic hz /chassis/joint_states`

### Q5: 电压/温度显示 "--"

- VBUS 约每 20 秒更新一次，首次读取需要等待
- 电机温度在 `motor_temperatures` 中提供，确保 `ChassisDiagnostics` 消息正常发布
- 检查：`ros2 topic echo /chassis/diagnostics --once`

### Q6: 错误码显示异常（非数字字符）

rosbridge 对 `uint8[]` 使用 base64 编码，Web 端已做解码处理。如仍显示异常，检查 `diagnostics.js` 中的 base64 解码逻辑。

### Q7: 如何从外部网络访问？

```bash
# 确保防火墙放行端口
sudo ufw allow 8080
sudo ufw allow 9090

# 启动时绑定所有接口
python3 server.py --host 0.0.0.0 --port 8080
```

然后在浏览器访问 `http://<机器人IP>:8080`，rosbridge 地址填写 `ws://<机器人IP>:9090`。

### Q8: 如何自定义相机配置？

编辑 `server.py` 中的 `CAMERA_CONFIG` 字典：

```python
CAMERA_CONFIG = {
    'd405_1': {'type': 'realsense', 'serial': '409122272399', 'size': (1280, 720), 'fps': 6, 'depth': True},
    'd405_2': {'type': 'realsense', 'serial': '409122273344', 'size': (1280, 720), 'fps': 6, 'depth': True},
    'zed':    {'type': 'zed', 'resolution': 'HD720', 'fps': 15, 'depth': True},
}
JPEG_QUALITY = 95
PC_DOWNSAMPLE = 1      # 点云降采样 (1=全分辨率, 2=半)
PC_DTYPE = 'float16'   # 点云精度 ('float16' 或 'float32')
```

> **前端只显示彩色流**，深度数据仅在录制时后台保存。

修改后重启 Web 服务器生效。

