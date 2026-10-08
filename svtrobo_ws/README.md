# SVTROBO - 四轮独立转向/独立驱动全向移动机器人

基于 ROS2 Humble 的 4WIS4WID 全向移动机器人控制系统，支持双臂控制、Web 浏览器、F710 手柄、Python API 多种控制方式。

## 项目结构

```
svtrobo_ws/
├── src/
│   ├── start_all.sh                                  # 一键启动全部服务
│   ├── stop_all.sh                                   # 一键停止全部服务
│   ├── arm_preset_manager/                            # 双臂控制 (Python)
│   │   ├── arm_preset_manager/preset_manager_node.py  #   预设姿态管理节点
│   │   ├── config/arm_presets.yaml                    #   预设关节值 + D-pad 映射
│   │   ├── launch/arm_preset_manager.launch.py        #   bimanual 双臂启动文件
│   │   └── scripts/start_arm.sh                       #   独立启动脚本
│   ├── chassis_control/                              # 底盘控制 (C++, 1kHz)
│   │   ├── src/                                      #   底盘/舵向/轮驱/升降/滤波 实现文件
│   │   ├── include/chassis_control/                  #   头文件
│   │   ├── config/params.yaml                        #   ROS2 参数（底盘半径、零位角度等）
│   │   ├── launch/svtrobo_bringup.launch.py          #   底盘+升降 启动文件
│   │   ├── msg/ChassisDiagnostics.msg                #   自定义诊断消息
│   │   └── scripts/                                  #   Python 控制器封装 & 测试脚本
│   ├── f710_teleop/                                  # F710 手柄遥操作
│   │   ├── f710_teleop/my_controller_node.py         #   手柄节点（X/D 模式检测）
│   │   ├── config/f710_teleop.yaml                   #   手柄参数（死区、速度上限等）
│   │   └── launch/f710_teleop.launch.py              #   启动文件
│   ├── web_control/                                  # Web 控制台 (aiohttp)
│   │   ├── server.py                                 #   Web 服务器 (8080)
│   │   ├── bag_converter.py                          #   bag (.db3) → JSONL 自动转换
│   │   ├── static/                                   #   前端（HTML/CSS/JS 模块）
│   │   └── WEB_CONTROL_GUIDE.md                      #   Web 控制台使用说明
│   ├── camera_driver/                                # 摄像头驱动 (Python)
│   │   └── camera_driver/
│   │       ├── realsense_camera.py                    #   RealSense D405 驱动
│   │       ├── realsense_node.py                      #   RealSense ROS2 节点
│   │       ├── zed_camera.py                          #   ZED 2i 驱动 (SDK + sl.Mat复用)
│   │       ├── zed_node.py                            #   ZED ROS2 节点
│   │       └── CAMERA_DRIVER_GUIDE.md                 #   摄像头驱动 API 文档
│   ├── openarm_can/                                  # OpenArm CAN-FD 驱动层
│   ├── openarm_description/                          # OpenArm URDF/xacro 模型
│   ├── linker_hand_ros2_sdk/                         # Linker Hand 灵巧手 (Python)
│   │   ├── linker_hand_ros2_sdk/linker_hand.py        #   灵巧手 ROS2 节点
│   │   ├── linker_hand_ros2_sdk/LinkerHand/           #   灵巧手 SDK API
│   │   └── launch/linker_hand.launch.py              #   灵巧手启动文件
│   └── openarm_ros2/                                 # OpenArm ros2_control 接口
├── scripts/                                          # 运维脚本
│   ├── chassis_watchdog.sh                           #   底盘节点存活监控
│   └── pcan_monitor.sh                               #   PCAN/CAN 状态监控
├── systemd/                                          # systemd service 文件备份
├── recordings/                                       # 录制数据存储（含 README 格式说明）
├── SYSTEM_CONFIG.md                                  # 系统配置文档 (11个systemd服务)
├── ROBOT_SYSTEM_GUIDE.md                             # 系统完整技术文档
└── README.md                                         # 本文件
```

## 系统架构

```
┌─────────────────────────────────────────────────────┐
│                   控制接口                            │
│  Web 浏览器 (roslibjs)  │  F710 手柄 (/dev/input/js0)  │  Python API  │
└────────────┬────────────┴──────────┬──────────────┘
             │                       │
       rosbridge (9090)         /f710/enable
             │                       │
┌────────────▼───────────────────────▼──────────────┐
│  /svtrobot_cmd (Twist)    /lift_control_cmd       │
└────────────┬──────────────────────┬──────────────┘
             │                      │
   ┌─────────▼──────────┐  ┌───────▼───────┐
   │  chassis_control    │  │  lift_control  │
   │  (C++, 1000Hz)      │  │  (RS485/Modbus)│
   └──┬───┬───┬───┬───┘  └───────────────┘
      │   │   │   │
   CAN2 转向电机  CAN3 轮电机
   0x65 0x66 0x67 0x68    ZLAC8015D ×2
   RobStride ×4

┌───────────────────────────────────────────────────┐
│               双臂控制 (bimanual)                   │
│  F710 D-pad → preset_manager_node → ros2_control  │
│                                                     │
│   CAN0 右臂 (openarm_right)    CAN1 左臂 (openarm_left)  │
│   7 DOF + 夹爪                 7 DOF + 夹爪              │
└───────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────┐
│             灵巧手 (Linker Hand O6)                │
│  /cb_*_hand_control_cmd → LinkerHand SDK → CAN    │
│                                                     │
│   CAN0 右手 (0x27)            CAN1 左手 (0x28 待接) │
│   O6 (6 DOF)                  O6 (6 DOF)            │
└───────────────────────────────────────────────────┘
```

## CAN 总线分配

| 接口 | 模式 | Bitrate | 用途 |
|------|------|---------|------|
| can0 | CAN FD | 1M/5M | 右臂 (openarm_right) + 右灵巧手 O6 (0x27) |
| can1 | CAN FD | 1M/5M | 左臂 (openarm_left) + 左灵巧手 O6 (0x28 待接) |
| can2 | CAN 2.0 | 1M | 底盘转向电机 (RobStride ×4) |
| can3 | CAN 2.0 | 1M | 底盘轮电机 (ZLAC8015D ×2) |
| can4 | CAN 2.0 | 500K | 扩展 |
| can5 | CAN 2.0 | 500K | 扩展 |

## 功能包说明

### arm_preset_manager — 双臂预设姿态管理

基于 OpenArm 硬件的 bimanual 双臂控制包，通过 CAN-FD 驱动双臂。

### linker_hand_ros2_sdk — 灵巧手控制

Linker Hand 系列灵巧手 ROS2 驱动包，通过 CAN 总线控制 O6 灵巧手（6 自由度）。
- 右手 O6 (can0, ID 0x27) — 已部署
- 左手 O6 (can1, ID 0x28) — 待接入

- **bimanual 双臂模式**：右臂 can0、左臂 can1，各 7 DOF + 夹爪
- **6 个 ros2_control controller**：joint_state_broadcaster + 左右位置控制 + 左右夹爪
- **手柄 D-pad 快速切换**预设姿态（home/ready/carry/place），与底盘控制解耦
- **文本指令备用**：通过 `/arm/preset_cmd` 话题控制
- **启动自动回 home 位**
- **急停保护**：锁存式急停，需手动解除

```
ros2 launch arm_preset_manager arm_preset_manager.launch.py
```

> 详见 [arm_preset_manager/README.md](src/arm_preset_manager/README.md)

### 传感器 — 测距

4 路 SEN0492 激光测距传感器（前/右/后/左），通过 RS485/Modbus RTU 接入 `/dev/ttyACM1`。
- 地址：前(0x51)、右(0x52)、后(0x53)、左(0x54)
- 采样率 ~5Hz，API 推送 ~10Hz
- 随 `svtrobo-web` 服务自动启动，串口打开失败自动重试
- REST: `/api/sensors/distance`，WebSocket: `/ws/distance`

### chassis_control — 底盘与升降控制

核心 C++ 控制节点，1kHz 控制循环。

- **4 轮独立转向**：4 个 RobStride 电机（CAN2），闭环角度控制
- **4 轮独立驱动**：2 个 ZLAC8015D 驱动器（CAN3），左右轮各控一对
- **升降机构**：RS485/Modbus 控制
- **指令超时保护**：0.5s 无新指令自动归零
- **诊断发布**：电压、温度、错误码、轮速（100Hz）

```
ros2 launch chassis_control svtrobo_bringup.launch.py
```

### f710_teleop — F710 手柄遥操作

Python ROS2 节点，直接读取 `/dev/input/jsX` 设备。

- **手柄模式：D 模式（DirectInput）**，不是 X 模式
- **X/D 模式自动检测**：读取 sysfs 设备名称判断手柄模式，X 模式自动禁用控制并前端警告
- 左摇杆(axes4/5)控制前后/平移，右摇杆(axes2)控制转向
- A 解锁使能，B 急停，LB/RB 升降，LT/RT 加减速
- 40% 死区 + 低通滤波 + 加速度限制，运动平滑
- 发布原始手柄数据到 `/f710/joy`，支持录制
- Web 前端可通过 `/f710/enable` 远程启停
- 发布模式状态到 `/f710/mode`（"X"/"D"/"unknown"）

```
ros2 launch f710_teleop f710_teleop.launch.py
```

### web_control — Web 控制台

基于 aiohttp + roslibjs 的浏览器控制界面。

- 实时 MJPEG 摄像头画面（D405 ×2, ZED 2i）
- WASD/Q/E 键盘控制底盘
- 升降控制、电机状态、电池/温度诊断
- 手柄/Web 模式一键切换（默认手柄模式）
- X 模式手柄自动检测与前端警告横幅
- 数据采集：彩色图(13.5fps) + 深度图 + 点云(~1.3Hz) + IMU(~70Hz) + ROS2 bag
- **录制结束后自动转换**：bag (.db3) → JSONL 格式，方便深度学习训练
- IMU 实时 WebSocket 推送 (/ws/imu) + HTTP 回退 (/api/imu)
- 测距传感器 4 路 SEN0492 激光测距（前/右/后/左），REST + WebSocket 实时推送
- ROS 连接断开时统一清理所有模块状态（摄像头、底盘、诊断面板归位）

```
python3 src/web_control/server.py          # Web 服务 (8080)
ros2 launch rosbridge_server rosbridge_websocket_launch.py  # rosbridge (9090)
```

**测距传感器 API：**

| 端点 | 方法 | 说明 |
|------|------|------|
| `/api/sensors/distance` | GET | 4 路测距数据（mm）|
| `/ws/distance` | WebSocket | 测距实时推送 ~10Hz |

> 4 路 SEN0492 激光测距（RS485/Modbus RTU，/dev/ttyACM1），随 web 服务自动启动。

### camera_driver — 摄像头驱动

支持 RealSense D405 和 ZED 2i 的 Python 采集模块。

- **ZED SDK 模式**：HD720 彩色图 + NEURAL 深度 + XYZRGBA 点云 + IMU (~70Hz)
- **RealSense D405**：彩色图 + 深度图，快速设备检测避免阻塞
- sl.Mat 对象复用，减少每帧 C++ 堆分配开销
- 相机内参获取、点云全分辨率 float16
- IMU 独立线程读取，前端 WebSocket 实时推送

## systemd 服务 (12个)

| 服务 | 说明 | 端口 |
|------|------|------|
| f710-fix.service | F710 手柄驱动修复 (oneshot) | - |
| svtrobo-can.service | CAN 接口配置 (oneshot) | - |
| svtrobo-rosbridge.service | Rosbridge WebSocket | 9090 |
| svtrobo-arm.service | 双臂控制 (bimanual) | - |
| svtrobo-chassis.service | 底盘+升降控制 | - |
| svtrobo-chassis-watchdog.service | 底盘节点存活监控 | - |
| svtrobo-f710.service | F710 手柄遥操作 | - |
| svtrobo-linker-hand.service | Linker Hand 灵巧手控制 (左+右) | - |
| svtrobo-motion-player.service | 机械臂动作回放节点 | - |
| svtrobo-web.service | Web 控制台 + 测距传感器 | 8080 |
| svtrobo-nodeapi.service | Node.js API | 28181 |
| pcan-monitor.service | PCAN/CAN 状态监控 | - |

> 详见 [SYSTEM_CONFIG.md](SYSTEM_CONFIG.md)

## 快速启动

### 一键启动全部服务

```bash
cd ~/svtrobo_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
bash src/start_all.sh
```

启动内容：底盘控制 → rosbridge → Web 服务。手柄设备 `/dev/input/js0` 检测到则自动启动手柄节点。

### 一键停止

```bash
bash src/stop_all.sh
```

### 单独启动

```bash
# 仅双臂
ros2 launch arm_preset_manager arm_preset_manager.launch.py

# 仅灵巧手（右手 O6）
ros2 launch linker_hand_ros2_sdk linker_hand.launch.py

# 仅底盘 + 升降
ros2 launch chassis_control svtrobo_bringup.launch.py

# 仅手柄
ros2 launch f710_teleop f710_teleop.launch.py

# Web 控制（需先启动 rosbridge）
ros2 launch rosbridge_server rosbridge_websocket_launch.py
python3 src/web_control/server.py
```

### 从源码构建

```bash
cd ~/svtrobo_ws
source /opt/ros/humble/setup.bash
colcon build --base-paths src
source install/setup.bash
```

## ROS2 话题

### 底盘与手柄

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/svtrobot_cmd` | geometry_msgs/Twist | 控制 → 底盘 | 线速度 x/y + 角速度 z |
| `/chassis/cmd_feedback` | geometry_msgs/Twist | 底盘 → 外部 | 速度反馈 (100Hz) |
| `/lift_control_cmd` | std_msgs/Int32MultiArray | 控制 → 升降 | [方向, 速度] |
| `/f710/joy` | sensor_msgs/Joy | 手柄 → 外部 | 原始摇杆/按钮状态 |
| `/f710/enable` | std_msgs/Bool | Web → 手柄 | 启停手柄控制 |
| `/f710/status` | std_msgs/Bool | 手柄 → Web | 手柄使能状态 |
| `/f710/mode` | std_msgs/String | 手柄 → Web | 手柄模式检测："X"/"D"/"unknown" |
| `/chassis/joint_states` | sensor_msgs/JointState | 底盘 → 外部 | 电机位置/速度/力矩 (100Hz) |
| `/chassis/diagnostics` | ChassisDiagnostics | 底盘 → 外部 | 电压/温度/错误码 (100Hz) |

### 双臂控制

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/joint_states` | sensor_msgs/JointState | 双臂 → 外部 | 16 个关节状态（左右各 7+joint + finger） |
| `/left_forward_position_controller/commands` | Float64MultiArray | 控制 → 左臂 | 左臂关节位置 [j1~j7] |
| `/right_forward_position_controller/commands` | Float64MultiArray | 控制 → 右臂 | 右臂关节位置 [j1~j7] |
| `/left_gripper_controller/commands` | Float64MultiArray | 控制 → 左夹爪 | 0.0=闭合, 0.044=全开 |
| `/right_gripper_controller/commands` | Float64MultiArray | 控制 → 右夹爪 | 0.0=闭合, 0.044=全开 |
| `/arm/preset_cmd` | std_msgs/String | 外部 → 双臂 | 文本指令（预设名/gripper_open/close/emergency） |

### 灵巧手

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/cb_right_hand_control_cmd` | sensor_msgs/JointState | 控制 → 右手 | 右手手指位置/速度指令 (O6, 6 DOF) |
| `/cb_left_hand_control_cmd` | sensor_msgs/JointState | 控制 → 左手 | 左手手指位置/速度指令 (O6, 6 DOF, 待接) |
| `/cb_right_hand_state` | sensor_msgs/JointState | 右手 → 外部 | 右手关节状态 (~60 Hz) |
| `/cb_left_hand_state` | sensor_msgs/JointState | 左手 → 外部 | 左手关节状态 (~60 Hz, 待接) |
| `/cb_right_hand_info` | std_msgs/String (JSON) | 右手 → 外部 | 右手信息（版本/速度/电流/温度/力矩） |
| `/cb_hand_setting_cmd` | std_msgs/String (JSON) | 外部 → 手 | 设置指令（速度/力矩/清故障） |

## 数据录制

Web 控制台右下角录制按钮或 F710 手柄 X/Y 按钮，自动采集：

| 数据 | 格式 | 频率 | 说明 |
|------|------|------|------|
| ZED 彩色图 | JPEG | 2 fps | 1280x720, ~49KB/帧 |
| ZED 深度图 | JPEG (JET colormap) | 2 fps | 归一化着色后保存 |
| ZED 点云 | npz | ~2 fps | 全分辨率(720x1280) float16, ~7MB/帧 |
| IMU | imu.jsonl | ~15 Hz | accel/gyro/mag/pressure/temp |
| ROS2 bag | db3 | 原始频率 | 5个话题，录制结束自动转JSONL |
| 录制摘要 | summary.json | - | 时长/帧数/大小 |

> **前端只显示彩色流**，深度/点云数据仅在后台录制保存。每小时约 5GB。

保存路径：`/svtrobo_data/recordings/<日期>/<时间>/`（`~/svtrobo_ws/recordings` 为符号链接）

```
recordings/YYYY-MM-DD/HHMMSS/
├── images/zed/                  # ZED 左眼彩色图 JPEG (2fps)
├── images/zed_right/             # ZED 右眼彩色图 JPEG (2fps)
├── depth/zed/                   # 深度图 JET colormap JPEG (2fps)
├── pointcloud/zed/              # 点云 npz (全分辨率 float16, ~2fps)
├── rosbag/                      # ROS2 bag (5个话题)
├── imu.jsonl                    # IMU数据 (~70Hz)
├── summary.json                 # 录制摘要
├── chassis_joint_states.jsonl   # 8 关节状态 (~10Hz)
├── chassis_diagnostics.jsonl    # 电压/温度/错误码/轮速 (~10Hz)
├── svtrobot_cmd.jsonl           # 底盘速度指令 (~50Hz)
├── f710_joy.jsonl               # 手柄原始数据 (~50Hz)
└── lift_control_cmd.jsonl       # 升降控制指令 (~25Hz)
```

> 所有 JSONL 记录均包含 `_timestamp_ns` 字段，可用于多话题时间对齐。详见 [recordings/README.md](recordings/README.md)。

## 文档索引

| 文档 | 内容 |
|------|------|
| [ROBOT_SYSTEM_GUIDE.md](ROBOT_SYSTEM_GUIDE.md) | 系统完整技术文档 |
| [SYSTEM_CONFIG.md](SYSTEM_CONFIG.md) | 系统配置文档 (10个 systemd 服务) |
| [recordings/README.md](recordings/README.md) | 录制数据格式与 JSONL 字段说明 |
| [src/arm_preset_manager/README.md](src/arm_preset_manager/README.md) | 双臂控制使用文档 |
| [src/chassis_control/PYTHON_API_GUIDE.md](src/chassis_control/PYTHON_API_GUIDE.md) | Python 控制接口文档 |
| [src/web_control/WEB_CONTROL_GUIDE.md](src/web_control/WEB_CONTROL_GUIDE.md) | Web 控制台使用说明 |
| [src/camera_driver/CAMERA_DRIVER_GUIDE.md](src/camera_driver/CAMERA_DRIVER_GUIDE.md) | 摄像头驱动 API 文档 |
| [src/f710_teleop/手柄操作指导说明.md](src/f710_teleop/手柄操作指导说明.md) | 手柄按键说明 |
| [src/linker_hand_ros2_sdk/README.md](src/linker_hand_ros2_sdk/README.md) | 灵巧手控制使用文档 |
