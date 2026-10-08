# arm_preset_manager

双臂（bimanual）机械臂预设姿态管理与启动 ROS2 功能包。

基于 [OpenArm](https://github.com/AiryBunny/OpenArm) 硬件接口，通过 CAN 总线驱动左右两条机械臂（各 7 自由度 + 夹爪），支持手柄 D-pad 快速切换预置姿态。

## 功能概览

- **bimanual 双臂模式**：同时驱动左右两条机械臂（right_can=can0, left_can=can1）。
- **ros2_control 硬件接口**：通过 `openarm_can` 包的 CAN-FD 驱动与电机通讯。
- **6 个 controller 自动启动**（按延时顺序）：
  - `joint_state_broadcaster` — 发布关节状态到 `/joint_states`
  - `left_forward_position_controller` — 左臂关节位置控制
  - `right_forward_position_controller` — 右臂关节位置控制
  - `left_gripper_controller` — 左夹爪控制
  - `right_gripper_controller` — 右夹爪控制
- **预设姿态管理**：通过手柄 D-pad（方向键）快速切换预设姿态，与底盘控制完全解耦。
- **文本指令备用**：通过 `/arm/preset_cmd` 话题发送文本指令控制。
- **急停保护**：支持急停锁存与解除。

## 话题列表

| 话题 | 消息类型 | 方向 | 说明 |
|------|---------|------|------|
| `/joint_states` | `sensor_msgs/JointState` | 发布 | 双臂 16 个关节状态（左臂 openarm_left_joint1~7 + finger, 右臂 openarm_right_joint1~7 + finger） |
| `/left_forward_position_controller/commands` | `Float64MultiArray` | 订阅 | 左臂关节位置指令 [j1~j7] |
| `/right_forward_position_controller/commands` | `Float64MultiArray` | 订阅 | 右臂关节位置指令 [j1~j7] |
| `/left_gripper_controller/commands` | `Float64MultiArray` | 订阅 | 左夹爪开合度 (0.0=闭合, 0.044=全开) |
| `/right_gripper_controller/commands` | `Float64MultiArray` | 订阅 | 右夹爪开合度 (0.0=闭合, 0.044=全开) |
| `/f710/joy` | `sensor_msgs/Joy` | 订阅 | 手柄原始数据（只读 D-pad axes[6]/axes[7]，不碰摇杆） |
| `/arm/preset_cmd` | `std_msgs/String` | 订阅 | 文本指令（预设名/gripper_open/gripper_close/emergency/emergency_reset） |

## 硬件连接

| 设备 | CAN 接口 | 说明 |
|------|---------|------|
| 右臂 (openarm_right) | can0 | 7 DOF + 夹爪 |
| 左臂 (openarm_left) | can1 | 7 DOF + 夹爪 |

CAN 接口需在启动前配置好（bitrate、fd-mode、up）。

## 预设姿态

预设配置文件：`config/arm_presets.yaml`

### D-pad 映射

| D-pad 方向 | 动作 |
|------------|------|
| ↑ 上 | `home` — 归零位 |
| ↓ 下 | `carry` — 搬运位 |
| ← 左 | `ready` — 准备位 |
| → 右 | `place` — 放置位 |

### 预设关节值

关节顺序：`[joint1, joint2, joint3, joint4, joint5, joint6, joint7]`

| 预设 | 关节值 (rad) | 夹爪 | 说明 |
|------|-------------|------|------|
| home | [0, 0, 0, 0, 0, 0, 0] | 闭合 | 归零位（启动后自动回到此位） |
| ready | [0.5, -0.3, 0.8, 1.2, 0, 0, 0] | 闭合 | 准备位 |
| carry | [0.2, -1.2, 1.5, 2.0, 0, 0, 0] | 全开 | 搬运位 |
| place | [0.5, -0.5, 0.5, 1.5, 0, 0.3, 0] | 闭合 | 放置位 |

关节限位参考：
- j1: [-1.40, 3.49], j2: [-1.75, 1.75], j3: [-1.57, 1.57], j4: [0, 2.44]
- j5: [-1.57, 1.57], j6: [-0.79, 0.79], j7: [-1.57, 1.57]
- 夹爪: 0.0 = 闭合, 0.044 = 全开

## 文本指令

通过 `/arm/preset_cmd` 话题发送文本指令（用于 Web 控制台或脚本）：

```bash
# 切换预设
ros2 topic pub --once /arm/preset_cmd std_msgs/String {data: home}
ros2 topic pub --once /arm/preset_cmd std_msgs/String {data: ready}

# 夹爪
ros2 topic pub --once /arm/preset_cmd std_msgs/String {data: gripper_open}
ros2 topic pub --once /arm/preset_cmd std_msgs/String {data: gripper_close}

# 急停
ros2 topic pub --once /arm/preset_cmd std_msgs/String {data: emergency}
ros2 topic pub --once /arm/preset_cmd std_msgs/String {data: emergency_reset}
```

## 依赖

- ROS2 Humble
- `openarm_description` — URDF/xacro 模型（bimanual 模式）
- `openarm_ros2` / `openarm_bringup` — ros2_control 硬件接口、controller 配置
- `openarm_can` — CAN-FD 驱动层
- `controller_manager` — ros2_control controller 管理
- `robot_state_publisher` — URDF 发布
- `sensor_msgs`, `std_msgs`, `geometry_msgs`

## 构建与运行

```bash
cd ~/svtrobo_ws
colcon build --packages-select arm_preset_manager
source install/setup.bash
```

### Launch 启动

```bash
# 默认参数（bimanual, can0+can1）
ros2 launch arm_preset_manager arm_preset_manager.launch.py

# 自定义 CAN 接口
ros2 launch arm_preset_manager arm_preset_manager.launch.py \
  right_can_interface:=can2 left_can_interface:=can3

# 无夹爪模式
ros2 launch arm_preset_manager arm_preset_manager.launch.py hand:=false
```

### Launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `right_can_interface` | `can0` | 右臂 CAN 接口 |
| `left_can_interface` | `can1` | 左臂 CAN 接口 |
| `hand` | `true` | 是否启用夹爪 |
| `joy_topic` | `/f710/joy` | 手柄原始数据话题 |
| `config_file` | (自动) | 预设配置文件路径 |

### systemd 服务

机械臂服务通过 systemd 管理：

```bash
# 查看状态
systemctl --user status svtrobo-arm

# 启动/停止
systemctl --user start svtrobo-arm
systemctl --user stop svtrobo-arm

# 查看日志
journalctl --user -u svtrobo-arm -f
```

服务配置要点：
- `ExecStart` 使用 `bash -lc` 加载 ROS2 环境
- 必须设置 `ROS_LOCALHOST_ONLY=1`（局域网有多个 ROS2 设备）

## Controller 启动时序

| 延时 | Controller | 说明 |
|------|-----------|------|
| 0s | robot_state_publisher + ros2_control_node | 硬件接口初始化 |
| 1.0s | joint_state_broadcaster | 开始发布关节状态 |
| 1.5s | left/right_forward_position_controller | 位置控制就绪 |
| 2.0s | left/right_gripper_controller | 夹爪控制就绪 |
| 5.0s | preset_manager_node | 预设管理器启动，3s 后自动回 home |

## 常见问题排查

### 1. 启动后 controller spawner 报 FATAL

```
[spawner-6] [FATAL] [controller_manager]: Available controllers:
```

- 检查 CAN 接口是否 UP：`ip link show can0`、`ip link show can1`
- 检查 `ROS_LOCALHOST_ONLY=1` 是否设置，否则 DDS 发现会失败
- 查看硬件接口日志：`journalctl --user -u svtrobo-arm --no-pager | grep -i error`

### 2. 机械臂灯不亮/红灯

- 绿灯 = 通讯正常，可控制
- 红灯 = CAN 通讯断开或未连接
- 检查 CAN 接口和线缆连接

### 3. colcon build 后 preset_manager_node 找不到

新版 setuptools 将可执行文件安装到 `bin/` 而非 `lib/`。若 `ros2 run` 找不到节点：

```bash
mkdir -p ~/svtrobo_ws/install/arm_preset_manager/lib/arm_preset_manager
ln -sf ~/svtrobo_ws/install/arm_preset_manager/bin/preset_manager_node \
  ~/svtrobo_ws/install/arm_preset_manager/lib/arm_preset_manager/preset_manager_node
```

### 4. 预设姿态不够用 / 关节值需要调整

编辑 `config/arm_presets.yaml`：

```yaml
presets:
  my_preset:
    joints: [0.1, -0.2, 0.3, 0.4, 0.0, 0.0, 0.0]
    gripper: 0.0
    description: 我的自定义姿态
```

添加后重新 `colcon build --packages-select arm_preset_manager` 即可。

### 5. 急停后如何恢复

```bash
ros2 topic pub --once /arm/preset_cmd std_msgs/String {data: emergency_reset}
```

或重启 arm 服务：`systemctl --user restart svtrobo-arm`
