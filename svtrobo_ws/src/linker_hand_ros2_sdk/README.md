# linker_hand_ros2_sdk — Linker Hand 灵巧手 ROS2 驱动

基于 [Linker Hand SDK](https://github.com/linker-bot/linkerhand-ros2-sdk) 的灵巧手 ROS2 驱动包，通过 CAN 总线控制 Linker Hand 系列灵巧手，支持多种型号（O6/L6/L6P/L7/L10/L20/L21/L25/G20）。

## 功能概览

- **多型号支持**：O6（6自由度）、L6/L6P（6自由度）、L7（7自由度）、L10（10自由度）、L20/L21/L25、G20（工业版）等
- **手指位置控制**：通过 JointState 话题发送手指位置指令
- **手指速度控制**：支持逐关节速度设定
- **力矩/电流/温度监控**：实时获取手部状态信息
- **触觉传感器**：支持单点压力传感器和矩阵压力传感器
- **自动初始化**：启动时自动设置速度、力矩并归零

## 当前硬件配置

| 手 | 型号 | CAN 接口 | 触觉 | CAN ID |
|------|------|---------|------|--------|
| 右手 | O6 (6自由度) | can0 | 无 | 0x27 |
| 左手 | O6 (6自由度) | can1 (待接) | 无 | 0x28 |

> **注意**：右手灵巧手与右臂机械臂共用 can0（CAN-FD 1M/5M），左手灵巧手与左臂共用 can1。

## ROS2 话题

### 控制话题（订阅）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cb_{left\|right}_hand_control_cmd` | `sensor_msgs/JointState` | 手指位置/速度/力矩指令 |
| `/cb_hand_setting_cmd` | `std_msgs/String` (JSON) | 设置指令（速度/力矩/清故障等） |

### 状态话题（发布）

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/cb_{left\|right}_hand_state` | `sensor_msgs/JointState` | ~60 Hz | 手指关节状态 |
| `/cb_{left\|right}_hand_info` | `std_msgs/String` (JSON) | ~60 Hz | 手部信息（版本/速度/电流/温度/力矩/故障） |

### 触觉话题（发布，需硬件支持）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cb_{left\|right}_hand_force` | `Float32MultiArray` | 单点压力传感器数据 |
| `/cb_{left\|right}_hand_matrix_touch` | `std_msgs/String` (JSON) | 矩阵压力传感器原始数据 |
| `/cb_{left\|right}_hand_matrix_touch_pc` | `PointCloud2` | 矩阵压力传感器点云格式 |
| `/cb_{left\|right}_hand_matrix_touch_mass` | `std_msgs/String` (JSON) | 矩阵压力传感器合值（单位：g） |

## Launch 文件

### linker_hand.launch.py — 单手模式

当前默认配置右手 O6：

```python
parameters=[{
    'hand_type': 'right',     # left | right
    'hand_joint': 'O6',       # O6/L6/L6P/L7/L10/L20/L21/L25/G20
    'is_touch': False,        # 是否有压力传感器
    'can': 'can0',            # CAN 总线接口
    'modbus': 'None',         # Modbus 接口 (None 或 /dev/ttyUSB0)
}]
```

### linker_hand_double.launch.py — 双手模式

同时启动左右两只灵巧手（示例为 G20 + can0/can1），需根据实际硬件修改参数。

## 快速开始

### 构建

```bash
cd ~/svtrobo_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select linker_hand_ros2_sdk
source install/setup.bash
```

> **依赖**：需要 `python-can >= 4.0`。系统自带 3.3.2 太旧，需手动升级：
> ```bash
> pip3 install --upgrade python-can
> ```

### 启动

```bash
# 单手（右手 O6，默认配置）
ros2 launch linker_hand_ros2_sdk linker_hand.launch.py

# 双手
ros2 launch linker_hand_ros2_sdk linker_hand_double.launch.py
```

### systemd 服务

灵巧手服务通过 systemd 管理：

```bash
# 查看状态
systemctl status svtrobo-linker-hand

# 启动/停止
sudo systemctl start svtrobo-linker-hand
sudo systemctl stop svtrobo-linker-hand

# 查看日志
journalctl -u svtrobo-linker-hand -f
```

服务配置：
- **依赖**：`After=svtrobo-can.service`（需 CAN 接口就绪）
- **环境**：`ROS_LOCALHOST_ONLY=1`
- **自动重启**：`Restart=on-failure, RestartSec=5`

## 手指控制示例

```bash
# 查看右手状态
ros2 topic echo /cb_right_hand_state

# 查看手部信息
ros2 topic echo /cb_right_hand_info

# 发送手指位置指令（O6: 6个关节，值 0-255）
ros2 topic pub --once /cb_right_hand_control_cmd sensor_msgs/JointState \
  "{name: ['thumb', 'index', 'middle', 'ring', 'little', 'palm'],
    position: [200, 255, 255, 255, 255, 180]}"
```

## 设置指令

通过 `/cb_hand_setting_cmd` 发送 JSON 指令：

```bash
# 设置速度
ros2 topic pub --once /cb_hand_setting_cmd std_msgs/String \
  "{data: '{\"setting_cmd\": \"set_speed\", \"params\": {\"hand_type\": \"right\", \"speed\": [200, 250, 250, 250, 250, 250]}}'}"

# 设置力矩
ros2 topic pub --once /cb_hand_setting_cmd std_msgs/String \
  "{data: '{\"setting_cmd\": \"set_max_torque_limits\", \"params\": {\"hand_type\": \"right\", \"torque\": [250, 250, 250, 250, 250, 250]}}'}"

# 清除故障
ros2 topic pub --once /cb_hand_setting_cmd std_msgs/String \
  "{data: '{\"setting_cmd\": \"clear_faults\", \"params\": {\"hand_type\": \"right\"}}'}"
```

## 依赖

- ROS2 Humble
- `python-can >= 4.0`（CAN 总线通信）
- `numpy`
- `rclpy`, `sensor_msgs`, `std_msgs`, `geometry_msgs`

## 常见问题排查

### 1. 启动报 python-can 版本错误

```
ModuleNotFoundError: No module named 'can.interfaces.socketcan`
```

系统自带 python-can 3.3.2 不支持 CAN-FD，需升级：

```bash
pip3 install 'python-can>=4.0'
```

### 2. CAN 接口未就绪

灵巧手依赖 CAN 接口已 UP。检查：

```bash
ip link show can0  # 确保 UP + CAN-FD
```

若未就绪，重启 CAN 服务：

```bash
sudo systemctl restart svtrobo-can
```

### 3. O6 手指值范围

O6 灵巧手 6 个自由度，位置值范围 0-255：

| 关节 | 范围 | 说明 |
|------|------|------|
| thumb | 0-255 | 拇指 |
| index | 0-255 | 食指 |
| middle | 0-255 | 中指 |
| ring | 0-255 | 无名指 |
| little | 0-255 | 小指 |
| palm | 0-255 | 掌部 |

- 255 = 完全张开
- 初始位置建议：`[200, 255, 255, 255, 255, 180]`

### 4. 与机械臂共用 CAN 总线

灵巧手与机械臂共用 can0（右手）/ can1（左手）。两者使用不同的 CAN ID，不会冲突：
- 机械臂：通过 OpenArm CAN-FD 驱动层（`openarm_can`）
- 灵巧手：通过 LinkerHand API（ID 0x27/0x28）
