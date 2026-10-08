# svt 服务器系统配置文档

> 最后更新: 2026-04-18
> 硬件: Jetson Orin (aarch64), L4T 5.15.185-tegra
> IP: 10.0.0.56 (eno1), 用户: svt

---

## 1. 系统信息

| 项目 | 值 |
|------|-----|
| Kernel | 5.15.185-tegra aarch64 PREEMPT |
| PCAN Driver | pcan.ko Release_20260126_n |
| ROS2 | Humble |
| Python | 3.10 |
| Node.js | v22.22.2 |
| pnpm | /usr/bin/pnpm |

## 2. systemd 自启服务 (共10个, 全部 enabled)

### 启动顺序与依赖链

```
sysinit.target
  └─ f710-fix.service (最早，Before=basic.target)
       └─ svtrobo-can.service (oneshot, 等 PCAN USB 就绪 ~30s)
            ├─ svtrobo-rosbridge.service (rosbridge WebSocket :9090)
            │    └─ svtrobo-chassis.service (chassis_control + lift_control)
            │         ├─ svtrobo-chassis-watchdog.service (每5秒检查 chassis_control_node 存活)
            │         ├─ svtrobo-f710.service (F710 手柄, 等 js0 最多 30s)
            │         ├─ svtrobo-web.service (web_control :8080)
            │         └─ svtrobo-nodeapi.service (Node.js API :28181)
            └─ pcan-monitor.service (每10秒检查 CAN 状态 + err-71 检测)
            └─ svtrobo-arm.service (双臂控制, bimanual, ROS_LOCALHOST_ONLY=1)
            └─ svtrobo-linker-hand.service (灵巧手 O6, ROS_LOCALHOST_ONLY=1)
            └─ (以上均 Wants=svtrobo-can.service)
```

所有服务均 `Restart=on-failure, RestartSec=5`（f710-fix 和 svtrobo-can 除外，它们是 oneshot）。

**ROS_LOCALHOST_ONLY**: svtrobo-arm 服务设置了 `ROS_LOCALHOST_ONLY=1`，限制 DDS 通讯仅在本机回环接口，防止局域网其他 ROS2 设备干扰。

### 2.1 f710-fix.service

**用途**: 解决 Jetson 内核内置 hid-logitech 驱动在 F710 上 probe 失败的问题。
**原理**: 在任何 USB 设备绑定前，设置 `ignore_special_drivers=1`，让 hid-core 跳过 hid-logitech，使用 hid-generic。

```ini
[Unit]
Description=F710 Gamepad Fix (Jetson hid-logitech workaround)
DefaultDependencies=no
Before=basic.target

[Service]
Type=oneshot
ExecStart=/bin/bash -c "echo 1 > /sys/module/hid/parameters/ignore_special_drivers"
RemainAfterExit=yes

[Install]
WantedBy=sysinit.target
```

**注意**: DO NOT 用 rmmod hid_logitech（builtin 不可能卸载）或 /etc/modprobe.d quirks（对 builtin 模块无效）。

### 2.2 svtrobo-can.service

**用途**: 等待 PCAN USB 设备注册、稳定后配置 CAN 接口 + 清理 FastRTPS 残留。

```ini
[Unit]
Description=Configure CAN interfaces for svtrobo
After=sysinit.target f710-fix.service
Wants=f710-fix.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/bash -c '\
WAIT_DEV=0; \
while [ $WAIT_DEV -lt 90 ] && ! ip link show can0 >/dev/null 2>&1; do \
  sleep 1; WAIT_DEV=$((WAIT_DEV+1)); \
done; \
logger "svtrobo-can: can0 appeared after ${WAIT_DEV}s"; \
if [ $WAIT_DEV -ge 90 ]; then logger "svtrobo-can: FATAL can0 not found after 90s"; exit 1; fi; \
sleep 5; \
ERR_BEFORE=$(dmesg | grep -c "pcan.*err"); \
WAIT=0; \
while [ $WAIT -lt 30 ]; do \
  sleep 1; \
  ERR_NOW=$(dmesg | grep -c "pcan.*err"); \
  if [ "$ERR_NOW" -eq "$ERR_BEFORE" ]; then break; fi; \
  ERR_BEFORE=$ERR_NOW; WAIT=$((WAIT+1)); \
done; \
logger "svtrobo-can: PCAN stabilized after ${WAIT}s (errors: $ERR_BEFORE)"; \
for can_if in can0 can1; do \
  ip link set $can_if down 2>/dev/null; \
  ip link set $can_if type can bitrate 1000000 dbitrate 5000000 fd on 2>/dev/null; \
  ip link set $can_if up 2>/dev/null; \
done; \
for can_if in can2 can3; do \
  ip link set $can_if down 2>/dev/null; \
  ip link set $can_if type can bitrate 1000000 2>/dev/null; \
  ip link set $can_if up 2>/dev/null; \
done; \
for can_if in can4 can5; do \
  ip link set $can_if down 2>/dev/null; \
  ip link set $can_if type can bitrate 500000 2>/dev/null; \
  ip link set $can_if up 2>/dev/null; \
done; \
chmod 777 /dev/ttyACM0 2>/dev/null; \
rm -rf /dev/shm/fastrtps_* 2>/dev/null; \
logger "svtrobo-can: CAN configured, FastRTPS shm cleaned"'

[Install]
WantedBy=multi-user.target
```

**CAN 配置**:

| 接口 | 模式 | Bitrate | 用途 |
|------|------|---------|------|
| can0 | CAN FD | 1M/5M | 右臂 (openarm_right) + 右灵巧手 O6 (0x27) |
| can1 | CAN FD | 1M/5M | 左臂 (openarm_left) + 左灵巧手 O6 (0x28 待接) |
| can2 | CAN 2.0 | 1M | 底盘转向电机 (RobStride) |
| can3 | CAN 2.0 | 1M | 底盘轮电机 (ZLAC8015D) |
| can4 | CAN 2.0 | 500K | 扩展 |
| can5 | CAN 2.0 | 500K | 扩展 |

### 2.3 svtrobo-rosbridge.service

```ini
[Unit]
Description=ROS2 Rosbridge WebSocket Server
After=network.target svtrobo-can.service
Wants=svtrobo-can.service

[Service]
Type=simple
User=svt
Environment=ROS_DOMAIN_ID=0
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/svt/svtrobo_ws/install/setup.bash && exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**端口**: 9090 (WebSocket)

### 2.4 svtrobo-arm.service

**用途**: OpenArm bimanual 双臂控制，通过 CAN-FD 驱动左右两条机械臂。

```ini
[Unit]
Description=OpenArm机械臂控制
After=svtrobo-can.service
Wants=svtrobo-can.service

[Service]
Type=simple
User=svt
Environment=ROS_DOMAIN_ID=0
Environment=ROS_LOCALHOST_ONLY=1
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/svt/svtrobo_ws/install/setup.bash && exec ros2 launch arm_preset_manager arm_preset_manager.launch.py"
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**启动节点**: robot_state_publisher, ros2_control_node, preset_manager_node

**Controller 启动时序**: joint_state_broadcaster(1s) → left/right_forward_position_controller(1.5s) → left/right_gripper_controller(2s) → preset_manager(5s)

**注意**: 必须设置 `ROS_LOCALHOST_ONLY=1`，局域网有多个 ROS2 设备，否则 DDS 发现会失败。

### 2.5 svtrobo-linker-hand.service

**用途**: Linker Hand O6 灵巧手控制，通过 CAN 总线驱动右手灵巧手（6 自由度）。

```ini
[Unit]
Description=Linker Hand O6 dexterous hand control
After=svtrobo-can.service
Wants=svtrobo-can.service

[Service]
Type=simple
User=svt
Environment=ROS_DOMAIN_ID=0
Environment=ROS_LOCALHOST_ONLY=1
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/svt/svtrobo_ws/install/setup.bash && exec ros2 launch linker_hand_ros2_sdk linker_hand.launch.py"
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**启动节点**: linker_hand_sdk (LinkerHand)

**CAN 接口**: can0（右手 O6, ID 0x27），与右臂机械臂共用 CAN 总线

**ROS2 话题**:
- 发布: `/cb_right_hand_state` (JointState, ~60Hz), `/cb_right_hand_info` (JSON)
- 订阅: `/cb_right_hand_control_cmd` (JointState), `/cb_hand_setting_cmd` (JSON)

**注意**:
- 需要 `python-can >= 4.0`（系统自带 3.3.2 不支持 CAN-FD）
- 灵巧手与机械臂共用 can0/can1，使用不同 CAN ID，不冲突

### 2.6 svtrobo-chassis.service

```ini
[Unit]
Description=svtrobo Chassis Control
After=svtrobo-rosbridge.service svtrobo-can.service
Wants=svtrobo-rosbridge.service svtrobo-can.service

[Service]
Type=simple
User=svt
Environment=ROS_DOMAIN_ID=0
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/svt/svtrobo_ws/install/setup.bash && exec ros2 launch chassis_control svtrobo_bringup.launch.py"
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**启动节点**: chassis_control_node, lift_control

**已知问题**: chassis_control_node crash 时 ros2 launch 父进程不退出，systemd 不会触发 Restart。现已通过 chassis-watchdog 自动恢复（见 2.5）。

### 2.7 svtrobo-chassis-watchdog.service

**用途**: 每5秒检查 chassis_control_node 是否存活，连续2次检测失败则自动 restart svtrobo-chassis 服务。
**原理**: 通过 `ros2 node list` 检查 `/chassis_control` 是否存在，解决 ros2 launch 父进程不退出导致 systemd 无法自动恢复的问题。

```ini
[Unit]
Description=svtrobo Chassis Watchdog
After=svtrobo-chassis.service
Wants=svtrobo-chassis.service

[Service]
Type=simple
User=svt
Environment=ROS_DOMAIN_ID=0
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/svt/svtrobo_ws/install/setup.bash && exec python3 /home/svt/svtrobo_ws/src/chassis_control/scripts/chassis_watchdog.py"
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**检测逻辑**: 每5秒轮询一次，连续2次未发现 `/chassis_control` 节点 → 执行 `systemctl restart svtrobo-chassis`。

### 2.8 pcan-monitor.service

**用途**: 每10秒检查 CAN 接口状态 + PCAN err-71 检测，自动尝试恢复。
**原理**: 监控 `ip link show canX` 状态和 dmesg 中的 PCAN 错误，检测到异常时执行 modprobe 重载和服务重启。

```ini
[Unit]
Description=PCAN CAN Monitor
After=svtrobo-can.service
Wants=svtrobo-can.service

[Service]
Type=simple
User=svt
ExecStart=/bin/bash -c "exec python3 /home/svt/svtrobo_ws/scripts/pcan_monitor.py"
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**检测逻辑**: 每10秒检查一次 CAN 接口状态和 dmesg 中 pcan err-71 错误计数。

### 2.9 svtrobo-f710.service

```ini
[Unit]
Description=svtrobo F710 Gamepad Teleop
After=svtrobo-chassis.service f710-fix.service
Wants=svtrobo-chassis.service f710-fix.service

[Service]
Type=simple
User=svt
Environment=ROS_DOMAIN_ID=0
ExecStartPre=/bin/bash -c "for i in $(seq 1 30); do [ -e /dev/input/js0 ] && exit 0; sleep 1; done; exit 1"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/svt/svtrobo_ws/install/setup.bash && exec ros2 launch f710_teleop f710_teleop.launch.py"
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**启动节点**: my_controller_node

**ExecStartPre**: 等待 /dev/input/js0 出现（最多30秒）

### 2.10 svtrobo-web.service

```ini
[Unit]
Description=svtrobo Web Control Panel
After=svtrobo-chassis.service

[Service]
Type=simple
User=svt
WorkingDirectory=/home/svt/svtrobo_ws/src/web_control
Environment=ROS_DOMAIN_ID=0
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source /home/svt/svtrobo_ws/install/setup.bash && exec python3 server.py --host 0.0.0.0 --port 8080"
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**端口**: 8080 (HTTP)

### 2.11 svtrobo-nodeapi.service

```ini
[Unit]
Description=svtrobo ROS Process API (Node.js)
After=svtrobo-chassis.service

[Service]
Type=simple
User=svt
WorkingDirectory=/home/svt/ros_process_api
Environment=ROS2_WORKSPACE_DIR=/home/svt/svtrobo_ws
Environment=SUDO_PASSWORD=***
Environment=PATH=/usr/bin:/bin:/usr/local/bin
ExecStart=/usr/bin/node /home/svt/ros_process_api/app.bundle.js
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

**端口**: 28181 (HTTP)

**环境变量覆盖**: app.bundle.js 硬编码了 /home/openarm 路径和 openarm 密码，通过 `ROS2_WORKSPACE_DIR` 和 `SUDO_PASSWORD` 环境变量覆盖。

## 3. udev 规则

### 3.1 PCAN udev 规则 (PCAN 驱动自带)

文件: `/etc/udev/rules.d/` 中 pcan 相关规则（驱动安装时自动生成），创建 `/dev/pcan-usb_pro_fd/` 等符号链接，MODE=0666。

### 3.2 CAN 接口重命名规则

文件: `/etc/udev/rules.d/` (use_pcan_as_can0_can1.sh 生成)

```
# Rename onboard mttcan to avoid can0/can1 conflicts.
SUBSYSTEM=="net", ACTION=="add", DRIVERS=="mttcan", KERNEL=="can0", NAME="mttcan0"
SUBSYSTEM=="net", ACTION=="add", DRIVERS=="mttcan", KERNEL=="can1", NAME="mttcan1"

# Map PEAK PCAN USB to can0/can1
SUBSYSTEM=="net", ACTION=="add", KERNEL=="can2", NAME="can0"
SUBSYSTEM=="net", ACTION=="add", KERNEL=="can3", NAME="can1"
```

**原理**: Jetson 内置 mttcan 会占 can0/can1 名称，通过 udev 将其重命名为 mttcan0/mttcan1，让 PCAN USB 设备使用 can0/can1。

## 4. USB 硬件拓扑

```
Bus 02 (USB 3.0, 10000M)
  └─ Hub (Realtek 4-port)
       └─ ZED 2i (f880, Video, 5000M)

Bus 01 (USB 2.0, 480M)
  ├─ AX210 Bluetooth
  └─ Hub (Realtek 4-port USB 2.0)
       ├─ Hub (Microchip 2514)
       │    ├─ F710 Gamepad (046d:c219, via hid-generic)
       │    ├─ PCAN-USB Pro FD (0c72:0011)
       │    ├─ PCAN-USB Pro FD (0c72:0011)
       │    ├─ PCAN-USB Pro FD (0c72:0011)
       │    └─ Hub (Microchip 2512)
       │         ├─ CH340 Serial (ttyACM0, 升降控制)
       │         └─ ZED 2i HID (f881, IMU)
       └─ Hub (Genesys Logic)
```

**3个 PCAN-USB Pro FD** 经两层 USB 2.0 Hub 连接，这是 err -71 的根因。

## 5. ROS2 节点配置

### 5.1 F710 手柄配置
文件: `src/f710_teleop/config/f710_teleop.yaml`

```yaml
# 关键参数
device_path: "/dev/input/js0"
deadzone: 0.4
publish_rate: 25.0
safety.gate_until_first_a: true
safety.estop_latch: true

# 速度
velocity.max_linear: 0.4
velocity.max_angular: 0.4
velocity.angular_scale: 5.0
speed_scale.initial: 0.25
speed_scale.max: 1.0
speed_scale.hard_max: 1.5

# 轴映射
axis.left_x: 0    # 左摇杆 左右
axis.left_y: 1    # 左摇杆 前后
axis.right_x: 2   # 右摇杆 左右 → angular.z

# 按钮映射 (D-mode, hid-generic)
button.a: 1
button.b: 2
button.lb: 4
button.rb: 5
button.lt: 6
button.rt: 7

# 采集按钮
button.x: 0       # 开始录制
button.y: 3       # 停止录制
recording.enabled: true
recording.server_url: "http://127.0.0.1:8080"

# 取反
invert.left_x: true
invert.right_x: true
disable_left_x_when_right_active: true

# 升降
lift.speed: 500
```

### 5.2 底盘参数
文件: `src/chassis_control/config/params.yaml`

```yaml
chassis_control:
  ros__parameters:
    robot:
      chassis_radius: 0.0875
      wheel_perimeter: 0.647
      fl_motor_start_angle: 0.2
      fr_motor_start_angle: 4.9
      rl_motor_start_angle: 2.7
      rr_motor_start_angle: 3.5
```

## 6. 网络配置

| 接口 | 状态 | 地址 |
|------|------|------|
| eno1 | UP | 10.0.0.56/24 |
| mttcan0 | DOWN | (Jetson 内置 CAN, udev 重命名) |
| mttcan1 | DOWN | (Jetson 内置 CAN, udev 重命名) |
| can0-5 | UP | (PCAN USB) |
| docker0 | DOWN | 172.17.0.1/16 |

## 7. 故障恢复快速参考

### CAN 全部 DOWN (PCAN err-71)
```bash
# 快速恢复 (modprobe 重载 + CAN 配置 + 服务重启)
ssh svt@10.0.0.56 "echo '<sudo密码>' | sudo -S modprobe -r pcan && sleep 2 && echo '<sudo密码>' | sudo -S modprobe pcan && sleep 5 && echo '<sudo密码>' | sudo -S systemctl restart svtrobo-can && echo '<sudo密码>' | sudo -S systemctl restart svtrobo-rosbridge && sleep 2 && echo '<sudo密码>' | sudo -S systemctl restart svtrobo-arm svtrobo-chassis svtrobo-f710 svtrobo-web svtrobo-nodeapi && rm -rf /dev/shm/fastrtps_* && echo RECOVERED"
```

### chassis_control_node 崩溃 (SDO write timeout)
```bash
# watchdog 会自动恢复，也可手动重启
ssh svt@10.0.0.56 "echo '<sudo密码>' | sudo -S systemctl restart svtrobo-chassis svtrobo-f710"
```

### DDS 发现失败 (节点互相看不到)
```bash
ssh svt@10.0.0.56 "echo '<sudo密码>' | sudo -S rm -rf /dev/shm/fastrtps_* && echo '<sudo密码>' | sudo -S systemctl restart svtrobo-rosbridge svtrobo-chassis svtrobo-f710"
```

### F710 手柄无摇杆数据
换电池。低电量时摇杆轴最先失效，按键和扳机仍有响应。

### 手柄设备丢失 (/dev/input/js0 不存在)
```bash
ssh svt@10.0.0.56 "echo '<sudo密码>' | sudo -S sh -c 'echo 1 > /sys/module/hid/parameters/ignore_special_drivers && echo 1-4.2.1.4 > /sys/bus/usb/drivers/usb/unbind && sleep 2 && echo 1-4.2.1.4 > /sys/bus/usb/drivers/usb/bind && sleep 3 && ls -la /dev/input/js0'"
```
注意: USB 路径会随 Hub 布局变化，用 `lsusb -t` 查当前路径。

## 8. 已知限制 (待改进)

1. **chassis_control_node 崩溃自动恢复** — 已通过 chassis-watchdog 服务实现，每5秒检测，连续2次失败自动 restart svtrobo-chassis
2. **PCAN err-71 运行时不稳定** — USB 2.0 Hub + 3个PCAN 过载，可能随时掉线。已通过 pcan-monitor 服务实现每10秒自动检测和恢复
3. **FastRTPS shm 残留** — 频繁重启后 DDS 发现完全失败
4. **F710 低电量** — 摇杆轴先失效，无低电量告警
5. **D405 相机** — 当前未连接服务器，录制时会快速跳过（~0.1s 检测）

## 9. 相机与录制系统

### 9.1 相机配置

| 相机 | 类型 | 序列号 | 分辨率 | 深度模式 | 状态 |
|------|------|--------|--------|----------|------|
| d405_1 | RealSense D405 | 409122272399 | 640x480 | z16 | 未连接 |
| d405_2 | RealSense D405 | 409122273344 | 640x480 | z16 | 未连接 |
| zed | ZED 2i (SDK) | S/N 33786357 | HD720@15fps | NEURAL | 正常 |

**配置文件**: `src/web_control/server.py` 中的 `CAMERA_CONFIG`

### 9.2 录制数据结构

录制会话保存在 `recordings/<YYYYMMDD_HHMMSS>/`：

```
recordings/<session>/
├── rosbag/              # ROS2 bag (所有话题)
├── images/<cam>/        # 彩色图 JPEG (采集频率)
├── depth/<cam>/         # 深度图 .jpg (JET colormap, 采集频率)
├── pointcloud/zed/      # ZED 点云 npz (非压缩), (720,1280,4) float32 (~8.9MB/帧, 2Hz)
├── imu.jsonl            # IMU 数据 (~15Hz (native grab rate)), 每行一个 JSON: accel, gyro_dps, gyro_rad, mag, imu_temp, pressure, env_temp
├── chassis_diagnostics.jsonl
├── chassis_joint_states.jsonl
├── f710_joy.jsonl
├── lift_control_cmd.jsonl
├── svtrobot_cmd.jsonl
└── summary.json         # 录制摘要 (时长、帧数、各话题统计等)
```

**录制帧率**: ~13.5fps (优化后，原 ~7.7fps)

**录制优化措施**:
- 保存线程使用 fast poll + drain 队列策略，最大化磁盘写入吞吐
- ZED 点云使用 sl.Mat 复用，避免重复分配内存
- 点云采样: 每10帧采1帧（PC_SKIP=10），15fps÷10 ≈ 1.4Hz，以 `np.savez` (非压缩) 保存 XYZRGBA 数据
- 深度图保存为 .jpg (JET colormap JPEG)，不再使用 .npy 格式

### 9.3 IMU 独立读取

IMU 数据通过独立后台线程读取，无需启动 ZED 相机的视频/深度流：

- **模式**: ZED SDK, VGA@15fps, DEPTH_MODE.NONE（最轻量）
- **频率**: ~15Hz 写入 `imu_cache`
- **前端 API**: WebSocket `/ws/imu` 实时推送，HTTP `GET /api/imu` 回退（返回最新 accel, gyro_dps, gyro_rad, mag, imu_temp, pressure, env_temp）
- **录制时**: 直接从 `imu_cache` 读取并写入 `imu.jsonl`，不通过 ROS2 话题（无 IMU 发布者）
- **录制结束**: ZED 相机关闭后，IMU-only 线程自动恢复独立运行

### 9.4 RealSense 快速设备检测

`realsense_camera.py` 的 `start()` 方法在调用 `pipeline.start()` 前，先用 `rs.context().query_devices()` 检查目标序列号是否存在（<0.1秒）。设备不存在时直接抛出 RuntimeError，避免 pipeline.start() 阻塞 ~15秒。

### 9.5 前端显示

- 相机卡片仅显示彩色图流（MJPEG），**不显示深度图**
- 深度数据仅在录制时后台保存到磁盘
- IMU 数据通过 WebSocket `/ws/imu` 实时推送，HTTP `/api/imu` 作为回退
- IMU 数据显示跟随连接状态：未连接时显示 `--` 占位，连接后实时更新，断开后恢复占位

### 9.6 前端连接行为

- **自动连接**: 页面打开/刷新时自动根据当前 URL 拼接 `ws://<host>/ws` 连接 rosbridge，URL 输入框设为只读
- **手动断开**: 点击"断开连接"后不会自动重连，需手动点击"连接"或刷新页面
- **硬件状态栏**: 页面加载时即检测硬件在线状态（不依赖 ROS 连接）：
  - ZED 2i: 通过 `lsusb` 检测 USB 设备 (2b03:f880)
  - D405: 通过 `pyrealsense2` 检测序列号匹配，回退 `lsusb`
  - IMU: 通过 `/api/imu` 检测数据是否可用
  - 底盘/升降/手柄: 连接 ROS 后通过 rosapi 检测节点
- **后端 `/camera/status`**: 新增 `device` 字段，返回硬件是否物理连接（区别于 `running` 是否正在采集）

### 9.7 ZED 相机与 IMU 协调

ZED SDK 同一时刻只允许一个进程打开相机，需要协调 IMU 线程和相机采集：

- **正常待机**: IMU 独立线程以 VGA+DEPTH_MODE.NONE 模式打开 ZED，~15-20Hz 读取传感器数据
- **手动启动 ZED 相机**: `CameraManager.start_camera("zed")` 先调用 `stop_imu_reader()` 释放 ZED，再启动相机（HD720+NEURAL）。启动失败时自动恢复 IMU 线程
- **手动停止 ZED 相机**: `CameraManager.stop_camera("zed")` 关闭相机后调用 `start_imu_reader()` 恢复 IMU
- **录制流程**: `RecordingManager` 同样在录制开始时 `stop_imu_reader()`，录制结束后 `start_imu_reader()`，IMU 数据从运行中的 ZED 相机获取

### 9.8 相机全屏

每个相机画面右上角有全屏按钮（expand 图标），仅在该相机启动后显示：
- 鼠标悬停时半透明浮现，平时不可见
- 点击使用浏览器原生 Fullscreen API 全屏显示画面
- 全屏状态下按钮放大，按 Esc 退出
- 停止相机后按钮自动隐藏