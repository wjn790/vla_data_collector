# f710_teleop

Logitech F710（或兼容手柄）控制底盘与升降机构的 ROS2 Python 功能包。

## 功能概览

- 从 Linux 手柄设备（默认 `/dev/input/js1`）直接读取按键与轴数据。
- 发布底盘速度：
  - 话题：`/svtrobot_cmd`
  - 消息类型：`geometry_msgs/Twist`
  - 映射：
    - 左摇杆前后 → `linear.x`
    - 左摇杆左右 → `linear.y`
    - 右摇杆左右 → `angular.z`
- 发布升降控制：
  - 话题：`/lift_control_cmd`
  - 消息类型：`std_msgs/Int32MultiArray`
  - 数据格式：`[direction, speed]`
    - `direction = 1`：上升
    - `direction = -1`：下降
    - `direction = 0`：停止
    - `speed = 500`（可配置）
- LT / RT 调整底盘速度缩放因子 `speed_scale ∈ [0.01, 1.0]`。

## 依赖与环境

- **ROS2 发行版**：建议 Humble / Foxy（其他发行版只要支持 `rclpy` 即可）。
- **运行依赖包**（通常随 ROS2 一起安装）：
  - `rclpy`
  - `geometry_msgs`
  - `std_msgs`
- **调试工具**（可选但强烈推荐）：
  - `joystick`：提供 `jstest`，用于查看手柄轴 / 按钮编号  
    ```bash
    sudo apt install joystick
    ```
- **设备权限**：运行节点的用户需要有读 `/dev/input/js*` 的权限：
  - 临时方式：直接用有权限的用户（如 `input` 组）；
  - 永久方式：
    - 将当前用户加入 `input` 组：
      ```bash
      sudo usermod -aG input $USER
      ```
      重新登录后生效。
    - 或通过 `udev` 规则放宽 `/dev/input/js*` 权限（根据实际安全策略配置）。

## 构建与运行

在工作空间根目录（例如 `~/ros2_ws`）执行：

```bash
cd ~/ros2_ws
colcon build --packages-select f710_teleop
source install/setup.bash
```

### 使用 launch 启动

```bash
ros2 launch f710_teleop f710_teleop.launch.py
```

默认参数来自 `config/f710_teleop.yaml`，其中包括：

- `device_path`：手柄设备路径（例如 `/dev/input/js1`）
- 各轴、按钮索引映射
- 速度缩放、升降速度等

### 直接运行节点

```bash
ros2 run f710_teleop my_controller_node --ros-args \
  -p device_path:=/dev/input/js1 \
  -p button.lt:=6 -p button.rt:=7
```

你可以根据 `jstest /dev/input/js1` 的输出，调整轴与按钮索引，以适配不同的手柄布局。

## 常见问题排查

### 1. 启动时提示找不到参数文件

```text
[WARNING] [launch_ros.actions.node]: Parameter file path is not a file: config/f710_teleop.yaml
```

- 原因：未通过安装后的 share 目录访问配置文件或未重新构建安装。
- 处理：
  - 确保已在工作空间根目录执行：
    ```bash
    cd ~/ros2_ws
    colcon build --packages-select f710_teleop
    source install/setup.bash
    ```
  - 使用提供的 launch 启动：
    ```bash
    ros2 launch f710_teleop f710_teleop.launch.py
    ```

### 2. 不动手柄时 `/svtrobot_cmd` 不是 0，或者一直是固定值

- **轴号因设备/驱动而异**：务必用 `jstest` 对照。例如有的布局上右摇杆左右为 **Axes2**、前后为 **Axes3**；也有的 F710 在 Linux 上轴 **2** 是扳机模拟量，误映射会不动杆也转。请把 `axis.right_x` 设成**右摇杆左右**实际对应的那根轴。
- 参数 `deadzone.remap`（默认 true）：死区外把输入线性映射到 `[-1,1]`，避免刚过死区就接近满速，手感更顺。
- **systemd 开机自启**：在 `f710_teleop.yaml` 中设置 `startup.zero_cmd_sec: 2.0`～`5.0`，启动后若干秒内只发全 0 速度，等手柄枚举稳定后再正常发指令。
- 确认当前使用的设备：
  ```bash
  ros2 param get /my_controller_node device_path
  ```
  如果是 `/dev/input/js0`，多数情况下是虚拟指针设备（例如 VMware）。
- 正确的手柄设备通常类似：
  ```bash
  ls -l /dev/input/js*
  jstest /dev/input/js1
  ```
  输出里包含 `Logitech`、`Gamepad` 等字样且有多个轴/按钮。
- 在 `config/f710_teleop.yaml` 或启动参数中将 `device_path` 改为正确的设备，例如：
  ```bash
  ros2 run f710_teleop my_controller_node --ros-args -p device_path:=/dev/input/js1
  ```

### 3. 动手柄 `/svtrobot_cmd` 仍然没有变化

- 用 `jstest /dev/input/js1` 查看：
  - 摇杆、按钮是否在对应轴 / 按钮编号上变化。
- 检查配置：
  - `axis.left_x` / `axis.left_y` / `axis.right_x` 是否与当前手柄轴编号一致；
  - `button.lt` / `button.rt` 是否与 LT / RT 按钮编号一致。
- 可通过命令行快速覆盖以测试：
  ```bash
  ros2 run f710_teleop my_controller_node --ros-args \
    -p device_path:=/dev/input/js1 \
    -p axis.left_x:=0 -p axis.left_y:=1 -p axis.right_x:=2 \
    -p button.lt:=6 -p button.rt:=7
  ```

### 4. 安全：首次按 A、急停（B）

- **`safety.gate_until_first_a`（默认 true）**：在**第一次按下 A（上升沿）之前**，只发全 0（速度/升降均不生效），避免开机自启未就绪时乱动；按过 A 之后正常发指令。
- **`safety.require_deadman`（默认 false）**：若设为 **true**，则必须**一直按住 A** 才走车/升降（松手即停）。一般只开 `gate_until_first_a` 即可，不必再开 deadman。
- **`safety.estop_latch`（默认 true）**：**B 上升沿**急停（短按即可），清零并锁存；再按一次 **A（上升沿）** 解除。
- **`axis.smoothing_alpha`**：摇杆低通（0 关闭），减轻偶发抖动。

若希望恢复旧行为（启动即可动、无需先按 A），在 `f710_teleop.yaml` 中设 `safety.gate_until_first_a: false`。


