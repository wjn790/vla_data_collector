# SVT LingBot 数据采集器

该采集器用于录制带时间戳的原始 episode，不会向机器人发布任何控制命令。它针对当前的 SVT 机器人进行了适配：

- ROS domain 0 中的 14 个机械臂关节
- ROS domain 56 中的底座控制命令及命令反馈
- 当前手套控制 JSONL 日志中的左手 LinkerHand O6 命令和右手 Wuji 20 自由度命令
- ZED 2i 顶部相机和两台 D405 腕部相机

采集器首先按照各数据源自身的时间戳，将所有数据分别保存到原始时间线上。所有必需数据源就绪后写入 `recording_start_ns`，每个 episode 结束时再通过离线最近时间戳匹配生成单个、连续的 15 Hz 对齐时间线。采集阶段不会切分 episode、删除中间帧、重排时间戳或生成 LeRobot 数据。

## 环境依赖

- Ubuntu + ROS 2：采集器为每个 ROS domain 启动独立的 `/usr/bin/python3` 子进程，运行前需 source 对应 ROS 环境的 `setup.bash`。
- 相机 SDK（系统级安装，无法 pip 安装）：ZED SDK 的 Python API（`pyzed`）与 Intel RealSense SDK（`pyrealsense2`），均在 conda 环境中使用。
- Python 依赖见 `requirements.txt`（numpy、PyYAML、msgpack；测试另需 pytest），当前验证环境为 `/home/svt/miniconda3`。
- 按钮流程采集（`supervisor/` 各控制器）依赖 `/home/svt/glove_control` 的手套遥操作运行时（`mixed_glove_teleop`、Wuji / LinkerHand 硬件 SDK），该运行时依赖真实手套硬件，不随本仓库分发。仓库内的 `glove_control/` 只包含纯标准库实现的 `hand_command_arbiter.py`，供无手套环境的机器回退使用；各引用点在 `/home/svt/glove_control` 不存在时自动回退到仓库内副本。

默认路径（数据输出根目录 `/svtrobo_data/vla_data` 等）以 SVT 机器人标准布局为前提，可在 `config/svt.yaml` 等配置中调整。

## 目录结构

- `collect_episode.py`：episode 采集入口（原始时间线录制 + 自动离线对齐）
- `camera_capture.py` / `camera_sync.py`：三路相机抓图、硬件时间戳与同步
- `offline_align.py`：离线 15 Hz 对齐
- `validate_episode.py` / `episode_quality_monitor.py` / `review_day_dataset.py` / `realign_day.py` / `repair_curated_dataset.py`：数据校验、质检与修复
- `raw_recording.py` / `common.py` / `hand_log_source.py` / `ros_domain_agent.py`：底层录制组件
- `supervisor/`：按钮流程与长任务采集控制器，以及策略客户端等执行侧组件
- `viewer/` + `data_viewer.py`：网页数据查看器（systemd 服务见 `deploy/`）
- `tests/`：离线单元测试（`python -m pytest tests`）
- `config/`：采集、质检与推理 YAML 配置
- `svtrobo_ws/`：机器人 ROS 2 工作空间源码快照（机械臂 / 底盘 / 灵巧手 / 相机驱动、web 控制、systemd 服务），取自设备 `/home/svt/svtrobo_ws` 的 git 跟踪文件并包含其后的未提交修改；原始仓库 `On-The-Ways/svtrobo_ws` 停在 2026-06-11。为公开分发，快照中的明文 sudo / CAN 密码已替换为占位符。

## 重要硬件限制

数据写入 `/svtrobo_data/vla_data` 目录下；不要将 episode 写入空间已接近占满的根文件系统。`/svtrobo_data` 本身归 `root` 所有，其中的 `recordings` 目录允许用户 `svt` 写入。左侧 D405 当前连接 USB 2.0，右侧 D405 连接 USB 3.x。在两台相机分别连接这两条总线的情况下，两路 RGB 视频流均通过了 640x480、30 Hz 的持续运行测试，测试过程中未出现超时或帧编号丢失。对齐后的连续时间线为 15 Hz，并以左腕相机每隔一帧的图像作为参考帧。

当前 `svtrobo-web.service` 占用 ZED 相机。采集器在该服务停止前会拒绝启动：

```bash
sudo systemctl stop svtrobo-web.service
```

采集完成后，如需使用网页画面，可重新启动该服务：

```bash
sudo systemctl start svtrobo-web.service
```

## 录制一个 episode

首先启动现有的机械臂和灵巧手遥操作程序。然后在另一个终端中运行：

```bash
cd /home/svt/lingbot_data_collector
/home/svt/miniconda3/bin/python collect_episode.py \
  --task "pick up the red block and place it in the tray"
```

默认不需要指定 episode 名称。采集器使用启动时的本地时间创建目录，例如 `/svtrobo_data/vla_data/2026-07-24/113012`。同一秒内重复启动时，会自动追加 `_01`、`_02` 等序号，避免覆盖已有数据。如需添加便于识别的后缀，仍可使用 `--episode-name red_block_001`。

按 `Ctrl-C` 停止录制，也可以指定录制时长：

```bash
/home/svt/miniconda3/bin/python collect_episode.py \
  --task "open the drawer" --duration 30
```

采集器会为每个 ROS domain 启动一个独立的 `/usr/bin/python3` 子进程，并使用 conda 环境中的 Python 调用相机 SDK。采集器不会启动遥操作程序，也绝不会发送动作命令。

顶部 ZED 2i 通过 SDK 以 HD720（1280x720）、30 Hz 模式运行，并设置 `sensors_required=false`。这样既可以避免 IMU 初始化失败，又可以保留图像的硬件时间戳。因此，三台相机的曝光时间戳均可在 SVT 主机上进行比较。

正常结束录制后，离线对齐会自动运行。`--no-offline-align` 仅用于诊断。已有的原始 episode 可以稍后执行对齐：

```bash
/home/svt/miniconda3/bin/python offline_align.py \
  /svtrobo_data/vla_data/2026-07-23/120000_red_block_001
```

## 验证数据

```bash
/home/svt/miniconda3/bin/python validate_episode.py \
  /svtrobo_data/vla_data/2026-07-23/120000_red_block_001 \
  --decode-images
```

每个 episode 包含以下内容：

```text
manifest.json
config.yaml
alignment_report.json
frames.jsonl
online_frames.jsonl
raw/cameras/camera_top/images/*.jpg
raw/cameras/camera_top/timestamps.jsonl
raw/cameras/camera_wrist_left/images/*.jpg
raw/cameras/camera_wrist_left/timestamps.jsonl
raw/cameras/camera_wrist_right/images/*.jpg
raw/cameras/camera_wrist_right/timestamps.jsonl
raw/ros/*.jsonl
raw/hands/teleop.jsonl
```

`frames.jsonl` 是离线对齐后的完整审计视图，不是已经切好的训练数据。每一帧包含 14 个机械臂数值、12 个虚拟手部数值（左手 6 个、右手 6 个）、3 个底座速度值、三张图像的引用、带正负号的各数据源时间差，以及有效性标志。Wuji 原始的 20 个数值仍保留在 `raw.hand` 中。`online_frames.jsonl` 仅保留用于诊断。

每个相机的抓图进程只取图并立即记录硬件时间戳，再把图像送入有界队列；独立的编码进程负责 JPEG 编码和写盘。因此编码或磁盘短时抖动不会阻塞 ZED 抓图。队列满时允许丢弃少量图像，累计数量写入 `manifest.camera_dropped_before_encoding`，不会用旧图像冒充当前帧。离线对齐以 30 Hz 左侧 D405 的每隔一帧作为 15 Hz 主时间线，然后分别选择时间戳最接近的右侧 D405、ZED、机械臂状态、左右臂控制命令、手部命令和手部真实状态。相机、机械臂和手部数据与主时间线的时间差必须不超过 30 ms；左右腕相机的曝光时间差也必须不超过 30 ms。

离线时间线同时受 `recording_start_ns`、`recording_end_ns` 及所有必需数据源的共同时间范围约束，因此不包含相机预热阶段，也不包含机械臂命令或手部反馈停止后的尾部。手部命令按 `_timestamp_ns` 对齐，硬件状态按 `_actual_state_timestamp_ns` 独立对齐。按钮控制节点以 60 Hz 写命令日志，硬件反馈在线程中以 30 Hz 更新缓存，硬件读取不会阻塞日志节拍。

离线对齐随后会自动修复夹在两个有效帧之间的单帧机械臂或手部异常。只有原始误差不超过 50 ms、相邻帧跨度不超过 170 ms、并且当前相机和腕部配对均有效时才允许按真实时间线性插值。相机异常、连续异常、首尾异常或更大误差不会修复，仍保留诊断数据并将 `valid_for_training` 设为 `false`。修复帧会在 `repair`、`validity.repaired` 和 `validity.repair_sources` 中明确标记，原始 `alignment_delta_ms` 不会被覆盖。转换为 LeRobot 格式之前，必须检查 `alignment_report.json`。

右手从 20 自由度投影到 6 自由度的配置位于 `config/svt.yaml`。该配置属于元数据映射，并非不可逆转换。正式采集前，应通过一个简短的标定 episode 检查默认映射是否符合 Wuji 实际关节的运动方向。

当前按钮控制日志同时包含目标命令和两只灵巧手的物理状态反馈。离线有效帧只接受 `hardware_feedback`，读取失败时保留最后一次反馈及其原始时间戳；一旦变旧就会被判为无效，不会使用命令值或 0 冒充实测状态。

## 离线冒烟测试

此测试使用生成的图像，不会连接 ROS 或机器人硬件，但仍要求手套遥操作日志保持新鲜（手部数据是必需数据源）。运行前请先启动手套遥操作程序，否则采集器会以 "hand data startup failed" 拒绝启动：

```bash
/home/svt/miniconda3/bin/python collect_episode.py \
  --task smoke_test --duration 2 --mock-cameras --no-ros \
  --output-root /svtrobo_data/recordings/lingbot_collector_smoke
```

由于机器人数据已禁用，这些帧按预期会被判定为不可用于训练。
