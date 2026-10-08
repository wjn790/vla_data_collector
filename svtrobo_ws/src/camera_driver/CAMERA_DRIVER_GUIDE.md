# SVTROBO 相机采集模块使用文档

> 文件位置：`camera_driver/camera_driver/`

---

## 目录

1. [系统概述](#1-系统概述)
2. [硬件信息](#2-硬件信息)
3. [前置条件](#3-前置条件)
4. [快速开始](#4-快速开始)
5. [RealSense D405 API](#5-realsense-d405-api)
6. [ZED 2i API](#6-zed-2i-api)
7. [ZED 2i IMU API](#7-zed-2i-imu-api)
8. [ROS2 相机节点](#8-ros2-相机节点)
9. [文件结构](#9-文件结构)
10. [注意事项与常见问题](#10-注意事项与常见问题)

---

## 1. 系统概述

本模块提供 RealSense D405 和 ZED 2i 相机的 Python 采集接口，支持：

- 彩色图像 + 深度图同时采集
- 自动深度对齐到彩色图（RealSense）
- ZED 2i IMU 传感器数据采集（加速度计、陀螺仪、磁力计）
- 上下文管理器自动管理生命周期
- 采集并保存到磁盘（按相机名称区分）
- 相机内参获取
- 彩色点云生成（RealSense）

---

## 2. 硬件信息

### 2.1 已连接相机

| 相机 | 型号 | 序列号 | USB 设备 |
|------|------|--------|----------|
| D405 #1 | Intel RealSense D405 | `409122272399` | Bus 001 |
| D405 #2 | Intel RealSense D405 | `409122273344` | Bus 001 |
| ZED 2i | STEREOLABS ZED 2i | - | `/dev/video0` (视频), `/dev/hidraw*` (IMU) |

### 2.2 分辨率支持

**D405 支持分辨率（彩色 / 深度）：**

| 分辨率 | 彩色 fps | 深度 fps | 备注 |
|--------|----------|----------|------|
| 1280x720 | 5/6/15 | 5/6/15 | **当前使用** @6fps |
| 848x480 | 5/10 | 5/10 | |
| 640x480 | 5/15/30 | 5/15/30 | |
| 640x360 | - | 30 | |
| 480x270 | 5/15/30/60 | 5/15/30/60 | |
| 424x240 | 5/15/30/60 | - | |

> 注意：彩色和深度流需要相同 fps 才能同时运行。两台D405在1280x720@6fps下可同时工作。

**ZED 2i（OpenCV V4L2 模式）：**

| 分辨率 | 说明 |
|--------|------|
| 1344x376 (VGA) | Side-by-Side，左右各 672x376，当前唯一可用 |

> ZED SDK 需要 NVIDIA GPU + CUDA，本机无 GPU，仅支持 OpenCV 降级模式。

---

## 3. 前置条件

### 依赖安装

```bash
# RealSense SDK
sudo apt install librealsense2-dev librealsense2-utils
pip3 install pyrealsense2

# 通用依赖（通常已安装）
pip3 install numpy opencv-python

# ZED 2i IMU 需要的 udev 规则（一次性设置）
echo 'SUBSYSTEM=="hidraw", ATTRS{idVendor}=="2b03", ATTRS{idProduct}=="f881", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/99-zed-imu.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 验证设备

```bash
# 列出 RealSense 设备
rs-enumerate-devices --short

# 列出视频设备
ls /dev/video*

# 查看 ZED 设备
cat /sys/class/video4linux/video0/name
# 应输出: ZED 2i
```

### 导入模块

```python
import sys
sys.path.insert(0, '/home/svt/svtrobo_ws')

from camera_driver.camera_driver import RealSenseCamera, ZEDCamera, ZEDIMU
```

---

## 4. 快速开始

### 4.1 RealSense D405

```python
import sys
sys.path.insert(0, '/home/svt/svtrobo_ws')
from camera_driver.camera_driver import RealSenseCamera

# 列出所有设备
devices = RealSenseCamera.list_devices()
for d in devices:
    print(f"  {d['name']} SN:{d['serial']}")

# 采集并保存
with RealSenseCamera(serial='409122272399') as cam:
    color_path, depth_path = cam.capture_and_save(name='d405_1')
    print(f'已保存: {color_path}')

# 仅获取 numpy 数组
with RealSenseCamera(serial='409122272399') as cam:
    color, depth = cam.capture()
    print(f'彩色: {color.shape}, 深度: {depth.shape}')
```

### 4.2 ZED 2i

```python
import sys
sys.path.insert(0, '/home/svt/svtrobo_ws')
from camera_driver.camera_driver import ZEDCamera

with ZEDCamera() as zed:
    left, depth = zed.capture()
    print(f'左眼: {left.shape}, 深度: {depth.shape}')

# 采集并保存
with ZEDCamera() as zed:
    left_path, depth_path = zed.capture_and_save(name='zed_2i')
```

---

## 5. RealSense D405 API

> 源文件：`camera_driver/camera_driver/realsense_camera.py`

### 5.1 构造函数

```python
RealSenseCamera(
    serial='',              # 设备序列号，空字符串自动选第一个
    color_size=(1280, 720), # (width, height) 彩色图分辨率
    depth_size=(1280, 720), # (width, height) 深度图分辨率
    fps=6,                  # 帧率
)
```

**参数说明：**

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `serial` | str | `''` | 设备序列号，空则选第一个设备 |
| `color_size` | tuple | `(1280, 720)` | 彩色图 (宽, 高) |
| `depth_size` | tuple | `(1280, 720)` | 深度图 (宽, 高) |
| `fps` | int | `6` | 帧率 |

### 5.2 `start()` — 启动相机

```python
cam = RealSenseCamera(serial='409122272399')
cam.start()
# 使用完毕后
cam.stop()
```

预热丢弃前 30 帧，确保曝光和白平衡稳定。

### 5.3 `stop()` — 停止相机

释放 pipeline 资源。

### 5.4 `capture()` — 采集一帧

```python
color, depth = cam.capture()
# color: numpy (H, W, 3) BGR uint8
# depth: numpy (H, W) uint16, 单位: depth_scale (默认 0.001 米)
# 失败返回 (None, None)
```

深度图已对齐到彩色图。深度值 × `depth_scale` = 实际距离（米）。

### 5.5 `capture_and_save()` — 采集并保存

```python
color_path, depth_path = cam.capture_and_save(
    save_dir=None,  # 保存目录，默认 camera_driver/captures/
    name='d405_1',  # 文件名前缀
)
# 返回: ('.../d405_1_20260410_123456_color.png', '.../d405_1_20260410_123456_depth.png')
# 失败返回: (None, None)
```

**参数说明：**

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `save_dir` | str | `None` | 保存目录，默认 `camera_driver/captures/` |
| `name` | str | 设备 SN | 文件名前缀，用于区分不同相机 |

**输出文件命名规则：**

```
{name}_{YYYYMMDD}_{HHMMSS}_{微秒}_color.png
{name}_{YYYYMMDD}_{HHMMSS}_{微秒}_depth.png
```

### 5.6 `get_intrinsics()` — 获取相机内参

```python
intrinsics = cam.get_intrinsics()
# {
#   'fx': 393.5, 'fy': 392.6,
#   'ppx': 318.2, 'ppy': 239.2,
#   'width': 1280, 'height': 720,
#   'coeffs': [0.0, 0.0, 0.0, 0.0, 0.0]
# }
```

### 5.7 `get_depth_scale()` — 获取深度比例因子

```python
scale = cam.get_depth_scale()  # 例: 0.001
# depth_米 = depth_像素值 × scale
```

### 5.8 `capture_pointcloud()` — 采集彩色点云

```python
points = cam.capture_pointcloud(max_depth=1.0)
# numpy (N, 6): [x, y, z, r, g, b]
# x/y/z 单位: 米
# 超过 max_depth 的点被过滤
```

### 5.9 `list_devices()` — 列出设备（静态方法）

```python
devices = RealSenseCamera.list_devices()
# [{'name': 'Intel RealSense D405', 'serial': '409122272399', 'firmware': '5.15.1.55'}, ...]
```

### 5.10 上下文管理器

```python
# 推荐：自动管理生命周期
with RealSenseCamera(serial='409122272399') as cam:
    color, depth = cam.capture()
# 退出 with 时自动 stop()
```

### 5.11 同时使用两台 D405

```python
cam1 = RealSenseCamera(serial='409122272399')
cam2 = RealSenseCamera(serial='409122273344')

cam1.start()
cam2.start()

color1, depth1 = cam1.capture()
color2, depth2 = cam2.capture()

cam1.stop()
cam2.stop()
```

> 注意：不要同时启动超过 USB 带宽限制的组合。两台 D405 在 1280x720@6fps 下可同时工作。

---

## 6. ZED 2i API

> 源文件：`camera_driver/camera_driver/zed_camera.py`

### 6.1 SDK 模式 (Jetson Orin)

ZED SDK 已安装在 Jetson Orin 上，运行在 **SDK 模式**：

| 功能 | 状态 |
|------|------|
| 彩色图 | 1280x720 (HD720) @ 15fps (录制保存2Hz) |
| 深度图 | NEURAL 深度模式，float32 (mm) |
| 点云 | XYZRGBA float16 (~7MB/帧, 2Hz录制, 全分辨率, 可配降采样+dtype) |
| 右眼 | 1280x720 (HD720), 与左眼同步采集 |
| IMU | SDK + grab原生频率 (~15Hz, 录制时); SDK + 独立线程 (~15-20Hz, 待机时) |

> 当 SDK 不可用时自动降级到 OpenCV V4L2 模式 (672x376)。

### 6.2 构造函数

```python
ZEDCamera(
    resolution='HD720',    # 分辨率: HD2K/HD1080/HD720/VGA
    fps=30,                # 帧率
    depth_mode='NEURAL',   # 仅 SDK 模式生效
    min_depth=100.0,       # 最小深度 mm，仅 SDK 模式生效
    force_opencv=False,    # 强制 OpenCV 模式
)
```

### 6.3 `capture()` — 采集一帧

```python
left, depth = zed.capture()
# SDK模式: left: numpy (720, 1280, 3) BGR uint8, depth: numpy (720, 1280) float32 (mm)
# OpenCV模式: left: numpy (376, 672, 3) BGR uint8, depth: numpy (376, 672) uint16 mm
# 失败返回 (None, None)
```

> SDK 模式下内部复用 `sl.Mat` 对象 (`_img_mat`, `_depth_mat`)，避免每帧 C++ 堆分配开销。

### 6.4 `capture_stereo()` — 采集左右眼 + 深度

```python
left, right, depth = zed.capture_stereo()
# left:  (376, 672, 3) BGR uint8
# right: (376, 672, 3) BGR uint8
# depth: (376, 672) uint16 mm
```

### 6.5 `capture_and_save()` — 采集并保存

```python
left_path, depth_path = zed.capture_and_save(
    save_dir=None,
    name='zed_2i',
)
# 输出: zed_2i_{timestamp}_left.png, zed_2i_{timestamp}_depth.png
```

### 6.6 `capture_pointcloud()` — 采集点云（仅 SDK 模式）

```python
points = zed.capture_pointcloud()
# numpy (720, 1280, 4) float16 (录制保存, 可配置PC_DTYPE): X, Y, Z (mm) + RGBA 打包
# 返回 None 如果非 SDK 模式或 color_only 模式
```

> 内部复用 `_pc_mat` (sl.Mat)，避免每帧分配。

### 6.7 `get_imu_data()` — 获取 IMU 数据（SDK 模式）

```python
imu = zed.get_imu_data()
# dict: accel, gyro_dps, gyro_rad, mag, mag_valid, imu_temp, pressure, env_temp,
#        timestamp_ns, timestamp_s
# 返回 None 如果不可用
```

### 6.8 `get_intrinsics()` — 获取内参

```python
intrinsics = zed.get_intrinsics()
# 仅 SDK 模式可用，OpenCV 模式返回 None
```

### 6.9 上下文管理器

```python
with ZEDCamera() as zed:
    left, depth = zed.capture()
```

---

## 7. ZED 2i IMU API

> 源文件：`camera_driver/camera_driver/zed_imu.py`
>
> 双模式 IMU 驱动：SDK 优先（精度更高，与相机时间戳同步），USB HID 备用（无需 CUDA）。
> 基于 [stereolabs/zed-open-capture](https://github.com/stereolabs/zed-open-capture) 协议。

### 7.1 传感器规格

| 传感器 | 数据 | 单位 | 频率 |
|--------|------|------|------|
| 加速度计 | X, Y, Z | m/s² | 400 Hz |
| 陀螺仪 | X, Y, Z | deg/s, rad/s | 400 Hz |
| 磁力计 | X, Y, Z | µT | ~50 Hz |
| 温度 | IMU 芯片温度 | °C | 1 Hz |
| 环境传感器 | 气压、湿度 | hPa, % | 1 Hz |

### 7.2 构造函数

```python
ZEDIMU(
    hidraw_path=None,      # hidraw 设备路径，默认自动搜索
    ping_interval=400,     # ping 保活间隔（读取次数，~400=1秒）
    force_hid=False,       # 强制使用 USB HID 模式，跳过 SDK
)
```

### 7.3 `start()` — 启动 IMU 数据流

```python
imu = ZEDIMU()
imu.start()
# 自动选择模式: SDK 优先(若 pyzed 可用), 失败则降级 USB HID
# USB HID 模式自动搜索 /dev/hidraw* 找到 ZED 2i MCU 设备 (VID=2b03, PID=f881)
# 后台线程持续读取数据，自动 ping 保活
```

### 7.4 `stop()` — 停止数据流

```python
imu.stop()
# 关闭 HID 设备，停止后台线程
```

### 7.5 `read()` — 读取最新数据

```python
data = imu.read()
# 返回 dict，包含:
#   'valid':        bool, IMU 数据是否有效
#   'timestamp_ns': int, 纳秒时间戳
#   'timestamp_s':  float, 秒时间戳
#   'accel':        (float, float, float), 加速度 m/s²
#   'gyro_rad':     (float, float, float), 角速度 rad/s
#   'gyro_dps':     (float, float, float), 角速度 deg/s
#   'mag':          (float, float, float), 磁力 µT
#   'mag_valid':    int, 磁力计数据状态 (0=无, 1=旧, 2=新)
#   'imu_temp':     float, IMU 温度 °C
#   'env_valid':    int, 环境传感器有效标志
#   'env_temp':     float, 环境温度 °C
#   'pressure':     float, 气压 hPa
#   'humidity':     float, 湿度 %
#   'frame_sync':   int, 帧同步标志
# 无数据时返回 None
```

### 7.6 `find_hidraw()` — 查找设备（静态方法）

```python
path = ZEDIMU.find_hidraw()
# 返回 '/dev/hidraw3' 或 None
```

### 7.7 上下文管理器

```python
with ZEDIMU() as imu:
    data = imu.read()
    if data:
        print(f"加速度: {data['accel']}")
        print(f"角速度: {data['gyro_dps']} deg/s")
# 退出 with 时自动 stop()
```

### 7.8 完整示例

```python
import sys
sys.path.insert(0, '/home/svt/svtrobo_ws')
from camera_driver.camera_driver import ZEDIMU
import time

with ZEDIMU() as imu:
    time.sleep(0.5)  # 等待首批数据
    for i in range(20):
        data = imu.read()
        if data:
            a = data['accel']
            g = data['gyro_dps']
            print(f'Accel: X={a[0]:+.4f} Y={a[1]:+.4f} Z={a[2]:+.4f}  '
                  f'Gyro: X={g[0]:+.3f} Y={g[1]:+.3f} Z={g[2]:+.3f}')
        time.sleep(0.05)
```

### 7.9 ROS2 IMU 节点

> 源文件：`camera_driver/camera_driver/zed_imu_node.py`

**发布话题：**

| 话题 | 消息类型 | 说明 | 频率 |
|------|---------|------|------|
| `~/zed/imu/data` | `sensor_msgs/Imu` | 加速度 + 角速度 | ~100 Hz |
| `~/zed/imu/mag` | `sensor_msgs/MagneticField` | 磁力计 | ~50 Hz |
| `~/zed/imu/temperature` | `sensor_msgs/Temperature` | IMU 温度 | ~1 Hz |

**参数：**

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `frame_id` | str | `'zed_imu_link'` | TF frame ID |
| `publish_mag` | bool | `True` | 是否发布磁力计 |
| `publish_temp` | bool | `True` | 是否发布温度 |
| `publish_env` | bool | `False` | 是否发布气压/湿度 |
| `force_hid` | bool | `False` | 强制使用 USB HID 模式（跳过 SDK） |

**启动：**

```bash
python3 src/camera_driver/camera_driver/zed_imu_node.py
```

**前提：** 需要 udev 规则允许用户空间访问 hidraw 设备（见第 3 节）。

---

## 8. ROS2 相机节点

除了 Python 采集 API，camera_driver 还提供了 ROS2 节点，可直接发布 sensor_msgs/Image 和 CameraInfo 话题。

### 8.1 RealSense D405 ROS2 节点

> 源文件：`camera_driver/camera_driver/realsense_node.py`

**发布话题：**

| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `{namespace}/color/image_raw` | `sensor_msgs/Image` | 彩色图 (BGR8) |
| `{namespace}/depth/image_raw` | `sensor_msgs/Image` | 深度图 (16UC1, 对齐到彩色) |
| `{namespace}/color/camera_info` | `sensor_msgs/CameraInfo` | 彩色相机内参 |
| `{namespace}/depth/camera_info` | `sensor_msgs/CameraInfo` | 深度相机内参 |

**参数：**

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `serial_number` | str | `''` | 设备序列号，空则选第一个 |
| `namespace` | str | `'d405_1'` | 话题命名空间 |
| `color_width` | int | `1280` | 彩色图宽度 |
| `color_height` | int | `720` | 彩色图高度 |
| `depth_width` | int | `1280` | 深度图宽度 |
| `depth_height` | int | `720` | 深度图高度 |
| `fps` | int | `6` | 帧率 |
| `frame_id` | str | `'camera_link'` | TF frame ID |

**启动：**

```bash
ros2 run camera_driver realsense_node --ros-args \
  -p serial_number:=409122272399 \
  -p namespace:=d405_1
```

### 8.2 ZED 2i ROS2 节点

> 源文件：`camera_driver/camera_driver/zed_node.py`

双模式：SDK 优先（需 NVIDIA GPU），OpenCV 降级（SGBM 估算深度）。

**发布话题：**

| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `zed/left/image_raw` | `sensor_msgs/Image` | 左眼彩色图 (BGR8) |
| `zed/right/image_raw` | `sensor_msgs/Image` | 右眼彩色图 (BGR8，仅 OpenCV 模式) |
| `zed/depth/image_raw` | `sensor_msgs/Image` | 深度图 (16UC1) |
| `zed/left/camera_info` | `sensor_msgs/CameraInfo` | 左眼相机内参（仅 SDK 模式） |

**参数：**

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `resolution` | str | `'HD720'` | 分辨率：HD2K/HD1080/HD720/VGA |
| `fps` | int | `30` | 帧率 |
| `depth_mode` | str | `'NEURAL'` | SDK 深度模式：NEURAL/ULTRA/QUALITY/PERFORMANCE |
| `min_depth` | float | `100.0` | 最小深度 mm（仅 SDK） |
| `frame_id` | str | `'zed_link'` | TF frame ID |
| `force_opencv` | bool | `False` | 强制使用 OpenCV 模式 |

**启动：**

```bash
ros2 run camera_driver zed_node --ros-args \
  -p resolution:=HD720 -p depth_mode:=NEURAL
```

---

## 9. 文件结构

```
svtrobo_ws/
└── camera_driver/
    ├── camera_driver/
    │   ├── __init__.py              # 模块入口，导出 RealSenseCamera, ZEDCamera, ZEDIMU
    │   ├── realsense_camera.py      # D405 采集模块
    │   ├── realsense_node.py        # D405 ROS2 发布节点
    │   ├── zed_camera.py            # ZED 2i 采集模块
    │   ├── zed_node.py              # ZED 2i ROS2 发布节点
    │   ├── zed_imu.py               # ZED 2i IMU 传感器驱动（USB HID）
    │   └── zed_imu_node.py          # ZED 2i IMU ROS2 发布节点
    └── captures/                    # 默认保存目录
        ├── d405_1_*_color.png       # D405 #1 彩色图
        ├── d405_1_*_depth.png       # D405 #1 深度图
        ├── d405_2_*_color.png       # D405 #2 彩色图
        ├── d405_2_*_depth.png       # D405 #2 深度图
        ├── zed_2i_*_left.png        # ZED 左眼图像
        └── zed_2i_*_depth.png       # ZED 深度图
```

---

## 10. 注意事项与常见问题

### Q1: D405 启动报错 `Couldn't resolve requests`

分辨率/fps 组合不兼容。参考第 2.2 节支持的分辨率表，确保彩色和深度流使用相同 fps。

### Q2: 两台 D405 同时使用时帧率下降

USB 带宽限制。降低分辨率或帧率：

```python
cam = RealSenseCamera(serial='...', color_size=(1280, 720), depth_size=(1280, 720), fps=6)
```

### Q3: ZED 启动报错 `未找到 ZED 相机设备`

检查 ZED 是否被其他程序占用：

```bash
# 检查设备
cat /sys/class/video4linux/video0/name
# 如果不是 ZED，检查 USB 连接
lsusb | grep STEREOLABS
```

### Q4: 深度图看起来全黑

D405 深度图是 uint16 格式，普通图片查看器可能显示为全黑。用代码查看实际数值：

```python
import cv2
depth = cv2.imread('depth.png', cv2.IMREAD_UNCHANGED)
print(f'范围: [{depth.min()}, {depth.max()}]')
```

或转为伪彩色可视化：

```python
depth_color = cv2.applyColorMap(
    cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)
cv2.imwrite('depth_visual.png', depth_color)
```

### Q5: 如何在有 GPU 的机器上使用 ZED SDK？

安装 ZED SDK 后，代码自动检测并使用 SDK 模式，无需修改代码：

```bash
# 下载并安装 ZED SDK（需要 NVIDIA GPU + CUDA）
# https://www.stereolabs.com/developers/release/
pip3 install pyzed
```

SDK 模式自动获得：高分辨率深度、NEURAL 深度模式、点云等。
> 注：IMU 数据已通过 USB HID 接口直接读取，无需 SDK。

### Q6: 如何修改默认保存路径？

```python
cam.capture_and_save(save_dir='/your/custom/path', name='d405_1')
```

### Q7: 相机预热时间长吗？

预热丢弃 30 帧，约 1-6 秒（取决于帧率）。如果不需要稳定曝光，可以修改 `realsense_camera.py` 中的预热帧数。

### Q8: IMU 启动报错 `未找到 ZED 2i IMU HID 设备`

检查 hidraw 设备和权限：

```bash
# 查看 ZED HID 设备是否存在
lsusb | grep -i stereolabs
# 应看到: ID 2b03:f881 STEREOLABS ZED-2i HID INTERFACE

# 检查 hidraw 设备权限
ls -la /dev/hidraw*
# 应该是 crw-rw-rw-（0666），如果是 crw------- 需要安装 udev 规则：
echo 'SUBSYSTEM=="hidraw", ATTRS{idVendor}=="2b03", ATTRS{idProduct}=="f881", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/99-zed-imu.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Q9: IMU 数据突然停止更新

IMU 需要周期性 ping 保持数据流（驱动已内置自动 ping）。如果 USB 连接不稳定导致设备断开，需重新调用 `start()`。
