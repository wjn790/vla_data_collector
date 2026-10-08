"""
ZED 2i IMU 传感器驱动

双模式: SDK 优先, USB HID 备用

SDK 模式通过 ZED SDK 获取 IMU 数据（推荐，精度更高，与相机时间戳同步）。
USB HID 模式通过 hidraw 直接读取（无需 CUDA，作为降级方案）。

数据包含:
  - 加速度计 (m/s^2)
  - 陀螺仪 (deg/s, rad/s)
  - 磁力计 (uT)
  - 温度、气压、湿度

用法:
    from camera_driver.zed_imu import ZEDIMU

    imu = ZEDIMU()
    imu.start()
    data = imu.read()   # dict or None
    imu.stop()

    # 上下文管理器
    with ZEDIMU() as imu:
        data = imu.read()
"""

import math
import os
import threading
import time

try:
    import pyzed.sl as sl
    HAS_ZED_SDK = True
except ImportError:
    HAS_ZED_SDK = False

import fcntl
import select
import struct

# ---- ZED 2i HID 协议常量 (备用模式) ----
VID_ZED = 0x2b03
PID_ZED2i_MCU = 0xf881

REP_ID_SENSOR_DATA = 0x01
REP_ID_REQUEST_SET = 0x21
REP_ID_SENSOR_STREAM_STATUS = 0x32
RQ_CMD_PING = 0xF2

DEFAULT_GRAVITY = 9.8189
ACC_SCALE = DEFAULT_GRAVITY * (8.0 / 32768.0)
GYRO_SCALE_DPS = 1000.0 / 32768.0
GYRO_SCALE_RAD = GYRO_SCALE_DPS * (3.14159265 / 180.0)
MAG_SCALE = 1.0 / 16.0
TEMP_SCALE = 0.01
TS_SCALE = 39062.5


def _HIDIOCSFEATURE(n):
    return (3 << 30) | (n << 16) | (0x48 << 8) | 6


class ZEDIMU:
    """ZED 2i IMU 传感器管理类

    自动选择模式: SDK 优先, USB HID 备用。
    """

    def __init__(self, hidraw_path=None, ping_interval=400, force_hid=False):
        """
        Args:
            hidraw_path: hidraw 设备路径（仅 HID 模式），默认自动搜索
            ping_interval: HID 模式 ping 保活间隔
            force_hid: 强制使用 USB HID 模式
        """
        self._hidraw_path = hidraw_path
        self._ping_interval = ping_interval
        self._force_hid = force_hid

        self._mode = None  # 'sdk' or 'hid'
        self._running = False

        # SDK 模式对象
        self._zed = None

        # HID 模式对象
        self._fd = None
        self._read_count = 0

        # 最新数据 (线程安全)
        self._lock = threading.Lock()
        self._latest = None

        # 后台线程
        self._thread = None

    @property
    def mode(self):
        """当前模式: 'sdk', 'hid' 或 None"""
        return self._mode

    # ---- 自动查找 hidraw 设备 ----

    @staticmethod
    def find_hidraw():
        """查找 ZED 2i MCU 对应的 hidraw 设备路径"""
        import glob
        for path in sorted(glob.glob('/dev/hidraw*')):
            try:
                basename = os.path.basename(path)
                uevent_path = os.path.join('/sys/class/hidraw', basename, 'device/uevent')
                with open(uevent_path) as f:
                    content = f.read()
                expected = 'HID_ID=0003:{:08X}:{:08X}'.format(VID_ZED, PID_ZED2i_MCU)
                if expected in content:
                    return path
            except Exception:
                continue
        return None

    # ---- 启动/停止 ----

    def start(self):
        """启动 IMU 数据流（自动选择 SDK 或 HID 模式）"""
        if self._running:
            return

        if HAS_ZED_SDK and not self._force_hid:
            if self._start_sdk():
                self._running = True
                return
            print('[ZED-IMU] SDK 模式启动失败，尝试 USB HID 备用模式...')

        # 降级到 USB HID
        self._start_hid()
        self._running = True

    def stop(self):
        """停止 IMU 数据流"""
        if not self._running:
            return
        self._running = False

        if self._thread is not None:
            self._thread.join(timeout=3)
            self._thread = None

        if self._mode == 'sdk' and self._zed is not None:
            try:
                if self._zed.is_opened():
                    self._zed.close()
            except Exception:
                pass
            self._zed = None

        if self._mode == 'hid' and self._fd is not None:
            try:
                buf = bytearray(2)
                buf[0] = REP_ID_SENSOR_STREAM_STATUS
                buf[1] = 0x00
                fcntl.ioctl(self._fd, _HIDIOCSFEATURE(2), bytes(buf))
            except Exception:
                pass
            os.close(self._fd)
            self._fd = None

        self._mode = None
        print('[ZED-IMU] 已停止')

    # ---- SDK 模式 ----

    def _start_sdk(self):
        """SDK 模式启动: 打开 ZED 相机获取 IMU 数据"""
        try:
            self._zed = sl.Camera()
            params = sl.InitParameters()
            params.camera_resolution = sl.RESOLUTION.VGA
            params.camera_fps = 15
            params.depth_mode = sl.DEPTH_MODE.NONE
            params.sensors_required = True

            err = self._zed.open(params)
            if err != sl.ERROR_CODE.SUCCESS:
                print('[ZED-IMU] SDK 打开相机失败: {}'.format(err))
                self._zed = None
                return False

            # 启动后台轮询线程
            self._mode = 'sdk'
            self._running = True
            self._thread = threading.Thread(target=self._sdk_read_loop, daemon=True)
            self._thread.start()

            # 等待首帧数据
            time.sleep(0.3)

            sn = self._zed.get_camera_information().serial_number
            print('[ZED-IMU] SDK 模式启动, SN: {}'.format(sn))
            return True

        except Exception as e:
            print('[ZED-IMU] SDK 启动异常: {}'.format(e))
            self._zed = None
            return False

    def _sdk_read_loop(self):
        """SDK 模式后台读取线程"""
        sensors = sl.SensorsData()
        while self._running:
            try:
                if self._zed.grab(sl.RuntimeParameters()) == sl.ERROR_CODE.SUCCESS:
                    if self._zed.get_sensors_data(sensors, sl.TIME_REFERENCE.IMAGE) == sl.ERROR_CODE.SUCCESS:
                        imu = sensors.get_imu_data()
                        if imu.is_available:
                            data = self._parse_sdk_data(sensors)
                            if data:
                                with self._lock:
                                    self._latest = data
                else:
                    time.sleep(0.001)
            except Exception as e:
                if self._running:
                    print('[ZED-IMU] SDK 读取异常: {}'.format(e), flush=True)
                    time.sleep(0.01)

    def _parse_sdk_data(self, sensors):
        """从 SDK SensorsData 提取 IMU 数据 (SDK 5.x API)"""
        imu = sensors.get_imu_data()
        acc = imu.get_linear_acceleration()
        gyro = imu.get_angular_velocity()

        mag_data = sensors.get_magnetometer_data()
        barometer = sensors.get_barometer_data()
        temp_data = sensors.get_temperature_data()

        ts_ns = imu.timestamp.get_nanoseconds()

        # 磁力计
        if mag_data.is_available:
            mag = mag_data.get_magnetic_field_calibrated()
        else:
            mag = (0.0, 0.0, 0.0)

        # 温度
        try:
            imu_temp_raw = temp_data.get(sl.SENSOR_LOCATION.IMU)
            imu_temp = imu_temp_raw * 0.01 if imu_temp_raw and imu_temp_raw > 0 else 0.0
        except Exception:
            imu_temp = 0.0
        try:
            baro_temp_raw = temp_data.get(sl.SENSOR_LOCATION.BAROMETER)
            env_temp = baro_temp_raw * 0.01 if baro_temp_raw and baro_temp_raw > 0 else 0.0
        except Exception:
            env_temp = 0.0

        return {
            'valid': True,
            'timestamp_ns': ts_ns,
            'timestamp_s': ts_ns / 1e9,

            'accel': (acc[0], acc[1], acc[2]),
            'gyro_rad': (math.radians(gyro[0]), math.radians(gyro[1]), math.radians(gyro[2])),
            'gyro_dps': (gyro[0], gyro[1], gyro[2]),
            'mag': mag,
            'mag_valid': 1 if mag_data.is_available else 0,
            'imu_temp': imu_temp,
            'env_valid': 1 if barometer.is_available else 0,
            'env_temp': env_temp,
            'pressure': barometer.pressure if barometer.is_available else 0.0,
            'humidity': 0.0,
            'frame_sync': 0,
        }

    # ---- USB HID 模式 (备用) ----

    def _start_hid(self):
        """USB HID 模式启动"""
        if self._hidraw_path is None:
            self._hidraw_path = self.find_hidraw()
        if self._hidraw_path is None:
            raise RuntimeError('未找到 ZED 2i IMU 设备 (SDK 和 USB HID 均不可用)')

        self._fd = os.open(self._hidraw_path, os.O_RDWR)

        buf = bytearray(2)
        buf[0] = REP_ID_SENSOR_STREAM_STATUS
        buf[1] = 0x01
        fcntl.ioctl(self._fd, _HIDIOCSFEATURE(2), bytes(buf))

        self._mode = 'hid'
        self._read_count = 0

        self._thread = threading.Thread(target=self._hid_read_loop, daemon=True)
        self._thread.start()

        print('[ZED-IMU] USB HID 模式启动, 设备: {}'.format(self._hidraw_path))

    def _hid_read_loop(self):
        """HID 模式后台读取线程"""
        while self._running:
            try:
                ready, _, _ = select.select([self._fd], [], [], 2.0)
                if not ready:
                    continue

                data = os.read(self._fd, 64)
                if len(data) < 46 or data[0] != REP_ID_SENSOR_DATA:
                    continue

                parsed = self._parse_hid(data)
                if parsed:
                    with self._lock:
                        self._latest = parsed
                    self._read_count += 1

                    if self._read_count % self._ping_interval == 0:
                        self._send_ping()

            except OSError:
                if self._running:
                    print('[ZED-IMU] HID 读取错误，设备可能已断开')
                break
            except Exception as e:
                if self._running:
                    print('[ZED-IMU] HID 异常: {}'.format(e))

    def _send_ping(self):
        try:
            buf = bytearray(2)
            buf[0] = REP_ID_REQUEST_SET
            buf[1] = RQ_CMD_PING
            fcntl.ioctl(self._fd, _HIDIOCSFEATURE(2), bytes(buf))
        except Exception:
            pass

    @staticmethod
    def _parse_hid(data):
        """解析 HID 传感器报告"""
        if len(data) < 46:
            return None

        imu_valid = data[1] == 0
        timestamp_raw = struct.unpack_from('<Q', data, 2)[0]
        gx, gy, gz = struct.unpack_from('<3h', data, 10)
        ax, ay, az = struct.unpack_from('<3h', data, 16)
        frame_sync = data[22]
        imu_temp = struct.unpack_from('<h', data, 28)[0] if len(data) >= 30 else 0

        mag_valid = data[30] if len(data) > 30 else 0
        mx, my, mz = (0, 0, 0)
        if len(data) >= 37:
            mx, my, mz = struct.unpack_from('<3h', data, 31)

        env_valid = data[46] if len(data) > 46 else 0
        env_temp = struct.unpack_from('<h', data, 47)[0] if len(data) >= 49 else 0
        pressure = struct.unpack_from('<I', data, 49)[0] if len(data) >= 53 else 0
        humidity = struct.unpack_from('<I', data, 53)[0] if len(data) >= 57 else 0

        ts_ns = timestamp_raw * TS_SCALE

        return {
            'valid': imu_valid,
            'timestamp_ns': ts_ns,
            'timestamp_s': ts_ns / 1e9,
            'accel': (ax * ACC_SCALE, ay * ACC_SCALE, az * ACC_SCALE),
            'gyro_rad': (gx * GYRO_SCALE_RAD, gy * GYRO_SCALE_RAD, gz * GYRO_SCALE_RAD),
            'gyro_dps': (gx * GYRO_SCALE_DPS, gy * GYRO_SCALE_DPS, gz * GYRO_SCALE_DPS),
            'mag': (mx * MAG_SCALE, my * MAG_SCALE, mz * MAG_SCALE),
            'mag_valid': mag_valid,
            'imu_temp': imu_temp * TEMP_SCALE,
            'env_valid': env_valid,
            'env_temp': env_temp * TEMP_SCALE,
            'pressure': pressure * 0.0001,
            'humidity': humidity * 0.01,
            'frame_sync': frame_sync,
        }

    # ---- 读取接口 ----

    def read(self):
        """获取最新一次 IMU 采样数据"""
        with self._lock:
            return self._latest

    # ---- 上下文管理器 ----

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.stop()
        return False


if __name__ == '__main__':
    mode_str = 'SDK+HID(auto)' if HAS_ZED_SDK else 'HID only'
    print('ZED 2i IMU 测试 (模式: {}, Ctrl+C 退出)'.format(mode_str))
    print('-' * 60)

    with ZEDIMU() as imu:
        print('实际模式: {}'.format(imu.mode))
        time.sleep(0.5)
        for i in range(30):
            data = imu.read()
            if data:
                a = data['accel']
                g = data['gyro_dps']
                print('[{:3d}] Accel(m/s2): X={:+8.4f} Y={:+8.4f} Z={:+8.4f}  '
                      'Gyro(dps): X={:+7.3f} Y={:+7.3f} Z={:+7.3f}  '
                      'T={:.1f}C  mode={}'.format(i, a[0], a[1], a[2], g[0], g[1], g[2], data['imu_temp'], imu.mode))
            else:
                print('[{:3d}] 无数据'.format(i))
            time.sleep(0.05)