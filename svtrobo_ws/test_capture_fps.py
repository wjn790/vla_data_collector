#!/usr/bin/env python3
"""测试 ZED 相机各采集通道的实际帧率"""

import sys
import time
import threading
import json
import numpy as np

sys.path.insert(0, 'src/camera_driver')
from camera_driver import ZEDCamera

# === 测试参数 ===
TEST_DURATION = 10  # 每项测试持续秒数
ZED_RESOLUTION = 'HD720'
ZED_FPS = 15

results = {}

def test_zed_color_only():
    """测试 ZED 纯彩色模式采集帧率"""
    print("\n" + "="*60)
    print("测试1: ZED color-only 模式 (左眼)")
    print("="*60)

    with ZEDCamera(resolution=ZED_RESOLUTION, fps=ZED_FPS, color_only=True) as zed:
        count = 0
        shapes = []
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < TEST_DURATION:
            left, depth = zed.capture()
            if left is not None:
                count += 1
                if count == 1:
                    shapes.append(left.shape)
        elapsed = time.perf_counter() - t0
        fps = count / elapsed
        results['zed_color_only'] = {
            'fps': round(fps, 2),
            'frames': count,
            'duration': round(elapsed, 2),
            'target_fps': ZED_FPS,
            'shape': list(shapes[0]) if shapes else None,
        }
        print(f"  采集帧数: {count}, 耗时: {elapsed:.2f}s")
        print(f"  实际帧率: {fps:.2f} FPS (目标: {ZED_FPS} FPS)")
        print(f"  图像尺寸: {shapes[0] if shapes else 'N/A'}")
        print(f"  {'✓ 满足要求' if fps >= ZED_FPS * 0.9 else '✗ 未达标'}")


def test_zed_stereo_depth():
    """测试 ZED 立体 + 深度模式 (左+右+深度+点云)"""
    print("\n" + "="*60)
    print("测试2: ZED stereo+depth 模式 (左眼+右眼+深度+点云)")
    print("="*60)

    with ZEDCamera(resolution=ZED_RESOLUTION, fps=ZED_FPS,
                   color_only=False, depth_mode='NEURAL') as zed:
        count = 0
        pc_count = 0
        shapes = {'left': None, 'right': None, 'depth': None, 'pc': None}
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < TEST_DURATION:
            left, right, depth = zed.capture_stereo()
            if left is not None:
                count += 1
                if count == 1:
                    shapes['left'] = left.shape
                    shapes['right'] = right.shape if right is not None else None
                    shapes['depth'] = depth.shape if depth is not None else None
                # Point cloud
                pc = zed.capture_pointcloud()
                if pc is not None:
                    pc_count += 1
                    if pc_count == 1:
                        shapes['pc'] = pc.shape
        elapsed = time.perf_counter() - t0
        fps = count / elapsed
        pc_fps = pc_count / elapsed
        results['zed_stereo_depth'] = {
            'fps': round(fps, 2),
            'pc_fps': round(pc_fps, 2),
            'frames': count,
            'pc_frames': pc_count,
            'duration': round(elapsed, 2),
            'target_fps': ZED_FPS,
            'shapes': {k: list(v) if v else None for k, v in shapes.items()},
        }
        print(f"  帧采集数: {count}, 点云采集数: {pc_count}, 耗时: {elapsed:.2f}s")
        print(f"  帧率: {fps:.2f} FPS (目标: {ZED_FPS} FPS)")
        print(f"  点云帧率: {pc_fps:.2f} FPS")
        print(f"  左眼: {shapes['left']}, 右眼: {shapes['right']}")
        print(f"  深度: {shapes['depth']}, 点云: {shapes['pc']}")
        print(f"  {'✓ 满足要求' if fps >= ZED_FPS * 0.9 else '✗ 未达标'}")


def test_zed_imu():
    """测试 ZED IMU 采集频率"""
    print("\n" + "="*60)
    print("测试3: ZED IMU 数据采集频率")
    print("="*60)

    with ZEDCamera(resolution=ZED_RESOLUTION, fps=ZED_FPS, color_only=True) as zed:
        count = 0
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < TEST_DURATION:
            imu = zed.get_imu_data()
            if imu is not None:
                count += 1
            time.sleep(0.001)  # 1ms 最小间隔
        elapsed = time.perf_counter() - t0
        hz = count / elapsed
        results['imu'] = {
            'hz': round(hz, 2),
            'samples': count,
            'duration': round(elapsed, 2),
            'target_hz': '70 (SDK native)',
        }
        print(f"  采集样本数: {count}, 耗时: {elapsed:.2f}s")
        print(f"  实际频率: {hz:.2f} Hz")
        print(f"  {'✓ 满足要求' if hz >= 50 else '✗ 未达标'}")


def test_recording_2hz():
    """模拟 server.py 中 RecordingManager 的 2Hz deadline 调度"""
    print("\n" + "="*60)
    print("测试4: 模拟 RecordingManager 2Hz deadline 保存帧率")
    print("="*60)

    RECORD_INTERVAL = 0.5  # 与 server.py 一致
    TEST_CAMERAS = ['zed']

    with ZEDCamera(resolution=ZED_RESOLUTION, fps=ZED_FPS,
                   color_only=False, depth_mode='NEURAL') as zed:
        _start_time = time.monotonic()
        _next_deadline = {name: _start_time + RECORD_INTERVAL for name in TEST_CAMERAS}
        saved = {name: 0 for name in TEST_CAMERAS}
        pc_saved = 0

        t0 = time.perf_counter()
        while time.perf_counter() - t0 < TEST_DURATION:
            _now = time.monotonic()
            for name in TEST_CAMERAS:
                if _now < _next_deadline[name]:
                    # 模拟采集一帧
                    left, right, depth = zed.capture_stereo()
                    pc = zed.capture_pointcloud()
                    continue

                # deadline 到了，保存
                left, right, depth = zed.capture_stereo()
                if left is not None:
                    saved[name] += 1
                    pc = zed.capture_pointcloud()
                    if pc is not None:
                        pc_saved += 1

                _next_deadline[name] += RECORD_INTERVAL
                if _next_deadline[name] < _now:
                    _next_deadline[name] = _now + RECORD_INTERVAL

            time.sleep(0.001)

        elapsed = time.perf_counter() - t0
        fps = saved['zed'] / elapsed
        pc_fps = pc_saved / elapsed
        results['recording_2hz'] = {
            'frame_fps': round(fps, 2),
            'pc_fps': round(pc_fps, 2),
            'frames_saved': saved['zed'],
            'pc_saved': pc_saved,
            'duration': round(elapsed, 2),
            'target_fps': 2.0,
        }
        print(f"  保存帧数: {saved['zed']}, 点云帧数: {pc_saved}, 耗时: {elapsed:.2f}s")
        print(f"  实际保存帧率: {fps:.2f} FPS (目标: 2.0 FPS)")
        print(f"  点云保存帧率: {pc_fps:.2f} FPS")
        print(f"  {'✓ 满足要求' if abs(fps - 2.0) < 0.3 else '✗ 未达标'}")


def test_streaming_throughput():
    """测试 MJPEG 编码吞吐量 (server.py STREAM_FPS=15)"""
    print("\n" + "="*60)
    print("测试5: MJPEG 编码吞吐量 (模拟 STREAM_FPS=15 推流)")
    print("="*60)

    import cv2

    JPEG_QUALITY = 95
    STREAM_FPS = 15

    with ZEDCamera(resolution=ZED_RESOLUTION, fps=ZED_FPS,
                   color_only=False, depth_mode='NEURAL') as zed:
        encoded = 0
        total_bytes = 0
        encode_times = []
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < TEST_DURATION:
            left, right, depth = zed.capture_stereo()
            if left is not None:
                t_enc_start = time.perf_counter()
                _, jpeg = cv2.imencode('.jpg', left, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                encode_times.append(time.perf_counter() - t_enc_start)
                total_bytes += len(jpeg)
                encoded += 1
            time.sleep(1.0 / STREAM_FPS)
        elapsed = time.perf_counter() - t0
        fps = encoded / elapsed
        avg_size_kb = (total_bytes / max(encoded, 1)) / 1024
        avg_enc_ms = (sum(encode_times) / max(len(encode_times), 1)) * 1000
        results['streaming'] = {
            'fps': round(fps, 2),
            'frames_encoded': encoded,
            'avg_size_kb': round(avg_size_kb, 1),
            'avg_encode_ms': round(avg_enc_ms, 2),
            'duration': round(elapsed, 2),
            'target_fps': STREAM_FPS,
        }
        print(f"  编码帧数: {encoded}, 耗时: {elapsed:.2f}s")
        print(f"  实际推流帧率: {fps:.2f} FPS (目标: {STREAM_FPS} FPS)")
        print(f"  平均帧大小: {avg_size_kb:.1f} KB")
        print(f"  平均编码耗时: {avg_enc_ms:.2f} ms")
        print(f"  {'✓ 满足要求' if fps >= STREAM_FPS * 0.9 else '✗ 未达标'}")


if __name__ == '__main__':
    print(f"ZED 帧率测试 (分辨率: {ZED_RESOLUTION}, 目标FPS: {ZED_FPS})")
    print(f"每项测试持续 {TEST_DURATION} 秒\n")

    try:
        test_zed_color_only()
    except Exception as e:
        print(f"  测试失败: {e}")
        results['zed_color_only'] = {'error': str(e)}

    try:
        test_zed_stereo_depth()
    except Exception as e:
        print(f"  测试失败: {e}")
        results['zed_stereo_depth'] = {'error': str(e)}

    try:
        test_zed_imu()
    except Exception as e:
        print(f"  测试失败: {e}")
        results['imu'] = {'error': str(e)}

    try:
        test_recording_2hz()
    except Exception as e:
        print(f"  测试失败: {e}")
        results['recording_2hz'] = {'error': str(e)}

    try:
        test_streaming_throughput()
    except Exception as e:
        print(f"  测试失败: {e}")
        results['streaming'] = {'error': str(e)}

    # 汇总
    print("\n" + "="*60)
    print("帧率测试汇总")
    print("="*60)
    for name, data in results.items():
        if 'error' in data:
            print(f"  {name}: 失败 - {data['error']}")
        elif 'fps' in data:
            target = data.get('target_fps', data.get('target_hz', '?'))
            ok = data['fps'] >= (float(target) * 0.9 if isinstance(target, (int, float)) else 50)
            status = '✓' if ok else '✗'
            print(f"  {name}: {data['fps']} {'FPS' if 'target_fps' in data else 'Hz'} (目标: {target}) {status}")
        elif 'frame_fps' in data:
            ok = abs(data['frame_fps'] - data['target_fps']) < 0.3
            print(f"  {name}: {data['frame_fps']} FPS (目标: {data['target_fps']} FPS) {'✓' if ok else '✗'}")

    # 保存结果
    with open('test_fps_results.json', 'w') as f:
        json.dump(results, f, indent=2, default=str)
    print(f"\n详细结果已保存到 test_fps_results.json")
