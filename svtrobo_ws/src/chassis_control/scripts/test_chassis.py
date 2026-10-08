#!/usr/bin/env python3
"""
SVTROBO 底盘交互式测试工具

通过终端菜单控制底盘运动、查询状态、测试升降机构。

使用方法:
    python3 test_chassis.py
"""

import sys
import time
import threading

sys.path.insert(0, "/home/openarm/svtrobo_ws/src/chassis_control/scripts")
from svtrobo_controller import SVTROBOController


# ══════════════════════════════════════════════════════════
#  全局状态
# ══════════════════════════════════════════════════════════

g_robot: SVTROBOController = None
g_speed = 0.1       # 当前线速度 (m/s)
g_rot_speed = 0.2   # 当前角速度 (rad/s)
g_lift_speed = 300   # 升降速度 (RPM)
g_monitoring = False
g_monitor_thread = None


# ══════════════════════════════════════════════════════════
#  工具函数
# ══════════════════════════════════════════════════════════

def print_separator():
    print("=" * 60)


def print_header(title):
    print_separator()
    print(f"  {title}")
    print_separator()


def input_float(prompt, default=None):
    """读取浮点数输入，支持默认值。"""
    if default is not None:
        raw = input(f"{prompt} [{default}]: ").strip()
        if raw == "":
            return default
    else:
        raw = input(f"{prompt}: ").strip()
        if raw == "":
            return None
    try:
        return float(raw)
    except ValueError:
        print(f"  无效输入: '{raw}'")
        return default


def confirm(prompt):
    """确认操作。"""
    raw = input(f"{prompt} (y/N): ").strip().lower()
    return raw in ("y", "yes")


def monitor_loop():
    """后台状态监控线程。"""
    global g_monitoring
    while g_monitoring:
        state = g_robot.get_state()
        if state:
            angles = [f"{a:+.3f}" for a in state["steer_angles"]]
            speeds = [f"{s:+.1f}" for s in state["wheel_speeds"]]
            cmd = f"({state.get('vx_set', 0):.2f}, {state.get('vy_set', 0):.2f}, {state.get('wz_set', 0):.2f})"
            print(f"\r  [状态] 角度:{angles} 转速:{speeds} 指令:{cmd}   ", end="", flush=True)
        time.sleep(0.1)
    print()  # 换行


# ══════════════════════════════════════════════════════════
#  功能菜单
# ══════════════════════════════════════════════════════════

def menu_query_state():
    """查询并显示当前底盘状态。"""
    print_header("底盘状态查询")
    state = g_robot.get_state()
    if state is None:
        print("  尚未收到状态数据，请稍后再试。")
        return

    print(f"  时间戳:         {state['stamp'].sec}.{state['stamp'].nanosec}")
    print()
    print(f"  舵向角度 (rad):  FL={state['steer_angles'][0]:+.4f}  "
          f"FR={state['steer_angles'][1]:+.4f}  "
          f"RL={state['steer_angles'][2]:+.4f}  "
          f"RR={state['steer_angles'][3]:+.4f}")
    print(f"  舵向速度 (rad/s): FL={state['steer_velocities'][0]:+.4f}  "
          f"FR={state['steer_velocities'][1]:+.4f}  "
          f"RL={state['steer_velocities'][2]:+.4f}  "
          f"RR={state['steer_velocities'][3]:+.4f}")
    print(f"  舵向力矩 (Nm):   FL={state['steer_torques'][0]:+.4f}  "
          f"FR={state['steer_torques'][1]:+.4f}  "
          f"RL={state['steer_torques'][2]:+.4f}  "
          f"RR={state['steer_torques'][3]:+.4f}")
    print(f"  轮速 (RPM):      FL={state['wheel_speeds'][0]:+.2f}  "
          f"FR={state['wheel_speeds'][1]:+.2f}  "
          f"RL={state['wheel_speeds'][2]:+.2f}  "
          f"RR={state['wheel_speeds'][3]:+.2f}")
    print()
    print(f"  当前指令:  vx={state.get('vx_set', 0):.3f} m/s  "
          f"vy={state.get('vy_set', 0):.3f} m/s  "
          f"wz={state.get('wz_set', 0):.3f} rad/s")
    print(f"  连接状态:  {g_robot.is_connected}")


def menu_realtime_monitor():
    """实时状态监控（持续刷新）。"""
    global g_monitoring, g_monitor_thread
    print_header("实时状态监控")
    print("  按回车键停止监控并返回菜单。")
    print()

    g_monitoring = True
    g_monitor_thread = threading.Thread(target=monitor_loop, daemon=True)
    g_monitor_thread.start()

    input()  # 等待回车
    g_monitoring = False
    g_monitor_thread.join(timeout=2.0)


def menu_move():
    """运动控制。"""
    global g_speed
    print_header("运动控制")
    print(f"  当前速度: {g_speed} m/s")
    print()
    print("  1) 前进")
    print("  2) 后退")
    print("  3) 左移")
    print("  4) 右移")
    print("  5) 逆时针旋转")
    print("  6) 顺时针旋转")
    print("  7) 自定义 move(vx, vy, wz)")
    print("  8) 修改速度")
    print("  0) 返回")

    choice = input("\n  选择: ").strip()
    if choice == "1":
        duration = input_float("  持续时间(秒)", 2.0)
        print(f"  前进 {g_speed} m/s，{duration} 秒...")
        g_robot.move_forward(g_speed)
        time.sleep(duration)
        g_robot.stop()
        print("  已停止。")
    elif choice == "2":
        duration = input_float("  持续时间(秒)", 2.0)
        print(f"  后退 {g_speed} m/s，{duration} 秒...")
        g_robot.move_backward(g_speed)
        time.sleep(duration)
        g_robot.stop()
        print("  已停止。")
    elif choice == "3":
        duration = input_float("  持续时间(秒)", 2.0)
        print(f"  左移 {g_speed} m/s，{duration} 秒...")
        g_robot.move_left(g_speed)
        time.sleep(duration)
        g_robot.stop()
        print("  已停止。")
    elif choice == "4":
        duration = input_float("  持续时间(秒)", 2.0)
        print(f"  右移 {g_speed} m/s，{duration} 秒...")
        g_robot.move_right(g_speed)
        time.sleep(duration)
        g_robot.stop()
        print("  已停止。")
    elif choice == "5":
        duration = input_float("  持续时间(秒)", 2.0)
        print(f"  逆时针旋转 {g_rot_speed} rad/s，{duration} 秒...")
        g_robot.rotate(g_rot_speed)
        time.sleep(duration)
        g_robot.stop()
        print("  已停止。")
    elif choice == "6":
        duration = input_float("  持续时间(秒)", 2.0)
        print(f"  顺时针旋转 {g_rot_speed} rad/s，{duration} 秒...")
        g_robot.rotate(-g_rot_speed)
        time.sleep(duration)
        g_robot.stop()
        print("  已停止。")
    elif choice == "7":
        vx = input_float("  vx (m/s)", 0.0)
        vy = input_float("  vy (m/s)", 0.0)
        wz = input_float("  wz (rad/s)", 0.0)
        duration = input_float("  持续时间(秒)", 2.0)
        print(f"  move({vx}, {vy}, {wz})，{duration} 秒...")
        g_robot.move(vx, vy, wz)
        time.sleep(duration)
        g_robot.stop()
        print("  已停止。")
    elif choice == "8":
        new_speed = input_float("  新的默认速度 (m/s)", g_speed)
        if new_speed is not None and new_speed > 0:
            g_speed = new_speed
            print(f"  默认速度已设为 {g_speed} m/s")
        else:
            print("  速度必须为正数")


def menu_lift():
    """升降机构控制。"""
    global g_lift_speed
    print_header("升降机构控制")
    print(f"  当前升降速度: {g_lift_speed} RPM")
    print()
    print("  1) 正转（上升）")
    print("  2) 反转（下降）")
    print("  3) 停止")
    print("  4) 修改速度")
    print("  0) 返回")

    choice = input("\n  选择: ").strip()
    if choice == "1":
        duration = input_float("  持续时间(秒)", 3.0)
        print(f"  正转 {g_lift_speed} RPM，{duration} 秒...")
        g_robot.control_lift(1, g_lift_speed)
        time.sleep(duration)
        g_robot.stop_lift()
        print("  已停止。")
    elif choice == "2":
        duration = input_float("  持续时间(秒)", 3.0)
        print(f"  反转 {g_lift_speed} RPM，{duration} 秒...")
        g_robot.control_lift(-1, g_lift_speed)
        time.sleep(duration)
        g_robot.stop_lift()
        print("  已停止。")
    elif choice == "3":
        g_robot.stop_lift()
        print("  已发送停止指令。")
    elif choice == "4":
        new_speed = input_float("  新的升降速度 (RPM, 1~6000)", g_lift_speed)
        if new_speed is not None and 1 <= new_speed <= 6000:
            g_lift_speed = int(new_speed)
            print(f"  升降速度已设为 {g_lift_speed} RPM")
        else:
            print("  速度范围: 1~6000 RPM")


def menu_observation():
    """深度学习观测向量测试。"""
    print_header("深度学习观测向量")
    obs = g_robot.get_observation()
    if obs is None:
        print("  无数据。")
        return

    print(f"  shape: {obs.shape}  dtype: {obs.dtype}")
    print()
    print(f"  [0:4]  舵向角度: {obs[0:4]}")
    print(f"  [4:8]  舵向速度: {obs[4:8]}")
    print(f"  [8:12] 舵向力矩: {obs[8:12]}")
    print(f"  [12:16] 轮子转速: {obs[12:16]}")
    print()
    print(f"  完整向量: {obs}")


def menu_continuous():
    """连续运动 — 按住方向持续运动，松开停止。"""
    print_header("连续运动模式 (WASDQE)")
    print("  W=前进  S=后退  A=左移  D=右移  Q=逆时针  E=顺时针")
    print("  空格=停止  X=退出")
    print(f"  速度: {g_speed} m/s  角速度: {g_rot_speed} rad/s")
    print()

    try:
        import termios
        import tty
        has_termios = True
    except ImportError:
        has_termios = False

    if not has_termios:
        print("  (非交互终端模式，输入字母后回车)")
        while True:
            cmd = input("  > ").strip().lower()
            if cmd == "w":
                g_robot.move_forward(g_speed)
            elif cmd == "s":
                g_robot.move_backward(g_speed)
            elif cmd == "a":
                g_robot.move_left(g_speed)
            elif cmd == "d":
                g_robot.move_right(g_speed)
            elif cmd == "q":
                g_robot.rotate(g_rot_speed)
            elif cmd == "e":
                g_robot.rotate(-g_rot_speed)
            elif cmd == " " or cmd == "":
                g_robot.stop()
            elif cmd == "x":
                g_robot.stop()
                break
        return

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        while True:
            ch = sys.stdin.read(1)
            if ch == "w":
                g_robot.move_forward(g_speed)
                sys.stdout.write("\r  前进   \r")
            elif ch == "s":
                g_robot.move_backward(g_speed)
                sys.stdout.write("\r  后退   \r")
            elif ch == "a":
                g_robot.move_left(g_speed)
                sys.stdout.write("\r  左移   \r")
            elif ch == "d":
                g_robot.move_right(g_speed)
                sys.stdout.write("\r  右移   \r")
            elif ch == "q":
                g_robot.rotate(g_rot_speed)
                sys.stdout.write("\r  逆时针 \r")
            elif ch == "e":
                g_robot.rotate(-g_rot_speed)
                sys.stdout.write("\r  顺时针 \r")
            elif ch == " ":
                g_robot.stop()
                sys.stdout.write("\r  停止   \r")
            elif ch == "x" or ch == "\x03" or ch == "\x1b":
                break
            sys.stdout.flush()
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        g_robot.stop()
        print("  退出连续模式。")


# ══════════════════════════════════════════════════════════
#  主菜单
# ══════════════════════════════════════════════════════════

def main():
    global g_robot, g_speed, g_rot_speed

    print_header("SVTROBO 底盘交互式测试工具")
    print("  正在连接底盘...")

    g_robot = SVTROBOController(auto_stop_timeout=1.0)
    g_robot.start()

    state = g_robot.wait_for_state(timeout=10.0)
    if state is None:
        print("\n  超时：未收到底盘状态数据。")
        print("  请确认：")
        print("    1. 底盘节点已启动 (ros2 launch chassis_control svtrobo_bringup.launch.py)")
        print("    2. C++ 代码已重新编译 (colcon build)")
        g_robot.shutdown()
        return

    print(f"  底盘已连接!")
    print(f"  舵向角度: {[f'{a:.3f}' for a in state['steer_angles']]}")
    print(f"  舵向速度: {[f'{v:.3f}' for v in state['steer_velocities']]}")
    print(f"  舵向力矩: {[f'{t:.3f}' for t in state['steer_torques']]}")
    print(f"  轮速:     {[f'{s:.1f}' for s in state['wheel_speeds']]}")

    while True:
        print()
        print_separator()
        print(f"  SVTROBO 测试菜单    速度={g_speed} m/s  角速度={g_rot_speed} rad/s")
        print_separator()
        print("  1) 查询状态（单次）")
        print("  2) 实时状态监控")
        print("  3) 运动控制（定时）")
        print("  4) 连续运动（WASDQE 键盘控制）")
        print("  5) 升降机构控制")
        print("  6) 观测向量测试（深度学习）")
        print("  7) 修改默认速度")
        print("  8) 紧急停止")
        print("  0) 退出")
        print_separator()

        choice = input("  选择: ").strip()

        if choice == "1":
            menu_query_state()
        elif choice == "2":
            menu_realtime_monitor()
        elif choice == "3":
            menu_move()
        elif choice == "4":
            menu_continuous()
        elif choice == "5":
            menu_lift()
        elif choice == "6":
            menu_observation()
        elif choice == "7":
            new_speed = input_float("  线速度 (m/s)", g_speed)
            new_rot = input_float("  角速度 (rad/s)", g_rot_speed)
            if new_speed is not None and new_speed > 0:
                g_speed = new_speed
            if new_rot is not None and new_rot > 0:
                g_rot_speed = new_rot
            print(f"  已更新: 速度={g_speed} m/s, 角速度={g_rot_speed} rad/s")
        elif choice == "8":
            g_robot.stop()
            g_robot.stop_lift()
            print("  已发送紧急停止指令（底盘 + 升降）。")
        elif choice == "0":
            break
        else:
            print(f"  无效选择: {choice}")

    g_robot.stop()
    g_robot.shutdown()
    print("  控制器已关闭，再见。")


if __name__ == "__main__":
    main()
