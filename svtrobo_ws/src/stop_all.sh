#!/bin/bash
# SVTROBO 一键停止所有服务 (通过 systemd)

set -e

SUDO="echo '<sudo密码，在真机上填写>' | sudo -S"

echo "======================================"
echo "  SVTROBO 一键停止 (systemd)"
echo "======================================"

# 先发送停止指令，确保机器人安全停下
echo "发送安全停止指令..."
source /opt/ros/humble/setup.bash 2>/dev/null
timeout 2 ros2 topic pub --once /svtrobot_cmd geometry_msgs/msg/Twist \
    "{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}" 2>/dev/null && echo "  [已发送] 底盘停止指令" || echo "  [跳过] 底盘节点未运行"
timeout 2 ros2 topic pub --once /lift_control_cmd std_msgs/msg/Int32MultiArray \
    "{data: [0, 0]}" 2>/dev/null && echo "  [已发送] 升降停止指令" || echo "  [跳过] 升降节点未运行"

sleep 0.5

# 反序停止服务（先停依赖方，再停被依赖方）
SERVICES=(
    "pcan-monitor"
    "svtrobo-chassis-watchdog"
    "svtrobo-nodeapi"
    "svtrobo-web"
    "svtrobo-f710"
    "svtrobo-model-estop"
    "svtrobo-chassis"
    "svtrobo-rosbridge"
    "svtrobo-can"
    "f710-fix"
)

echo ""
echo "停止服务..."
for svc in "${SERVICES[@]}"; do
    $SUDO systemctl stop $svc 2>/dev/null
    status=$(systemctl is-active $svc 2>/dev/null)
    if [ "$status" = "inactive" ] || [ "$status" = "failed" ]; then
        echo "  [OK] $svc 已停止"
    else
        echo "  [!!] $svc: $status"
    fi
done

echo ""
echo "======================================"
echo "  所有服务已停止"
echo "======================================"
