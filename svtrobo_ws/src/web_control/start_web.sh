#!/bin/bash
# SVTROBO Web Control Launcher
# Starts rosbridge_server + aiohttp camera/static server
# rosbridge uses system Python3, web server uses conda svtrobo env

set +e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Source ROS2 environment
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/svt/svtrobo_ws/install/setup.bash 2>/dev/null

echo "======================================"
echo "  SVTROBO Web Control"
echo "======================================"

# Start rosbridge_server (needs system Python with tornado)
echo "[1/2] Starting rosbridge_server on port 9090..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 &
ROSBRIDGE_PID=$!
sleep 2

if ! kill -0 $ROSBRIDGE_PID 2>/dev/null; then
    echo "ERROR: rosbridge_server failed to start"
    exit 1
fi
echo "      rosbridge_server started (PID: $ROSBRIDGE_PID)"

# Start aiohttp web server in conda svtrobo env (for pyrealsense2 + cv2)
echo "[2/2] Starting web server on port 8080 (conda svtrobo)..."
cd "$SCRIPT_DIR"
source /home/svt/miniconda3/etc/profile.d/conda.sh
conda activate svtrobo
python3 server.py --host 0.0.0.0 --port 8080 &
SERVER_PID=$!
sleep 1

if ! kill -0 $SERVER_PID 2>/dev/null; then
    echo "ERROR: Web server failed to start"
    kill $ROSBRIDGE_PID 2>/dev/null
    exit 1
fi

echo ""
echo "======================================"
echo "  Ready! Open in browser:"
echo "  http://localhost:8080"
echo "======================================"
echo ""
echo "Press Ctrl+C to stop..."

# Cleanup on exit
cleanup() {
    echo ""
    echo "Shutting down..."
    kill $SERVER_PID 2>/dev/null
    kill $ROSBRIDGE_PID 2>/dev/null
    wait 2>/dev/null
    echo "Done."
}
trap cleanup SIGINT SIGTERM

wait
