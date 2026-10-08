#!/bin/bash
# 机械臂独立启动脚本
source /opt/ros/humble/setup.bash
source /home/svt/svtrobo_ws/install/setup.bash

exec ros2 launch arm_preset_manager arm_preset_manager.launch.py "$@"
