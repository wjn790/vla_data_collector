#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='linker_hand_ros2_sdk',
            executable='linker_hand_sdk',
            name='linker_hand_sdk_left',
            output='screen',
            parameters=[{
                'hand_type': 'left',
                'hand_joint': 'O6',
                'is_touch': False,
                'can': 'can1',
                'modbus': 'None',
            }],
        ),

        Node(
            package='linker_hand_ros2_sdk',
            executable='linker_hand_sdk',
            name='linker_hand_sdk_right',
            output='screen',
            parameters=[{
                'hand_type': 'right',
                'hand_joint': 'O6',
                'is_touch': False,
                'can': 'can0',
                'modbus': 'None',
            }],
        ),
    ])
