from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory("chassis_control")
    params_yaml_path = os.path.join(pkg_dir, "config", "params.yaml")

    chassis_control_node = Node(
        package="chassis_control",
        executable="chassis_control_node",
        name="chassis_control",
        output="screen",
        parameters=[params_yaml_path],
    )
    
    # 升降机构RS485控制节点
    lift_rs485_node = Node(
         package='chassis_control',
         executable='lift_control',
         name='lift_control',
         output='screen',
         parameters=[params_yaml_path],
    )

    return LaunchDescription([
        chassis_control_node,
        lift_rs485_node,
    ])
