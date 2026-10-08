from setuptools import find_packages, setup

package_name = 'f710_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        ('share/' + package_name + '/launch', [
            'launch/f710_teleop.launch.py',
        ]),
        # Config files
        ('share/' + package_name + '/config', [
            'config/f710_teleop.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Logitech F710 手柄直接控制底盘和升降机构的 ROS2 功能包。',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'my_controller_node = f710_teleop.my_controller_node:main',
        ],
    },
)

