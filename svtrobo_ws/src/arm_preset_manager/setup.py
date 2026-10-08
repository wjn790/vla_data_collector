from setuptools import find_packages, setup

package_name = 'arm_preset_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/arm_presets.yaml',
            'config/arm_motion_player.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/arm_preset_manager.launch.py',
            'launch/arm_motion_player.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'preset_manager_node = arm_preset_manager.preset_manager_node:main',
            'motion_player_node = arm_preset_manager.motion_player_node:main',
        ],
    },
)
