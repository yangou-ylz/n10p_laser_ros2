#!/usr/bin/python3
"""N10P 一键导航 (带 EKF 融合)

用法:
  ros2 launch n10p_nav nav_ekf_launch.py map:=/path/to/map.yaml

注意: 不再依赖 FC yaw。AMCL 初始 yaw=0，用户在 RViz 用 "2D Pose Estimate"
  手动标一次初始位姿使点云对齐地图即可。
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = get_package_share_directory('n10p_bringup')
    nav_dir = get_package_share_directory('n10p_nav')

    map_yaml = LaunchConfiguration('map', default=os.path.join(
        nav_dir, '..', '..', '..', '..', '..', 'maps', 'n10p_map.yaml'))

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'n10p_bringup_launch.py')),
        launch_arguments={'scan_source': 'wired', 'use_ekf': 'true'}.items(),
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_dir, 'launch', 'nav_only_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'initial_yaw': '0.0',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=map_yaml,
                              description='地图 yaml 路径'),
        bringup_launch,
        TimerAction(period=8.0, actions=[nav_launch]),
    ])
