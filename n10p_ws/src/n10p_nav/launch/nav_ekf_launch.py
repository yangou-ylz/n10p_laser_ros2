#!/usr/bin/python3
"""N10P 一键导航 (带 EKF 融合) — 终端1条命令即可

用法:
  ros2 launch n10p_nav nav_ekf_launch.py map:=/home/ylz/n10p_leishen/maps/n10p_map.yaml

等效于:
  终端1: ros2 launch n10p_bringup n10p_bringup_launch.py scan_source:=wired
  终端2: ros2 launch n10p_nav nav_only_launch.py map:=<path>
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


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
        launch_arguments={'map': map_yaml}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=map_yaml,
                              description='地图 yaml 路径'),
        bringup_launch,
        TimerAction(period=5.0, actions=[nav_launch]),
    ])
