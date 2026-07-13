#!/usr/bin/python3
"""N10P 一键建图 (带 EKF 融合) — 终端1条命令即可

等效于:
  终端1: ros2 launch n10p_bringup n10p_bringup_launch.py scan_source:=wired
  终端2: ros2 launch n10p_slam slam_only_launch.py
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
    slam_dir = get_package_share_directory('n10p_slam')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'n10p_bringup_launch.py')),
        launch_arguments={'scan_source': 'wired', 'use_ekf': 'true'}.items(),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_dir, 'launch', 'slam_only_launch.py')),
        launch_arguments={'launch_rviz': 'false'}.items(),
    )

    return LaunchDescription([
        bringup_launch,
        TimerAction(period=3.0, actions=[slam_launch]),
    ])
