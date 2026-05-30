#!/usr/bin/python3
"""N10P SLAM 建图启动文件 — 配合 bringup 模式（不启动传感器）
前提: bringup 已在另一终端运行 (ano_bridge + driver + static TF)
本文件只启动: slam-toolbox + RViz2
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import os


def generate_launch_description():

    slam_dir = get_package_share_directory('n10p_slam')

    # ── 1. SLAM: slam-toolbox online async ──────────
    slam_config = os.path.join(slam_dir, 'config', 'mapper_params_online_async.yaml')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config],
    )

    # ── 2. RViz2 ─────────────────────────────────────
    rviz_config = os.path.join(slam_dir, 'config', 'n10p_slam.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        TimerAction(period=1.0, actions=[slam_node]),
        TimerAction(period=4.0, actions=[rviz_node]),
    ])
