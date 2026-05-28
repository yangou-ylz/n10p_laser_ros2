#!/usr/bin/python3
"""N10P SLAM 建图启动文件
启动: 凌霄飞控桥接 + N10P 雷达 + slam-toolbox + RViz2
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os


def generate_launch_description():

    # ── 1. 传感器层: 凌霄飞控 + N10P 雷达 + TF ──────
    bringup_dir = get_package_share_directory('n10p_bringup')
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'n10p_bringup_launch.py')
        )
    )

    # ── 2. SLAM: slam-toolbox online async ──────────
    slam_config = os.path.join(
        get_package_share_directory('n10p_slam'),
        'config', 'mapper_params_online_async.yaml')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config],
    )

    # ── 3. RViz2 (带延迟, 等 SLAM 初始化) ──────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('n10p_slam'),
            'config', 'n10p_slam.rviz')],
        output='screen',
    )

    return LaunchDescription([
        bringup_launch,
        TimerAction(period=3.0, actions=[slam_node]),     # 等传感器就绪
        TimerAction(period=6.0, actions=[rviz_node]),     # 等 SLAM 就绪
    ])
