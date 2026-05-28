#!/usr/bin/python3
"""N10P SLAM 建图启动文件 — 手持建图模式（不需要飞控）
启动: 占位里程计 + N10P 雷达 + slam-toolbox + RViz2
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import os


def generate_launch_description():

    # ── 1. 占位里程计 (飞控不在线时提供 odom → base_link TF) ──
    dummy_odom_node = Node(
        package='n10p_bringup',
        executable='dummy_odom_node',
        name='dummy_odom_node',
        output='screen',
    )

    # ── 2. N10P 激光雷达驱动 ──────────────────────────
    driver_params = os.path.join(
        get_package_share_directory('lslidar_driver'),
        'params', 'lsx10.yaml')

    driver_node = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        parameters=[driver_params],
    )

    # ── 3. 静态 TF: base_link → laser_frame ──────────
    # N10P 安装在下方，X前 Y左 Z上
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['0', '0', '-0.1', '0', '0', '0', 'base_link', 'laser_frame'],
    )

    # ── 4. SLAM: slam-toolbox online async ──────────
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

    # ── 5. RViz2 (等 SLAM 就绪) ──────────────────────
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
        dummy_odom_node,
        driver_node,
        static_tf_node,
        TimerAction(period=3.0, actions=[slam_node]),
        TimerAction(period=6.0, actions=[rviz_node]),
    ])
