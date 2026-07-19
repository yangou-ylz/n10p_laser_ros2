#!/usr/bin/python3
"""N10P SLAM 建图启动文件 — 配合 bringup 模式（不启动传感器）
前提: bringup 已在另一终端运行 (ano_bridge + driver + static TF)
本文件只启动: slam-toolbox (+ 可选 RViz2)

参数:
  launch_rviz:=false    是否启动 RViz2 (树莓派默认不启动)
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    slam_dir = get_package_share_directory('n10p_slam')
    launch_rviz_arg = DeclareLaunchArgument('launch_rviz', default_value='false')

    slam_config = os.path.join(slam_dir, 'config', 'mapper_params_online_async.yaml')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(slam_dir, 'config', 'n10p_slam.rviz')],
        output='screen',
    )

    def launch_setup(context):
        nodes = [TimerAction(period=1.0, actions=[slam_node])]
        if LaunchConfiguration('launch_rviz').perform(context).lower() == 'true':
            nodes.append(TimerAction(period=4.0, actions=[rviz_node]))
        return nodes

    return LaunchDescription([
        launch_rviz_arg,
        OpaqueFunction(function=launch_setup),
    ])
