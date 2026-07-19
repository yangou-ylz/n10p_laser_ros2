#!/usr/bin/python3
"""N10P Nav2 纯导航启动文件 — 配合 bringup 模式
前提: bringup 已在另一终端运行 (ano_bridge + driver + static TF)
      必须先建好地图保存到 maps/ 目录
本文件只启动: 地图服务 + AMCL + 规划器 + 控制器 + BT

用法:
  ros2 launch n10p_nav nav_only_launch.py map:=/home/ylz/n10p_leishen/maps/n10p_map.yaml
  ros2 launch n10p_nav nav_only_launch.py initial_x:=-9.38 initial_y:=-10.6 initial_yaw:=0.0
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    nav_dir = get_package_share_directory('n10p_nav')
    params_file = os.path.join(nav_dir, 'config', 'nav2_params_n10p.yaml')

    map_yaml_arg = DeclareLaunchArgument('map', default_value='/home/ylz/n10p_leishen/maps/n10p_map.yaml',
                                         description='地图 yaml 路径')
    launch_rviz_arg = DeclareLaunchArgument('launch_rviz', default_value='false')
    # 初始位姿 — 支持命令行覆盖, 无需重编 YAML
    initial_x_arg = DeclareLaunchArgument('initial_x', default_value='0.0')
    initial_y_arg = DeclareLaunchArgument('initial_y', default_value='0.0')
    initial_yaw_arg = DeclareLaunchArgument('initial_yaw', default_value='0.0')

    map_yaml = LaunchConfiguration('map')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')

    # ── 静态 TF: map → odom (bootstrap, AMCL 激活后自动覆盖) ──
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    # ── 地图服务 ──────────────────────────────────────────
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': map_yaml}],
    )

    # ── AMCL 定位 — 初始位姿支持命令行覆盖 ──────────────
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {
            'initial_pose.x': initial_x,
            'initial_pose.y': initial_y,
            'initial_pose.yaw': initial_yaw,
        }],
    )

    lifecycle_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
            'bond_timeout': 10.0,      # 树莓派增加超时容忍
            'service_timeout': 10.0,
        }],
    )

    # ── 路径规划 ──────────────────────────────────────────
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
    )

    # ── 控制器 ────────────────────────────────────────────
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
    )

    # ── 行为树 ────────────────────────────────────────────
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file],
    )

    lifecycle_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['planner_server', 'controller_server', 'bt_navigator'],
            'bond_timeout': 15.0,      # 树莓派启动较慢
            'service_timeout': 15.0,
        }],
    )

    # ── RViz2 (树莓派默认不启动) ──────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(nav_dir, 'config', 'n10p_nav.rviz')],
        output='screen',
    )

    def launch_setup(context):
        nodes = [
            static_tf_map_odom,
            TimerAction(period=1.0, actions=[map_server_node]),
            TimerAction(period=2.0, actions=[amcl_node]),
            TimerAction(period=3.0, actions=[lifecycle_localization]),
            TimerAction(period=5.0, actions=[planner_node]),
            TimerAction(period=5.0, actions=[controller_node]),
            TimerAction(period=5.0, actions=[bt_navigator_node]),
            TimerAction(period=7.0, actions=[lifecycle_navigation]),
        ]
        if LaunchConfiguration('launch_rviz').perform(context).lower() == 'true':
            nodes.append(TimerAction(period=9.0, actions=[rviz_node]))
        return nodes

    return LaunchDescription([
        map_yaml_arg,
        launch_rviz_arg,
        initial_x_arg, initial_y_arg, initial_yaw_arg,
        OpaqueFunction(function=launch_setup),
    ])
