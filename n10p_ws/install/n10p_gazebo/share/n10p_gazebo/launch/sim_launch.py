#!/usr/bin/python3
"""N10P Gazebo 仿真启动文件
启动: Gazebo + 无人机模型 + robot_state_publisher + Nav2 导航栈 + RViz2
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir = get_package_share_directory('n10p_gazebo')

    # ── 启动参数 ─────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    headless = LaunchConfiguration('headless', default='False')
    world_file = LaunchConfiguration('world', default=os.path.join(
        pkg_dir, 'worlds', 'simple_obstacles.world'))
    urdf_file = os.path.join(pkg_dir, 'urdf', 'n10p_drone.urdf')
    params_file = os.path.join(pkg_dir, 'config', 'n10p_sim_nav.yaml')
    rviz_config = os.path.join(pkg_dir, 'config', 'n10p_sim.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ══════════════════════════════════════════════════
    # 1. Gazebo (服务器 + 客户端)
    # ══════════════════════════════════════════════════
    gzserver = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen',
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
    )

    # ══════════════════════════════════════════════════
    # 2. 生成无人机模型
    # ══════════════════════════════════════════════════
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_n10p_drone',
        output='screen',
        arguments=[
            '-entity', 'n10p_drone',
            '-file', urdf_file,
            '-x', '0.0', '-y', '0.0', '-z', '0.1',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0',
        ],
    )

    # ══════════════════════════════════════════════════
    # 3. robot_state_publisher (发布 TF + robot_description)
    # ══════════════════════════════════════════════════
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )

    # ══════════════════════════════════════════════════
    # 4. Map Server — 空白静态地图 (全局 costmap 使用)
    # ══════════════════════════════════════════════════
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': '/home/ubuntu22/ROS2/n10p_leishen/maps/blank_map.yaml',
        }],
    )

    # ══════════════════════════════════════════════════
    # 5. 静态 TF: map → odom (仿真中 odom 即 ground truth)
    # ══════════════════════════════════════════════════
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    # ══════════════════════════════════════════════════
    # 5. Nav2 导航栈
    # ══════════════════════════════════════════════════
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
    )

    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file],
    )

    lifecycle_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'planner_server', 'controller_server', 'bt_navigator'],
            'bond_timeout': 15.0,
            'service_timeout': 15.0,
        }],
    )

    # ══════════════════════════════════════════════════
    # 7. RViz2 (仿真视图)
    # ══════════════════════════════════════════════════
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('headless', default_value='False'),
        DeclareLaunchArgument('world', default_value=world_file),

        # Gazebo 服务器先启动
        gzserver,
        gzclient,

        # 等 Gazebo 就绪后生成模型
        TimerAction(period=3.0, actions=[spawn_robot]),

        # robot_state_publisher + map_server + TF + scan relay (提前启动，留足初始化时间)
        TimerAction(period=5.0, actions=[robot_state_pub]),
        TimerAction(period=3.0, actions=[map_server]),
        TimerAction(period=5.5, actions=[static_tf_map_odom]),

        # Nav2 导航栈 (等 TF 树就绪)
        TimerAction(period=8.0, actions=[planner_node]),
        TimerAction(period=8.0, actions=[controller_node]),
        TimerAction(period=8.0, actions=[bt_navigator_node]),
        TimerAction(period=18.0, actions=[lifecycle_nav]),

        # RViz2 最后启动
        TimerAction(period=19.0, actions=[rviz_node]),
    ])
