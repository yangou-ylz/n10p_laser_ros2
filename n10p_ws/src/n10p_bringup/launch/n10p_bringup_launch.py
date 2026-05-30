#!/usr/bin/python3
# ============================================================================
# n10p_bringup_launch.py — 传感器全开启动文件
# ============================================================================
# 同时启动三个节点：飞控桥接 + 雷达驱动 + 静态 TF
# 适合：需要飞控里程计 + 雷达数据的场景（如 SLAM 建图配合飞控、Nav2 导航）
#
# 重要：这个文件启动了雷达驱动，所以不能和 slam_launch.py（也自带驱动）同时运行！
#       串口冲突 → 驱动崩溃(double free)
# ============================================================================
"""启动匿名凌霄飞控桥接节点 + N10P 驱动"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node        # 普通节点（非生命周期管理）
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():

    # ── 节点 1：匿名凌霄飞控桥接 ─────────────────────────
    # 作用：读取飞控串口 → 解析匿名协议 V7 → 发布 /odom + /imu + TF(odom→base_link)
    bridge_params = os.path.join(
        get_package_share_directory('n10p_bringup'),
        'params', 'ano_bridge.yaml')       # 指定飞控的串口号、波特率、scale 因子等

    bridge_node = Node(
        package='n10p_bringup',
        executable='ano_bridge_node',      # 对应 setup.py 中注册的 entry_point
        name='ano_bridge_node',
        output='screen',
        parameters=[bridge_params],        # 加载 YAML 参数文件
    )

    # ── 节点 2：N10P 激光雷达驱动 ───────────────────────
    # 作用：读取雷达串口 → 解析数据帧 → 发布 /scan（LaserScan）
    driver_params = os.path.join(
        get_package_share_directory('lslidar_driver'),
        'params', 'lsx10.yaml')            # 指定雷达的串口号、型号(N10_P)、量程等

    driver_node = Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',  # C++ 编译产出的可执行文件
        name='lslidar_driver_node',        # 节点运行时名字（发布 /scan 时发件人就是它）
        output='screen',
        parameters=[driver_params],
    )

    # ── 节点 3：静态 TF → base_link 到 laser_frame ─────
    # 作用：告诉 TF 系统"雷达装在机器人正下方 10cm 处"，这个关系永远不变
    # 参数格式：(x y z roll pitch yaw parent_frame child_frame)
    #   (0, 0, -0.1) = 雷达在 base_link 正下方 10cm（真实无人机雷达吊装在下方）
    #   base_link = 父坐标系（机器人本体）
    #   laser_frame = 子坐标系（雷达）
    # N10P 安装在无人机下方，坐标系: X前 Y左 Z上
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=['0', '0', '-0.1', '0', '0', '0', 'base_link', 'laser_frame'],
    )

    # ── 返回：三个节点一起启动 ───────────────────────────
    # ros2 launch 会同时启动这三个节点，它们各自独立运行
    return LaunchDescription([
        bridge_node,       # → 飞控数据 → /odom + /imu + TF
        driver_node,       # → 雷达数据 → /scan
        static_tf_node,    # → 静态 TF → base_link→laser_frame
    ])
