#!/bin/bash
# N10P ROS2 环境清理脚本 — 杀掉所有项目相关进程，清理 DDS 共享内存
# 用法: bash scripts/clean_ros2.sh

echo "清理 ROS2 环境..."

# 杀所有项目节点
pkill -9 -f "ano_bridge_node|lslidar_driver_node|static_tf_laser|imu_filter_node|async_slam_toolbox_node|slam_toolbox|keyboard_odom|dummy_odom|n10p_wifi_bridge|rviz2" 2>/dev/null

sleep 2

# 清理 DDS 共享内存
rm -f /dev/shm/fastrtps_* 2>/dev/null

# 重启 daemon
ros2 daemon stop 2>/dev/null
ros2 daemon start 2>/dev/null

echo "✅ 清理完成 — 所有残留进程已终止"
ros2 node list 2>/dev/null
