#!/bin/bash
# N10P ROS2 环境清理脚本
# 用法: bash scripts/clean_ros2.sh

echo "清理 ROS2 环境..."

# 杀占用串口的进程 (最可靠的方式)
for pid in $(lsof -t /dev/ttyUSB0 2>/dev/null); do kill -9 $pid 2>/dev/null; done
for pid in $(lsof -t /dev/ttyACM0 2>/dev/null); do kill -9 $pid 2>/dev/null; done

# 杀所有项目节点 (进程名匹配)
pkill -9 -f "lslidar_driver_node|ano_bridge_node|static_tf_laser|imu_filter_node|async_slam_toolbox_node|slam_toolbox|keyboard_odom|dummy_odom|n10p_wifi_bridge|rviz2" 2>/dev/null

sleep 2

# 清理 DDS
rm -f /dev/shm/fastrtps_* 2>/dev/null
ros2 daemon stop 2>/dev/null
ros2 daemon start 2>/dev/null

echo "✅ 清理完成"
lsof /dev/ttyUSB0 /dev/ttyACM0 2>/dev/null || echo "串口已释放"
