#!/bin/bash
# N10P Gazebo 仿真启动脚本
# 自动清理 DDS 共享内存残留，避免通信故障

echo ">>> 清理 DDS 残留进程和共享内存..."
pkill gzserver 2>/dev/null
pkill gzclient 2>/dev/null
sleep 1
rm -f /dev/shm/fastrtps_* 2>/dev/null
ros2 daemon stop 2>/dev/null
ros2 daemon start 2>/dev/null
echo ">>> 环境激活..."
ros2env
source ~/ROS2/n10p_leishen/n10p_ws/install/setup.bash
echo ">>> 启动仿真..."
ros2 launch n10p_gazebo sim_launch.py "$@"
