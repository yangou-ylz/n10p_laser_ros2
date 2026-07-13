#!/bin/bash
# N10P EKF 滤波验证脚本 — 自动编译+启动+检验
# 用法: bash scripts/verify_ekf.sh [--dynamic]

set -e
RED='\033[0;31m'; GREEN='\033[0;32m'; NC='\033[0m'

echo "===== EKF 滤波验证 ====="

# 1. 编译
echo "[1/4] 编译 n10p_fusion..."
source /opt/ros/humble/setup.bash
cd /home/ylz/n10p_leishen/n10p_ws
colcon build --packages-select n10p_fusion --symlink-install --parallel-workers 2 > /dev/null 2>&1
source install/setup.bash
# 修复 ament_python entry_points 路径
mkdir -p install/n10p_fusion/lib/n10p_fusion
cp install/n10p_fusion/bin/imu_filter_node install/n10p_fusion/lib/n10p_fusion/imu_filter_node 2>/dev/null || true
echo -e "${GREEN}[1/4] 编译通过${NC}"

# 2. 静态测试: 启动 → 检查滤波输出
echo "[2/4] 静态测试 (10s)..."
ros2 launch n10p_bringup n10p_bringup_launch.py scan_source:=wired use_ekf:=true &
PID=$!
sleep 8

# 检查节点
if ros2 node list 2>/dev/null | grep -q imu_filter_node; then
    echo -e "${GREEN}[2/4] imu_filter_node 运行中${NC}"
else
    echo -e "${RED}[2/4] FAIL: imu_filter_node 未启动${NC}"
    kill $PID 2>/dev/null; exit 1
fi

# 检查滤波输出
if ros2 topic echo /odometry/filtered --qos-reliability best_effort --once 2>/dev/null | grep -q "frame_id: odom"; then
    echo -e "${GREEN}[2/4] /odometry/filtered 有数据${NC}"
else
    echo -e "${RED}[2/4] FAIL: /odometry/filtered 无数据${NC}"
    kill $PID 2>/dev/null; exit 1
fi

# 检查频率
FREQ=$(timeout 3 ros2 topic hz /odometry/filtered --qos-reliability best_effort 2>/dev/null | grep "average rate" | awk '{print $3}')
if [ -n "$FREQ" ]; then
    echo -e "${GREEN}[2/4] /odometry/filtered 频率: ${FREQ}Hz${NC}"
else
    echo -e "${RED}[2/4] WARN: 无法获取频率${NC}"
fi

# 3. 确认 use_ekf=false 不受影响
echo "[3/4] 回归测试: use_ekf=false..."
kill $PID 2>/dev/null; sleep 2
ros2 launch n10p_bringup n10p_bringup_launch.py scan_source:=wired use_ekf:=false &
PID=$!
sleep 6

if ros2 node list 2>/dev/null | grep -q ano_bridge_node && ! ros2 node list 2>/dev/null | grep -q imu_filter_node; then
    echo -e "${GREEN}[3/4] 回归通过: use_ekf=false 无 EKF 节点${NC}"
else
    echo -e "${RED}[3/4] FAIL: 回归异常${NC}"
    kill $PID 2>/dev/null; exit 1
fi

if ros2 topic hz /odom 2>/dev/null | grep -q "average rate"; then
    echo -e "${GREEN}[3/4] /odom 正常发布${NC}"
fi

# 4. 动态测试 (可选)
if [ "$1" = "--dynamic" ]; then
    echo "[4/4] 动态测试: 请手持飞机缓慢运动 30 秒..."
    kill $PID 2>/dev/null; sleep 2
    ros2 launch n10p_bringup n10p_bringup_launch.py scan_source:=wired use_ekf:=true &
    PID=$!
    sleep 5
    echo "录制 rosbag 30s..."
    ros2 bag record /odometry/filtered /odom /imu -o /tmp/ekf_dynamic_test 2>/dev/null &
    BAG_PID=$!
    sleep 30
    kill $BAG_PID 2>/dev/null
    echo -e "${GREEN}[4/4] rosbag 已保存到 /tmp/ekf_dynamic_test${NC}"
fi

kill $PID 2>/dev/null
echo -e "${GREEN}===== 全部验证通过 =====${NC}"
