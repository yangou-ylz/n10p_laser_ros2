# 08 — Launch文件参考

## 所有 Launch 文件一览

| Launch文件 | 所属包 | 用途 | 启动驱动? | 启动里程计? |
|-----------|--------|------|:---:|:---:|
| `lslidar_launch.py` | lslidar_driver | 仅雷达驱动+RViz | ✅驱动 | ❌ |
| `n10p_bringup_launch.py` | n10p_bringup | 飞控+雷达全开 | ✅驱动 | ✅ano_bridge |
| `slam_launch.py` | n10p_slam | 手持建图(独立) | ✅驱动 | ✅dummy_odom |
| `slam_only_launch.py` | n10p_slam | SLAM仅(配合bringup) | ❌ | ❌ |
| `nav_launch.py` | n10p_nav | Nav2全栈导航 | ✅驱动 | ✅dummy_odom |
| `desktop_test_launch.py` | n10p_nav | 桌面测试(键盘) | ✅驱动 | ❌(需手动键盘) |
| `sim_launch.py` | n10p_gazebo | Gazebo仿真导航 | ❌(虚拟) | ✅planar_move |

## 公共参数

| 参数 | 默认值 | 可用值 | 说明 |
|------|--------|--------|------|
| `scan_source` | `wired` | `wired`, `wireless` | 雷达数据源切换 |
| `map` | 各launch默认路径 | 任意.yaml路径 | 地图文件路径 |

## 典型使用场景

```bash
# 1. 仅看雷达点云
ros2 launch lslidar_driver lslidar_launch.py

# 2. 手持建图 (有线)
ros2 launch n10p_slam slam_launch.py

# 3. 手持建图 (无线ESP32)
ros2 launch n10p_slam slam_launch.py scan_source:=wireless

# 4. 飞控在线SLAM (有线)
# 终端1
ros2 launch n10p_bringup n10p_bringup_launch.py
# 终端2
ros2 launch n10p_slam slam_only_launch.py

# 5. Nav2导航 (无线)
ros2 launch n10p_nav nav_launch.py scan_source:=wireless map:=/path/to/map.yaml

# 6. 桌面测试 (无线)
# 终端1
ros2 run n10p_bringup keyboard_odom_node
# 终端2
ros2 launch n10p_nav desktop_test_launch.py scan_source:=wireless

# 7. 仿真
bash ~/ROS2/n10p_leishen/scripts/start_simulation.sh
```

## 冲突注意事项

- `slam_launch.py` + `n10p_bringup_launch.py` = ❌ 串口冲突
- `slam_only_launch.py` + `n10p_bringup_launch.py` = ✅ 正确组合
- `dummy_odom` + `keyboard_odom` = ❌ TF冲突
- 有线 + 无线同时启动 = ❌ /scan多发布者
