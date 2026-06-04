# 04 — 工作空间结构

## 目录树

```
n10p_leishen/
├── CLAUDE.md                     # Claude Code 最高指令文件
├── user.md                       # 保姆级使用教程
├── env.md                        # 环境配置教程
├── requirements.txt              # 依赖清单
├── n10p_knowledge_base/          # N10P 硬件/协议官方资料
├── esp32_n10p_bridge/            # ESP32 WiFi桥接固件工程
├── maps/                         # SLAM建图保存的PGM地图
├── scripts/                      # 工具脚本
├── n10p_reference_doc/           # 本参考文档目录
└── n10p_ws/                      # ROS2 工作空间
    ├── build/                    # 编译中间产物
    ├── install/                  # 编译安装产物
    └── src/
        ├── Lslidar_ROS2_driver/   # N10P 官方驱动（2个子包）
        │   ├── lslidar_msgs/       #   自定义消息类型
        │   └── lslidar_driver/     #   雷达驱动 + launch + params + rviz
        ├── n10p_bringup/           # 启动配置包
        ├── n10p_slam/              # SLAM配置包
        ├── n10p_nav/               # Nav2导航包
        └── n10p_gazebo/            # Gazebo仿真包
```

## n10p_bringup 包 — 核心集成节点

| 文件 | 功能 |
|------|------|
| `ano_bridge_node.py` | 飞控串口→ROS2 (/odom+/imu+TF), 匿名协议V7 |
| `dummy_odom_node.py` | 占位里程计（全零位置+飞控四元数姿态） |
| `keyboard_odom_node.py` | 键盘全向里程计（WASD操控, 桌面测试用） |
| `n10p_wifi_bridge.py` | ESP32 WiFi TCP→/scan (无线雷达接入) |
| `launch/n10p_bringup_launch.py` | 飞控+雷达全开启动 |
| `params/ano_bridge.yaml` | 飞控串口+协议参数 |

## n10p_slam 包

| 文件 | 功能 |
|------|------|
| `launch/slam_launch.py` | 手持建图（dummy_odom+driver+SLAM+RViz） |
| `launch/slam_only_launch.py` | SLAM仅（配合bringup,无driver/odom） |
| `config/mapper_params_online_async.yaml` | SLAM参数 |
| `config/n10p_slam.rviz` | SLAM RViz配置 |

## n10p_nav 包

| 文件 | 功能 |
|------|------|
| `launch/nav_launch.py` | Nav2导航全启动 |
| `launch/desktop_test_launch.py` | 桌面测试模式（键盘里程计+Nav2） |
| `config/nav2_params_n10p.yaml` | Nav2参数（AMCL+Planner+Controller+costmaps） |
| `config/n10p_nav.rviz` | Nav2 RViz配置 |

## n10p_gazebo 包

| 文件 | 功能 |
|------|------|
| `urdf/n10p_drone.urdf` | 无人机模型（圆柱体+2D LiDAR+全向移动插件） |
| `worlds/simple_obstacles.world` | 4个箱子的简单世界 |
| `launch/sim_launch.py` | 仿真启动文件 |
| `config/n10p_sim_nav.yaml` | 仿真Nav2参数 |
| `config/n10p_sim.rviz` | 仿真RViz配置 |
| `n10p_gazebo/scan_relay.py` | LiDAR话题转发（已废弃：URDF已改为直接/scan） |

## 驱动已修复的源码bug

`Lslidar_ROS2_driver/lslidar_driver/src/lslidar_driver.cc`:
- L990: `angle_increment = 2*PI/scan_num`（原bug: /count_num, 导致角度增量为正确值两倍）
- L718, L862: 删除子函数内的 `delete packet_bytes`（double free, 内存归调用者polling管理）
- L794, L951, L1379: `delete` → `delete[]`（数组分配必须用delete[]）
