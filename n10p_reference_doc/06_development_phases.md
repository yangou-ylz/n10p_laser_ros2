# 06 — 开发阶段详情

## Phase 0: 环境验证 ✅ (2026-05-27)

- ros2env命令可用, ROS2 Humble环境正常
- dialout用户组, colcon+cmake工具链
- libpcap-dev缺失→用户手动安装
- 所有编译依赖确认齐全

## Phase 1: N10P 驱动编译与数据验证 ✅ (2026-05-27)

- 创建工作空间n10p_ws/，克隆Lslidar_ROS2_driver（M10P/N10P分支）
- 新增依赖: libpcl-dev, ros-humble-pcl-conversions
- 修改lsx10.yaml: 串口/dev/ttyACM0, 型号N10_P, 量程0.02~12m
- 编译成功: 2 packages (16.8s)
- 验证: /scan 10.05Hz, frame_id=laser_frame, 360°完整数据

## Phase 2: RViz2 可视化联调 ✅ (2026-05-27)

- 修复KI-004: Fixed Frame laser_link→laser_frame
- 修复KI-002/003: Reliability→Best Effort, Frame Rate→10Hz
- 端到端联调验证通过

## Phase 2.5: 凌霄飞控串口解析 ✅ (2026-05-27)

- 分析匿名协议V7, 8个帧ID映射到/odom和/imu
- 创建n10p_bringup包(ament_python), ano_bridge_node
- 编译测试通过: /dev/ttyACM0, 921600bps
- 已知: ACC scale需校准(z≈6.4而非9.8), 飞控静止位置保持0,0

## Phase 3: SLAM 建图 ✅ (2026-05-28)

- 创建n10p_slam包, 配置slam-toolbox online async
- 首次手持建图成功
- **驱动Bug修复**: angle_increment=2*PI/scan_num
- 创建dummy_odom_node（占位里程计）
- 保存首张地图: n10p_map.pgm(30KB)+n10p_map.yaml

## Phase 4: Nav2 导航 ✅ (2026-05-28)

- 创建n10p_nav包: AMCL+SmacPlanner2D+RegulatedPurePursuit+bt_navigator
- 全部lifecycle节点激活, 路径规划触发成功
- Bug修复: bt_navigator用navigate_w_replanning_time.xml

## Phase 5: Gazebo 仿真集成 ✅ (2026-05-28)

- 创建n10p_gazebo包: URDF无人机+4箱世界+Nav2栈
- **Bug修复**: planner SIGSEGV→static_layer+空白地图
- 仿真导航验证: 机器人移动到目标, planner无崩溃
- 仿真TF树: map(static)→odom(planar_move)→base_footprint→base_link→laser_frame

## Phase 5.5: 桌面测试模式 ✅ (2026-05-28)

- 创建keyboard_odom_node(WASD全向控制, 纯stdlib)
- 创建desktop_test_launch.py
- 用法: 终端1键盘里程计 + 终端2桌面测试

## Phase 5.6: ESP32 WiFi无线雷达 ✅ (2026-06-01)

- 3个ESP32 Phases: UART接收→WiFi TCP→端到端集成
- n10p_wifi_bridge.py: 独立Python ROS2节点
- 4个launch文件加scan_source参数(wired/wireless)
- SLAM旋转建图bug修复(correlation_search_space_dimension)
- desktop_test + map帧死锁修复(bootstrap静态TF)

## Phase 6: 树莓派4B移植 ← 当前任务

- arm64 Ubuntu 22.04.05 LTS Server
- TF卡64G+SSD 512G
- 复制项目代码+编译+验证全流程
- SLAM/Nav2参数适配树莓派性能

## Phase 7: 无人机MAVROS集成 (待开始)

- 飞控MAVROS通信
- 里程计从飞控直接获取
- 指令下发: /cmd_vel→飞控执行
