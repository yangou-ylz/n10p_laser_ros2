---
name: learning-plan
description: N10P SLAM项目的完整学习流程计划，分多个阶段从顶层到底层逐步学习
metadata:
  type: project
---

# N10P ROS2 SLAM 项目 — 学习流程计划

> 创建: 2026-07-12 | 目标: 让 ylz 从零开始完全掌控这个项目

## 学习原则

1. **老板视角优先**：先理解每层"干什么"，再深入"怎么干"
2. **层层剥洋葱**：从顶层框架 → 中层模块 → 底层实现，每层完全理解后才进入下一层
3. **先概念后代码**：先理解基本名词和原理，再去看具体代码
4. **每阶段末尾必须确认**：问"你现在完全理解了吗？"，得到明确肯定才进入下一阶段

---

## 第一阶段：项目全景地图 — "老板视角" 🔴 当前

**目标**：对项目的目标、硬件、数据流、包结构、运行模式有完全的全局认知。
能画出"数据从硬件流入，经过哪些包，最终输出什么"的完整图。
能回答"如果我要改 X，应该动哪个包的哪个文件"。

**子主题**：
1. 项目目标与最终形态（无人机+树莓派+N10P雷达+飞控的完整系统）
2. 硬件全景（N10P雷达、凌霄飞控、树莓派4B、ESP32 WiFi桥接）
3. 数据流向总图（从硬件串口到/cmd_vel的完整链路）
4. 六个ROS2包的职责（一句话说清每个包干什么）
5. TF坐标树（map→odom→base_link→laser_frame的完整含义）
6. 五种运行模式（仅雷达/传感器全开/SLAM建图/Nav2导航/仿真）
7. 项目目录结构速查
8. 关键术语词汇表（Node/Topic/Message/TF/Launch/QoS）

---

## 第二阶段：中层架构 — 六包详解

**目标**：理解每个包的内部结构、关键文件、关键参数。能独立修改配置。

**子阶段**：
2.1 **n10p_bringup 包** — 飞控解析 + 里程计 + WiFi桥接
  - ano_bridge_node: 匿名协议V7 → /odom + /imu + TF
  - dummy_odom_node: 为什么位置全零但SLAM能用
  - keyboard_odom_node: 键盘模拟全向运动
  - n10p_wifi_bridge.py: TCP直读流式数据

2.2 **lslidar_driver 包** — N10P驱动从串口字节到/scan
  - 帧格式（108字节，每帧16个点）
  - echo1/echo2 双回波同角度拼接 (2026-07-20 修正: 非双棱镜180°对装)
  - 函数调用链（main→polling→receive→data_processing→pubScanThread）
  - 关键参数lsx10.yaml

2.3 **n10p_slam 包** — SLAM配置与launch
  - slam-toolbox online async原理
  - 扫描匹配与回环检测
  - 关键参数mapper_params_online_async.yaml

2.4 **n10p_nav 包** — Nav2导航配置与launch
  - AMCL定位原理（粒子滤波）
  - SmacPlanner2D + RegulatedPurePursuit
  - costmap配置（全局静态+局部滚窗）
  - launch中的启动延迟与bootstrap TF

2.5 **n10p_gazebo 包** — 仿真环境
  - URDF无人机模型
  - 仿真与真机的关键区别

2.6 **Launch文件体系** — 7个launch文件的组合使用规则

---

## 第三阶段：关键协议与数据格式

**目标**：理解所有底层通信协议，能独立解析原始数据。

**子阶段**：
3.1 N10P帧格式（逐字节解析）
3.2 匿名协议V7（SC/AC双重校验、8种帧ID详解）
3.3 0xF5自定义帧格式（树莓派→STM32飞控通信）
3.4 ROS2标准消息格式（LaserScan/Odometry/OccupancyGrid/Twist）
3.5 TF坐标系变换的数学基础

---

## 第四阶段：代码级深入

**目标**：理解关键源码，能独立修改和调试。

**子阶段**：
4.1 lslidar_driver.cc核心逻辑（约1384行）
4.2 ano_bridge_node.py（约404行）
4.3 keyboard_odom_node.py的运动模型
4.4 n10p_wifi_bridge.py的帧同步与ScanAccumulator
4.5 launch文件的条件启动逻辑（IfCondition/UnlessCondition）

---

## 第五阶段：实战与调试

**目标**：掌握运行、调试、排查问题的完整能力。

**子阶段**：
5.1 环境激活与编译流程
5.2 验证命令速查
5.3 常见故障排查（13个已知Bug+坑点）
5.4 树莓派特殊约束（内存红线、编译限制）
5.5 飞控联调流程（0xF5帧发送验证）

---

## 第六阶段：系统集成与扩展

**目标**：理解整个系统的最终形态，能设计新的功能。

**子阶段**：
6.1 航点导航模式 vs 视觉伺服模式
6.2 树莓派数据融合逻辑（SLAM+K230→0xF5帧→飞控）
6.3 新功能开发方法论（从需求到部署的完整流程）
