# 05 — 架构决策记录

## ADR-001: 选择 slam-toolbox 做2D SLAM

**决策**：使用 `ros-humble-slam-toolbox` 的 online async 模式
**原因**：
- 对单线激光雷达支持良好，社区维护活跃
- 支持在线异步建图+离线优化，资源占用可控
- 手持模式可用全零里程计，靠扫描匹配自估运动
**备选**：Hector SLAM（无需里程计但建图质量差）、Cartographer（精度高但计算量大，不适合树莓派）、Gmapping（ROS1经典，ROS2版本不成熟）

## ADR-002: 分阶段开发策略

**决策**：Phase 0→1→2→...→7，每步可执行可验证
**原因**：避免一次性部署的调试困难。前步不通过后步不启动

## ADR-003: 零侵入双路径（有线+无线雷达）

**决策**：有线(lslidar_driver)和无线(n10p_wifi_bridge_node)完全隔离
- 两个路径都不修改对方的代码
- launch文件通过 `scan_source:=wired|wireless` 参数切换
- 下游(SLAM/Nav2/RViz2)只订阅/scan，不感知数据源
- 尝试过socat PTY方案让驱动无改动接入→失败（tcsetattr破坏PTY行规约）

## ADR-004: 放弃socat PTY → 独立Python节点

**原因**：lslidar_driver的tcsetattr()改变PTY终端属性导致poll()永久阻塞
**决策**：用独立Python节点，TCP socket直读流式数据，应用层帧同步

## ADR-005: 桌面测试用键盘里程计

**决策**：创建keyboard_odom_node（WASD全向控制），独立终端运行
**设计选择**：
- `odom→base_link` TF由键盘节点发布，20Hz
- launch文件**不启动**键盘节点（用户手动 `ros2 run`）
- 不得同时运行dummy_odom和keyboard_odom（TF冲突）

## ADR-006: 导航栈配置

- AMCL: OmniMotionModel（全向无人机）+ likelihood_field + 2000粒子
- 全局规划: SmacPlanner2D（Hybrid-A*）+ MOORE 8方向搜索
- 局部控制: RegulatedPurePursuit（纯追踪调节器）
- 行为树: navigate_w_replanning_time.xml（最简单，1Hz重规划+FollowPath）
- 全局costmap: static_layer + inflation_layer（固定在地图坐标，不滚窗）
- 局部costmap: obstacle_layer + inflation_layer（4m×4m滚窗）

## ADR-007: 全局costmap禁用滚窗障碍物层

**原因**：`rolling_window: true + obstacle_layer` 组合导致Nav2 planner SIGSEGV崩溃
**替代**：`static_layer + 空白PGM地图`。仿真用blank_map.pgm(200×200全空闲)，真实硬件用n10p_map.pgm

## ADR-008: bootstrap的map→odom静态TF

**原因**：AMCL激活前map帧不存在→RViz不能渲染地图→用户无法设初始位姿→死锁
**解决**：launch加 `static_transform_publisher map odom 0 0 0 0 0 0` 做启动引导，
AMCL激活后自动覆盖
