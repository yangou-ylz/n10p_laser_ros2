# 07 — Bug修复记录 + 已知坑点清单

> **最重要**的文档。每个bug都经历过，不要重蹈覆辙。

---

## 1. 驱动层 Bug

### Bug 1: angle_increment 错误 → 所有扫描被SLAM丢弃

- **文件**: `lslidar_driver.cc:990`
- **现象**: slam-toolbox报 "1058 range readings, expected 529"，所有扫描丢弃，/map无数据
- **根因**: `angle_increment = 2*PI/count_num` — 分母应为 `scan_num`（N10P每帧拼合前后半圈, scan_num=2*count_num）
- **修复**: `angle_increment = 2*PI/scan_num`
- **影响**: 所有使用原始驱动的场景

### Bug 2: double free + delete vs delete[] 崩溃

- **文件**: `lslidar_driver.cc:718, 862, 794, 951, 1379`
- **现象**: 驱动启动时 `double free or corruption (out)`, exit code -6
- **根因**: (1) `new unsigned char[500]`数组用`delete`而非`delete[]`释放;
  (2) data_processing子函数delete后调用者polling()再次delete同一指针
- **修复**: 5处`delete`→`delete[]`；删除子函数内的delete（内存归polling管理）
- **影响**: Humble和Jazzy都受影响

## 2. SLAM 层 Bug

### Bug 3: 手持建图旋转时地图严重变形

- **现象**: 直走建图正常，旋转时房间跟着转，同一房间被画了多层重叠
- **根因**: `correlation_search_space_dimension: 0.5` 旋转搜索仅±28°，手持旋转超过这个角度→匹配器找不到对应帧→误判为"房间旋转"
- **修复**: 
  ```yaml
  correlation_search_space_dimension: 1.5  # ±86°旋转搜索
  link_scan_maximum_distance: 3.0          # 帧间平移匹配扩大
  loop_search_maximum_distance: 8.0        # 回环检测扩大
  ```
- **辅助**: 慢速转、贴墙走、走回起点

### Bug 4: slam_launch.py 与 bringup 串口冲突

- **现象**: 两个launch同时运行时驱动double free崩溃
- **根因**: 两者都启动lslidar_driver_node→两个进程抢同一串口
- **修复**: 创建 `slam_only_launch.py`（无driver/odom），配合bringup使用。手持独立模式用 `slam_launch.py`

## 3. Nav2 层 Bug

### Bug 5: planner_server SIGSEGV (exit code -11)

- **现象**: 仿真中收到路径计算请求后planner立即崩溃
- **根因**: 全局costmap `rolling_window: true + obstacle_layer` 导致costmap空指针→getCost()内存访问违规
- **修复**: 全局costmap改用 `static_layer + inflation_layer` + 空白PGM地图

### Bug 6: lifecycle_manager service调用超时

- **现象**: controller_server配置超时 "Failed to change state for node: controller_server"
- **根因**: controller_server初始化时间 > bond_timeout(5s)
- **修复**: 仿真bond_timeout→15s, service_timeout→15s, lifecyle管理器延迟→18s

### Bug 7: map帧死锁

- **现象**: Nav2启动后终端刷"frame 'map' does not exist"，RViz一片空白
- **根因**: AMCL激活前map帧不存在→RViz不渲染地图→用户无法设初始位姿
- **修复**: launch文件中加 `static_transform_publisher map→odom 全零` 做bootstrap

### Bug 8: bt_navigator / RemovePassedGoals 不存在

- **现象**: bt_navigator报 "RemovePassedGoals plugin not found"
- **修复**: 用系统自带 `navigate_w_replanning_time.xml` 替代自定义行为树

## 4. 仿真层 Bug

### Bug 9: scan全部被丢弃 (Message Filter dropping)

- **现象**: costmap和RViz连续报 "dropping message: timestamp earlier than all data in transform cache"
- **根因**: (1) scan_relay没设use_sim_time→时钟不同步;
  (2) local_costmap没设transform_tolerance→微小偏差即丢弃
- **修复**: scan_relay加use_sim_time; local_costmap加transform_tolerance: 0.5

### Bug 10: Fast-DDS共享内存僵尸文件

- **现象**: 所有节点报 "RTPS_TRANSPORT_SHM Error: Failed init_port"
- **根因**: 之前进程残留的 `/dev/shm/fastrtps_*` 文件锁死DDS端口
- **修复**: 每次启动前 `rm -f /dev/shm/fastrtps_*`
- **影响**: 主要影响Gazebo仿真和Nav2服务调用

### Bug 11: ament_python entry_points安装路径

- **现象**: scan_relay启动失败 "executable not found"
- **根因**: console_scripts安装在bin/但launch系统在lib/<pkg>/查找
- **修复**: 每次colcon build后手动cp。URDF改为直接重映射后scan_relay已废弃

## 5. 集成层 Bug

### Bug 12: 两个里程计源TF冲突

- **现象**: costmap报 "Sensor origin at (22,0) is out of map bounds (66,-2) to (68,2)" →机器人定位飞44米
- **根因**: launch有dummy_odom+用户手动开keyboard_odom→两个odom→base_link TF冲突
- **修复**: 二选一，不同时运行

### Bug 13: wifi_bridge发布太早→costmap队列爆满

- **现象**: "Message Filter dropping message: queue is full"
- **根因**: wifi_bridge连接后立即发scan(330+fps→10Hz)，但AMCL未初始化/TF不完整→scan堆积
- **修复**: wifi_bridge加5秒启动延迟，旧数据丢弃

## 6. 已知坑点 (KI)

| KI# | 现象 | 快速解决 |
|-----|------|----------|
| KI-001 | 串口 Permission denied | `sudo usermod -a -G dialout $USER` |
| KI-002 | RViz无点云 | LaserScan→Reliability改Best Effort |
| KI-003 | TF extrapolation报错 | Global Options→Frame Rate改10Hz |
| KI-004 | 有数据无显示 | 发布static TF base_link→laser_frame |
| KI-005 | 驱动double free | 源码delete→delete[]（已修复） |
| KI-006 | 找不到/dev/ttyACM0 | lsusb查芯片型号，改lsx10.yaml的serial_port |
