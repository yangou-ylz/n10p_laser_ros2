# 09 — 配置文件参考

## N10P 驱动配置

**文件**: `n10p_ws/src/Lslidar_ROS2_driver/lslidar_driver/params/lsx10.yaml`

```yaml
/lslidar_driver_node:
  ros__parameters:
    frame_id: laser_frame           # 激光坐标系
    lidar_name: N10_P               # 雷达型号
    min_range: 0.02                 # 最小量程(m)
    max_range: 12.0                 # 最大量程(m)
    interface_selection: serial     # 接口: serial|net
    serial_port_: /dev/serial/by-id/usb-1a86_USB_Single_Serial_58EB011256-if00
    scan_topic: /scan               # 话题名
    pubScan: true                   # 发布/scan
    pubPointCloud2: false           # 不发布点云
    angle_disable_min: 0.0
    angle_disable_max: 0.0
```

## SLAM 配置

**文件**: `n10p_ws/src/n10p_slam/config/mapper_params_online_async.yaml`

树莓派关键参数:
```yaml
map_resolution: 0.1              # 分辨率(m) — 树莓派用0.1
map_update_interval: 3.0
max_laser_range: 12.0
minimum_laser_range: 0.02
minimum_travel_distance: 0.0     # 手持:每帧都处理
minimum_travel_heading: 0.0      # 手持:不依赖里程计
correlation_search_space_dimension: 1.5  # 手持旋转建图搜索窗口
link_scan_maximum_distance: 3.0
loop_search_maximum_distance: 8.0
ceres_num_threads: 2             # 树莓派用2
```

## Nav2 导航配置

**文件**: `n10p_ws/src/n10p_nav/config/nav2_params_n10p.yaml`

树莓派关键参数:
```yaml
amcl:
  robot_model_type: "nav2_amcl::OmniMotionModel"  # 全向模型
  max_particles: 1000             # 树莓派减半
  min_particles: 250
  laser_max_range: 12.0
  laser_min_range: 0.02

planner_server:
  GridBased:
    plugin: "nav2_smac_planner/SmacPlanner2D"
    motion_model_for_search: "MOORE"  # 8方向
    tolerance: 0.25
    minimum_turning_radius: 0.0       # 全向=无转弯半径

controller_server:
  controller_frequency: 10.0     # 树莓派减半
  FollowPath:
    plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
    desired_linear_vel: 0.3
    lookahead_dist: 0.5

global_costmap:
  rolling_window: false          # ❌绝对不能true（会导致SIGSEGV）
  plugins: ["static_layer", "inflation_layer"]

local_costmap:
  rolling_window: true
  width: 4 / height: 4           # 4m×4m滚窗
  plugins: ["obstacle_layer", "inflation_layer"]
```

## 飞控桥接配置

**文件**: `n10p_ws/src/n10p_bringup/params/ano_bridge.yaml`

```yaml
/ano_bridge_node:
  ros__parameters:
    serial_port: /dev/serial/by-id/usb-ANO_TC_ANO_RadioLink-if00
    baud_rate: 921600
```

## 仿真Nav2配置

**文件**: `n10p_ws/src/n10p_gazebo/config/n10p_sim_nav.yaml`

与 nav2_params_n10p.yaml 的关键区别:
- `use_sim_time: true` 所有节点
- 无AMCL（仿真odom=ground truth）
- 全局规划用NavfnPlanner（更轻量）
- local_costmap有transform_tolerance: 0.5
