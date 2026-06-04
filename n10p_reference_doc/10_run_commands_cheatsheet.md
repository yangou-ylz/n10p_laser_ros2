# 10 — 运行命令速查

## 编译

```bash
cd ~/ROS2/n10p_leishen/n10p_ws  # 树莓派: /mnt/ssd/n10p_leishen/n10p_ws

# 全量编译
colcon build --symlink-install

# 树莓派版（限制并行）
colcon build --parallel-workers 2 --symlink-install

# 增量编译（指定包）
colcon build --packages-select n10p_bringup n10p_slam n10p_nav

# 编译后激活
source install/setup.bash
```

## 话题检查

```bash
ros2 topic list                          # 列出所有话题
ros2 topic hz /scan                      # 雷达频率（应为 10Hz）
ros2 topic hz /odom                      # 里程计频率
ros2 topic echo /scan --once             # 查看一帧激光数据
ros2 topic info /scan                    # 查看发布者/订阅者
```

## TF检查

```bash
ros2 run tf2_ros tf2_echo odom base_link           # odom→base_link
ros2 run tf2_ros tf2_echo base_link laser_frame    # base_link→laser
ros2 run tf2_tools view_frames                     # 生成TF树PDF
```

## 节点检查

```bash
ros2 node list                          # 列出所有节点
ros2 node info /slam_toolbox            # 查看SLAM节点详情
ros2 node info /amcl                    # 查看AMCL发布者
```

## 服务检查

```bash
ros2 lifecycle get /planner_server      # 应为 active [3]
ros2 action list                        # 列出action server
ros2 service list                       # 列出service
```

## 保存/查看地图

```bash
# 保存地图 (slam-toolbox自带服务)
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/path/to/maps/n10p_map'}}"

# 查看地图
python3 scripts/map_viewer.py maps/n10p_map.yaml
```

## 常用监控

```bash
ros2 topic echo /scan --once --field ranges  # 看距离数据
ros2 topic echo /plan --once                 # 看规划路径
ros2 topic echo /cmd_vel --once              # 看速度指令
ros2 topic echo /amcl_pose --once            # 看AMCL定位结果
```
