#!/usr/bin/env python3
"""导航实时跟踪诊断: 监控 AMCL修正量、TF位移、滤波速度、扫描匹配是否跟上。

用法: python3 /home/ylz/n10p_leishen/scripts/diag_nav_tracking.py
要求: 导航已启动, 移动无人机进行测试
时长: 120秒, 每秒2次采样
"""
import rclpy, math, time, csv
from pathlib import Path
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

LOG_DIR = Path("/home/ylz/n10p_leishen/logs")

rclpy.init(args=[])
node = rclpy.create_node("_diag_nav_track")

# ── 缓存 ────────────────────────────────────
amcl_pose = None        # (x, y, yaw_deg)  AMCL 认为的机器人在地图中位姿
map_odom_tf = None      # (x, y, yaw_deg)  AMCL 发布的修正量
odom_base_tf = None     # (x, y, yaw_deg)  imu_filter 积分出的里程计位姿
filt_vel = None         # (vx, vy)  滤波后速度
scan_count = 0          # 扫描帧计数
last_scan_reset = time.monotonic()

best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=5)

def cb_amcl(msg: PoseWithCovarianceStamped):
    global amcl_pose
    q = msg.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    amcl_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, math.degrees(yaw))

def cb_tf(msg: TFMessage):
    global map_odom_tf, odom_base_tf
    for t in msg.transforms:
        if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
            q = t.transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            map_odom_tf = (t.transform.translation.x, t.transform.translation.y, math.degrees(yaw))
        elif t.header.frame_id == 'odom' and t.child_frame_id == 'base_link':
            q = t.transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            odom_base_tf = (t.transform.translation.x, t.transform.translation.y, math.degrees(yaw))

def cb_filt(msg: Odometry):
    global filt_vel
    filt_vel = (msg.twist.twist.linear.x, msg.twist.twist.linear.y)

def cb_scan(msg):
    global scan_count
    scan_count += 1

node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', cb_amcl, best_effort)
node.create_subscription(TFMessage, '/tf', cb_tf, 10)
node.create_subscription(Odometry, '/odometry/filtered', cb_filt, best_effort)
node.create_subscription(LaserScan, '/scan', cb_scan, best_effort)

LOG_DIR.mkdir(parents=True, exist_ok=True)
ts = time.strftime("%Y%m%d_%H%M%S")
log_path = LOG_DIR / f"diag_nav_tracking_{ts}.csv"

with open(log_path, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['elapsed_s',
                     # AMCL 地图位姿
                     'amcl_x', 'amcl_y', 'amcl_yaw_deg',
                     # AMCL 修正量 (map→odom)
                     'map_odom_x', 'map_odom_y', 'map_odom_yaw_deg',
                     # 里程计位姿 (odom→base_link, 积分)
                     'odom_base_x', 'odom_base_y', 'odom_base_yaw_deg',
                     # 滤波速度
                     'filt_vx', 'filt_vy',
                     # 扫描速率 (帧/秒, 近2秒平均)
                     'scan_hz',
                     # 综合指标: AMCL修正量的变化率 (越大越活跃)
                     'amcl_active'])

    print("=" * 85)
    print("  导航实时跟踪诊断 — 移动无人机！")
    print(f"  日志: {log_path}")
    print("=" * 85)
    header = (f"{'t':>5s} | {'AMCL位姿':>18s} | {'map→odom修正':>18s} | "
              f"{'odom→base':>18s} | {'滤波速度':>12s} | {'扫描':>6s} | {'AMCL活跃':>9s}")
    print(header)
    print("-" * 85)

    t0 = time.monotonic()
    last_sample = t0
    last_scan_reset = t0
    last_scan_count = 0
    prev_map_odom = None
    amcl_active = 0.0

    while time.monotonic() - t0 < 120:
        rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic()

        # 每秒2次采样
        if now - last_sample < 0.5:
            continue

        elapsed = now - t0

        ax, ay, ayaw = amcl_pose if amcl_pose else (0, 0, 0)
        mx, my, myaw = map_odom_tf if map_odom_tf else (0, 0, 0)
        ox, oy, oyaw = odom_base_tf if odom_base_tf else (0, 0, 0)
        vx, vy = filt_vel if filt_vel else (0, 0)

        # 扫描速率
        scan_dt = now - last_scan_reset
        if scan_dt > 1.5:
            scan_hz = scan_count / scan_dt
            scan_count = 0
            last_scan_reset = now
        else:
            scan_hz = 0

        # AMCL 活跃度: map→odom 每秒变化量
        if prev_map_odom is not None:
            dm = math.sqrt((mx - prev_map_odom[0])**2 + (my - prev_map_odom[1])**2)
            amcl_active = dm / (now - last_sample)  # m/s 变化率
        prev_map_odom = (mx, my)

        activity_mark = ""
        if amcl_active > 0.01:
            activity_mark = "●"  # AMCL 在活跃修正
        elif amcl_active > 0.001:
            activity_mark = "○"  # AMCL 缓慢修正
        else:
            activity_mark = "·"  # AMCL 基本不动

        print(f"{elapsed:5.1f}s | ({ax:7.3f},{ay:7.3f},{ayaw:6.1f}°) | "
              f"({mx:7.3f},{my:7.3f},{myaw:6.1f}°) | "
              f"({ox:7.3f},{oy:7.3f},{oyaw:6.1f}°) | "
              f"({vx:5.3f},{vy:5.3f}) | {scan_hz:5.1f}Hz | {activity_mark} {amcl_active:7.4f}")

        writer.writerow([f"{elapsed:.2f}",
                         f"{ax:.4f}", f"{ay:.4f}", f"{ayaw:.2f}",
                         f"{mx:.4f}", f"{my:.4f}", f"{myaw:.2f}",
                         f"{ox:.4f}", f"{oy:.4f}", f"{oyaw:.2f}",
                         f"{vx:.4f}", f"{vy:.4f}",
                         f"{scan_hz:.1f}",
                         f"{amcl_active:.6f}"])

        last_sample = now

print("-" * 85)
print(f"日志: {log_path}")
node.destroy_node()
rclpy.shutdown()
