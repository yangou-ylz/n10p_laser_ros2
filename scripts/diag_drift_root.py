#!/usr/bin/env python3
"""根因探查: 同时捕获速度链+TF链+AMCL修正, 定位位置漂移来源。

用法: python3 /home/ylz/n10p_leishen/scripts/diag_drift_root.py
要求: 导航已启动, 飞: 后→前→左→右, 每方向~50cm, 间停3s, 最后回原点降落
时长: 120s, 5Hz采样
"""
import rclpy, math, time, csv
from pathlib import Path
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

LOG_DIR = Path("/home/ylz/n10p_leishen/logs")

rclpy.init(args=[])
node = rclpy.create_node("_diag_drift_root")

# ── 7路数据缓存 ──────────────────────────────
fc_raw = None          # /fc_vel_raw: FC死区前原始速度
odom_vel = None        # /odom: 死区+符号后速度
filt_vel = None        # /odometry/filtered: imu_filter后速度
filt_pos = None        # /odometry/filtered: 积分位置
map_odom_tf = None     # /tf: map→odom (AMCL修正量)
odom_base_tf = None    # /tf: odom→base_link
amcl_pose = None       # /amcl_pose: 地图中位姿

best_effort = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=5)

def cb_fc(msg):    global fc_raw; fc_raw = (msg.twist.linear.x, msg.twist.linear.y)
def cb_odom(msg):
    global odom_vel; odom_vel = (msg.twist.twist.linear.x, msg.twist.twist.linear.y)
def cb_filt(msg):
    global filt_vel, filt_pos
    filt_vel = (msg.twist.twist.linear.x, msg.twist.twist.linear.y)
    filt_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
def cb_tf(msg):
    global map_odom_tf, odom_base_tf
    for t in msg.transforms:
        if t.header.frame_id == 'map' and t.child_frame_id == 'odom':
            q = t.transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            map_odom_tf = (t.transform.translation.x, t.transform.translation.y, math.degrees(yaw))
        elif t.header.frame_id == 'odom' and t.child_frame_id == 'base_link':
            q = t.transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            odom_base_tf = (t.transform.translation.x, t.transform.translation.y, math.degrees(yaw))
def cb_amcl(msg):
    global amcl_pose
    q = msg.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    amcl_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, math.degrees(yaw))

node.create_subscription(TwistStamped, '/fc_vel_raw', cb_fc, best_effort)
node.create_subscription(Odometry, '/odom', cb_odom, best_effort)
node.create_subscription(Odometry, '/odometry/filtered', cb_filt, best_effort)
node.create_subscription(TFMessage, '/tf', cb_tf, 10)
node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', cb_amcl, best_effort)

LOG_DIR.mkdir(parents=True, exist_ok=True)
ts = time.strftime("%Y%m%d_%H%M%S")
log_path = LOG_DIR / f"diag_drift_root_{ts}.csv"

with open(log_path, 'w', newline='') as f:
    writer = csv.writer(f)
    # 25列: 完整数据链
    writer.writerow([
        'elapsed_s',
        # 速度链: 原始 → 死区 → 滤波
        'fc_vx','fc_vy', 'odom_vx','odom_vy', 'filt_vx','filt_vy',
        # 位置链: 滤波积分位置
        'filt_px','filt_py',
        # TF链: odom→base
        'odom_base_x','odom_base_y','odom_base_yaw',
        # TF链: map→odom (AMCL修正)
        'map_odom_x','map_odom_y','map_odom_yaw',
        # AMCL绝对位姿
        'amcl_x','amcl_y','amcl_yaw',
        # 衍生指标
        'amcl_delta',        # AMCL修正量变化率 (m/s)
        'pos_gap_x',         # filt位置 vs AMCL位置的差距X
        'pos_gap_y',         # filt位置 vs AMCL位置的差距Y
        'odom_drift_speed',  # odom漂移速度
    ])

    print("=" * 95)
    print("  根因探查 — 飞: 后→前→左→右 (各50cm, 间停3s), 最后回原点降落")
    print(f"  日志: {log_path}")
    print("=" * 95)
    print(f"{'t':>5s} | {'FC原始':>14s} | {'滤波位置':>14s} | {'AMCL':>18s} | {'位置差(cm)':>11s} | 状态")
    print("-" * 95)

    t0 = time.monotonic()
    last_sample = t0
    prev_map_odom = None
    start_amcl = None
    start_filt_pos = None

    while time.monotonic() - t0 < 100:
        rclpy.spin_once(node, timeout_sec=0.01)
        now = time.monotonic()
        if now - last_sample < 0.2:
            continue

        elapsed = now - t0
        fv  = fc_raw if fc_raw else (0,0)
        ov  = odom_vel if odom_vel else (0,0)
        flv = filt_vel if filt_vel else (0,0)
        flp = filt_pos if filt_pos else (0,0)
        obt = odom_base_tf if odom_base_tf else (0,0,0)
        mot = map_odom_tf if map_odom_tf else (0,0,0)
        acl = amcl_pose if amcl_pose else (0,0,0)

        # 记录起点
        if start_amcl is None and acl[0] != 0:
            start_amcl = (acl[0], acl[1])
            start_filt_pos = (flp[0], flp[1])
            print(f"  >>> 起点: AMCL=({start_amcl[0]:.3f},{start_amcl[1]:.3f})")

        # AMCL修正变化率
        amcl_delta = 0.0
        if prev_map_odom is not None:
            dm = math.sqrt((mot[0]-prev_map_odom[0])**2 + (mot[1]-prev_map_odom[1])**2)
            amcl_delta = dm / (now - last_sample)
        prev_map_odom = (mot[0], mot[1])

        # 位置差: filt积分 vs AMCL (过滤波位置减AMCL位置, 正值=filt比AMCL更前/左)
        pos_gap_x = flp[0] - acl[0]
        pos_gap_y = flp[1] - acl[1]
        pos_gap_cm = math.sqrt(pos_gap_x**2 + pos_gap_y**2) * 100

        # odom漂移速度
        odom_drift = math.sqrt(flv[0]**2 + flv[1]**2) if abs(flv[0])<0.03 and abs(flv[1])<0.03 else 0

        # 状态标记
        status = ""
        if abs(fv[0]) > 0.1: status = "→ 前飞" if fv[0] > 0 else "← 后飞"
        elif abs(fv[1]) > 0.1: status = "↑ 左飞" if fv[1] > 0 else "↓ 右飞"
        elif pos_gap_cm > 15: status = "⚠ 偏差大!"

        print(f"{elapsed:5.1f}s | ({fv[0]:+6.3f},{fv[1]:+6.3f}) | ({flp[0]:+7.3f},{flp[1]:+7.3f}) | "
              f"({acl[0]:+7.3f},{acl[1]:+7.3f},{acl[2]:+6.1f}°) | {pos_gap_cm:6.1f}cm |{status}")

        writer.writerow([f"{elapsed:.2f}",
            f"{fv[0]:.4f}",f"{fv[1]:.4f}", f"{ov[0]:.4f}",f"{ov[1]:.4f}",
            f"{flv[0]:.4f}",f"{flv[1]:.4f}", f"{flp[0]:.4f}",f"{flp[1]:.4f}",
            f"{obt[0]:.4f}",f"{obt[1]:.4f}",f"{obt[2]:.2f}",
            f"{mot[0]:.4f}",f"{mot[1]:.4f}",f"{mot[2]:.2f}",
            f"{acl[0]:.4f}",f"{acl[1]:.4f}",f"{acl[2]:.2f}",
            f"{amcl_delta:.6f}", f"{pos_gap_x:.4f}",f"{pos_gap_y:.4f}", f"{odom_drift:.4f}"])

        last_sample = now

    # 汇总
    print("-" * 95)
    if start_amcl:
        end_amcl = (acl[0], acl[1]) if amcl_pose else (0,0)
        net_drift = math.sqrt((end_amcl[0]-start_amcl[0])**2 + (end_amcl[1]-start_amcl[1])**2)
        print(f"  起点AMCL: ({start_amcl[0]:.3f},{start_amcl[1]:.3f})  终点AMCL: ({end_amcl[0]:.3f},{end_amcl[1]:.3f})")
        print(f"  AMCL净漂移: {net_drift*100:.1f}cm")
        if net_drift > 0.2:
            print(f"  ⚠ 漂移>20cm, 需要排查!")
        else:
            print(f"  ✓ 漂移在可接受范围内")
    print(f"  日志: {log_path}")
    node.destroy_node()
    rclpy.shutdown()
