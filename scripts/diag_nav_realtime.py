#!/usr/bin/env python3
"""
导航实时性诊断脚本 — 监控 TF 链更新频率、AMCL 响应速度、odom 数据流。

用法:
  先启动导航:
    ros2 launch n10p_nav nav_ekf_launch.py map:=/home/ylz/n10p_leishen/maps/n10p_map.yaml

  另开终端:
    python3 /home/ylz/n10p_leishen/scripts/diag_nav_realtime.py

输出:
  - 终端每秒打印各数据源更新频率和延迟
  - 保存完整日志到 logs/diag_realtime_<时间戳>.csv
"""
import rclpy, math, time, csv
from pathlib import Path
from collections import defaultdict
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan

LOG_DIR = Path("/home/ylz/n10p_leishen/logs")


class RealtimeDiag:
    def __init__(self):
        self.node = rclpy.create_node("_diag_realtime")
        self.t0 = time.monotonic()

        # 计数器和时间戳
        self.counts = defaultdict(int)
        self.last_stamps = defaultdict(float)
        self.first_stamps = {}

        # AMCL 最新数据
        self.amcl_pose = None
        self.amcl_seq = 0

        # odom 最新数据
        self.odom_linear = None   # (vx, vy)
        self.odom_angular = None

        # TF 链最新值
        self.odom_base_t = None   # odom→base_link translation (x,y)
        self.map_odom_t = None    # map→odom translation (x,y)

        # 订阅
        self.node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._cb_amcl, 10)
        self.node.create_subscription(Odometry, "/odom", self._cb_odom, 10)
        self.node.create_subscription(TFMessage, "/tf", self._cb_tf, 10)
        self.node.create_subscription(LaserScan, "/scan", self._cb_scan, 10)

    def _tick(self, key):
        now = time.monotonic()
        self.counts[key] += 1
        if key not in self.first_stamps:
            self.first_stamps[key] = now
        self.last_stamps[key] = now

    def _cb_amcl(self, msg):
        self._tick("amcl_pose")
        self.amcl_pose = msg
        self.amcl_seq += 1

    def _cb_odom(self, msg):
        self._tick("odom")
        self.odom_linear = (msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.odom_angular = msg.twist.twist.angular.z

    def _cb_tf(self, msg):
        for t in msg.transforms:
            key = f"{t.header.frame_id}->{t.child_frame_id}"
            self._tick(f"tf:{key}")
            if key == "odom->base_link":
                self.odom_base_t = (t.transform.translation.x, t.transform.translation.y)
            elif key == "map->odom":
                self.map_odom_t = (t.transform.translation.x, t.transform.translation.y)

    def _cb_scan(self, msg):
        self._tick("scan")

    def run(self, duration=60):
        LOG_DIR.mkdir(parents=True, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        log_path = LOG_DIR / f"diag_realtime_{ts}.csv"

        with open(log_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "elapsed_s",
                "amcl_pose_hz", "odom_hz", "scan_hz",
                "tf_odom_base_hz", "tf_map_odom_hz",
                "amcl_x", "amcl_y", "amcl_yaw_deg",
                "odom_base_x", "odom_base_y",
                "map_odom_x", "map_odom_y",
                "note",
            ])

            print("=" * 78)
            print("  导航实时性诊断 — 运行 60 秒")
            print(f"  日志: {log_path}")
            print("=" * 78)
            print(f"{'t(s)':>5s} | {'AMCL':>6s} | {'odom':>6s} | {'scan':>6s} | {'o→b_TF':>7s} | {'m→o_TF':>7s} | amcl(x,y,yaw°) | 备注")
            print("-" * 78)

            last_print = self.t0
            prev_counts = defaultdict(int)

            while time.monotonic() - self.t0 < duration:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                now = time.monotonic()
                elapsed = now - self.t0

                if now - last_print < 1.0:
                    continue

                # 计算各数据源频率
                rates = {}
                for key in ["amcl_pose", "odom", "scan", "tf:odom->base_link", "tf:map->odom"]:
                    cnt = self.counts[key] - prev_counts[key]
                    rates[key] = cnt / (now - last_print)
                    prev_counts[key] = self.counts[key]

                # AMCL 位姿
                note = ""
                amcl_x = amcl_y = amcl_yaw = 0
                odom_bx = odom_by = 0
                map_ox = map_oy = 0

                if self.amcl_pose:
                    p = self.amcl_pose.pose.pose
                    amcl_x, amcl_y = p.position.x, p.position.y
                    q = p.orientation
                    siny = 2*(q.w*q.z + q.x*q.y)
                    cosy = 1 - 2*(q.z*q.z + q.y*q.y)
                    amcl_yaw = math.degrees(math.atan2(siny, cosy))

                if self.odom_base_t:
                    odom_bx, odom_by = self.odom_base_t
                if self.map_odom_t:
                    map_ox, map_oy = self.map_odom_t

                # 检测异常
                if rates.get("tf:odom->base_link", 0) < 20:
                    note += "⚠️ odom→base TF 低频! "
                if rates.get("amcl_pose", 0) < 5:
                    note += "⚠️ AMCL 低频! "
                if self.odom_base_t and self.odom_base_t[0] == 0 and self.odom_base_t[1] == 0:
                    # odom translation always zero — this is expected per memory
                    pass

                print(f"{elapsed:5.1f}s | {rates.get('amcl_pose',0):5.1f}Hz | {rates.get('odom',0):5.1f}Hz | "
                      f"{rates.get('scan',0):5.1f}Hz | {rates.get('tf:odom->base_link',0):6.1f}Hz | "
                      f"{rates.get('tf:map->odom',0):6.1f}Hz | "
                      f"({amcl_x:.2f},{amcl_y:.2f},{amcl_yaw:.0f}°) | {note}")

                writer.writerow([
                    f"{elapsed:.2f}",
                    f"{rates.get('amcl_pose',0):.1f}", f"{rates.get('odom',0):.1f}",
                    f"{rates.get('scan',0):.1f}", f"{rates.get('tf:odom->base_link',0):.1f}",
                    f"{rates.get('tf:map->odom',0):.1f}",
                    f"{amcl_x:.3f}", f"{amcl_y:.3f}", f"{amcl_yaw:.2f}",
                    f"{odom_bx:.3f}", f"{odom_by:.3f}",
                    f"{map_ox:.3f}", f"{map_oy:.3f}",
                    note,
                ])
                last_print = now

        # 总结
        print("-" * 78)
        for key in ["amcl_pose", "odom", "scan", "tf:odom->base_link", "tf:map->odom"]:
            if key in self.first_stamps:
                t = self.last_stamps[key] - self.first_stamps[key]
                avg_hz = self.counts[key] / t if t > 0 else 0
                print(f"  {key:20s}: {self.counts[key]:5d} 条, 平均 {avg_hz:.1f} Hz")

        print(f"\n日志已保存: {log_path}")
        self.node.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init(args=[])
    diag = RealtimeDiag()
    try:
        diag.run(60)
    except KeyboardInterrupt:
        print("\n用户中断")


if __name__ == "__main__":
    main()
