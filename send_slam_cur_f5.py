# -*- coding: utf-8 -*-
"""发送真实 SLAM 当前坐标 (cur) 为 0xF5 帧。
阶段：SLAM 接入但不控飞 — tar=cur, TARGET_VALID=0, 仅回 ACK。

用法:
  python3 send_slam_cur_f5.py --port /dev/ttyUSB0 --rate 10 --duration 30

要求:
  - 需 AMCL 正在发布 /amcl_pose (先开导航)
  - SLAM 未收敛 → SLAM_VALID=0
  - 静止 30s 跑完后检查 log 中的 c= 波动
"""
from __future__ import annotations

import argparse
import logging
import logging.handlers
import math
import sys
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

from ano_protocol import (
    ADDR_FC_STM32,
    CMD_RPI_POSITION,
    FLAG_SLAM_VALID,
    FLAG_TARGET_VALID,
    INVALID_S32,
    build_f5_position,
    hex_dump,
)
from linux_serial import LinuxSerial


def _yaw_from_quat(w, x, y, z):
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


class SlamCurSender:
    def __init__(self, port: str, baud: int, logger: logging.Logger):
        self.port = port
        self.logger = logger
        self.ser = LinuxSerial(port, baud)
        self.ser.open()
        self.latest_pose = None   # (x_m, y_m, z_m, yaw_rad)
        self.cov = None           # (x_var, y_var, yaw_var)
        self.running = True
        self._lock = threading.Lock()

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose
        x, y, z = p.position.x, p.position.y, p.position.z
        q = p.orientation
        yaw = _yaw_from_quat(q.w, q.x, q.y, q.z)
        c = msg.pose.covariance
        with self._lock:
            self.latest_pose = (x, y, z, yaw)
            self.cov = (c[0], c[7], c[35])

    def is_slam_valid(self):
        """AMCL 收敛判定: 位置 σ<0.3m 且 yaw σ<0.3rad"""
        with self._lock:
            if self.cov is None or self.latest_pose is None:
                return False
            xv, yv, yv2 = self.cov
            return (math.sqrt(max(0, xv)) < 0.3 and
                    math.sqrt(max(0, yv)) < 0.3 and
                    math.sqrt(max(0, yv2)) < 0.3)

    def get_cur_cm(self):
        with self._lock:
            if self.latest_pose is None:
                return None, None, None
            x, y, z, _ = self.latest_pose
            return round(x * 100), round(y * 100), round(z * 100)

    def send_frame(self, cur_x, cur_y, cur_z, target_valid=False):
        slam_ok = self.is_slam_valid()
        flags = 0
        if slam_ok:
            flags |= FLAG_SLAM_VALID
        if target_valid:
            flags |= FLAG_TARGET_VALID

        # tar = cur (不控飞)
        if cur_x is None:  # SLAM not ready
            cur_x = cur_y = cur_z = INVALID_S32
        tar_x, tar_y, tar_z = cur_x, cur_y, cur_z

        frame = build_f5_position(ADDR_FC_STM32,
                                  cur_x, cur_y, cur_z,
                                  tar_x, tar_y, tar_z, flags)
        self.ser.write(frame)
        return frame, flags

    def reader_loop(self):
        buf = bytearray()
        while self.running:
            try:
                chunk = self.ser.read_nonblocking(max_bytes=4096, wait_s=0.05)
            except Exception as e:
                self.logger.error("serial read error: %s", e)
                return
            if chunk:
                buf.extend(chunk)
                while len(buf) >= 6:
                    if buf[0] != 0xAA:
                        buf.pop(0)
                        continue
                    if len(buf) < 4:
                        break
                    plen = buf[3]
                    total = 4 + plen + 2
                    if len(buf) < total:
                        break
                    frame = bytes(buf[:total])
                    del buf[:total]
                    if frame[2] == 0xA0 and plen >= 2:
                        text = frame[4:4+plen-1].decode('gbk', errors='replace')
                        self.logger.info("[RX 0xA0 %s] %s",
                                         {0:'BLACK',1:'RED',2:'GREEN'}.get(frame[4], f"C{frame[4]}"),
                                         text.strip('\x00'))
                    elif frame[2] == 0xA0:
                        self.logger.info("[RX 0xA0] (len=%d)", plen)

    def run(self, rate: float, duration: float):
        th = threading.Thread(target=self.reader_loop, daemon=True)
        th.start()
        interval = 1.0 / rate
        end_t = time.monotonic() + duration
        tx_cnt = 0
        while time.monotonic() < end_t and self.running:
            cur_x, cur_y, cur_z = self.get_cur_cm()
            frame, flags = self.send_frame(cur_x, cur_y, cur_z, target_valid=False)
            tx_cnt += 1
            if tx_cnt <= 3 or tx_cnt % 50 == 0:
                self.logger.info("[TX #%d] 0xF5 cur=(%s,%s,%s) flags=0x%02X hex=%s",
                                 tx_cnt, cur_x, cur_y, cur_z, flags,
                                 hex_dump(frame))
            time.sleep(interval)
        self.running = False
        th.join(timeout=2.0)
        self.ser.close()
        self.logger.info("done: %d frames sent", tx_cnt)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=500000)
    ap.add_argument("--rate", type=float, default=10.0, help="Hz")
    ap.add_argument("--duration", type=float, default=30.0, help="seconds")
    ap.add_argument("--log-file", default="logs/slam_cur_static.log")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger("send_slam_cur")
    logger.handlers.clear()
    fmt = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
    console = logging.StreamHandler(); console.setFormatter(fmt); logger.addHandler(console)
    path = Path(args.log_file); path.parent.mkdir(parents=True, exist_ok=True)
    fh = logging.handlers.RotatingFileHandler(path, maxBytes=2*1024*1024, backupCount=3, encoding='utf-8')
    fh.setFormatter(fmt); logger.addHandler(fh)

    rclpy.init(args=sys.argv)
    sender = SlamCurSender(args.port, args.baud, logger)

    node = rclpy.create_node('_slam_cur_sender')
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                              sender.pose_callback, 10)

    def spin():
        while sender.running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

    spin_th = threading.Thread(target=spin, daemon=True)
    spin_th.start()
    time.sleep(0.5)
    sender.run(args.rate, args.duration)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
