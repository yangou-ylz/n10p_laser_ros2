#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ano_data_logger.py — 凌霄飞控串口数据记录/调试工具
====================================================

独立运行（无需 ROS2），实时显示飞控串口发来的所有数据帧。
可用于：验证硬件接线、检查帧率、记录飞行数据。

用法:
    # 实时显示所有帧
    python3 ano_data_logger.py --port /dev/ttyAMA0 --baud 500000

    # 只显示四元数帧
    python3 ano_data_logger.py --port /dev/ttyAMA0 --filter 0x04

    # 记录 30 秒数据到 CSV
    python3 ano_data_logger.py --port /dev/ttyAMA0 --output flight.csv --duration 30

    # 列出所有已知帧类型
    python3 ano_data_logger.py --list
"""

import sys
import os
import time
import argparse
import csv
import signal
from typing import Dict, Optional

# ── 处理直接运行 vs 包导入 ───────────────────────────────────────────
try:
    from .ano_protocol import FRAME_REGISTRY, FRAME_NAME, FrameDef
    from .ano_transport import SerialTransport
except ImportError:
    # 直接运行脚本时的回退导入
    _here = os.path.dirname(os.path.abspath(__file__))
    if _here not in sys.path:
        sys.path.insert(0, _here)
    from ano_protocol import FRAME_REGISTRY, FRAME_NAME, FrameDef
    from ano_transport import SerialTransport


# ═══════════════════════════════════════════════════════════════════════
# 帧显示格式化器
# ═══════════════════════════════════════════════════════════════════════

def _fmt(d: dict, cmd: int) -> str:
    """
    将解码后的帧字典格式化为一行可读字符串。

    每个帧类型有专用格式化逻辑，只显示关键字段。
    未知帧显示 hex 原始数据。
    """
    if 'error' in d:
        return f"[错误] {d['error']}  raw={d.get('raw', '?')}"
    if 'raw' in d and len(d) == 1:
        return f"[原始] {d['raw']}"

    try:
        if cmd == 0x01:  # 惯性传感器
            return (f"ACC=({d['acc_x']:+6d},{d['acc_y']:+6d},{d['acc_z']:+6d})  "
                    f"GYR=({d['gyr_x']:+6d},{d['gyr_y']:+6d},{d['gyr_z']:+6d})  "
                    f"振动={'⚠' if d['shock'] else '✓'}")

        elif cmd == 0x02:  # 气压/磁力
            return (f"Mag=({d['mag_x']:+5d},{d['mag_y']:+5d},{d['mag_z']:+5d})  "
                    f"气压高度={d['baro_alt_cm']}cm  温度={d['temp_c']:.1f}°C  "
                    f"mag_sta={d['mag_sta']} baro_sta={d['baro_sta']}")

        elif cmd == 0x03:  # 欧拉角（低频）
            return (f"Roll={d['roll_deg']:+7.2f}°  Pitch={d['pitch_deg']:+7.2f}°  "
                    f"Yaw={d['yaw_deg']:+7.2f}°  sta={d['fusion_sta']}  ⚠低频")

        elif cmd == 0x04:  # 四元数
            return (f"Roll={d['roll_deg']:+7.2f}°  Pitch={d['pitch_deg']:+7.2f}°  "
                    f"Yaw={d['yaw_deg']:+7.2f}°  "
                    f"Q=({d['w']:+.4f},{d['x']:+.4f},{d['y']:+.4f},{d['z']:+.4f})  "
                    f"sta={d['fusion_sta']}")

        elif cmd == 0x05:  # 融合高度
            return (f"高度={d['alt_fused_cm']}cm  附加={d['alt_add_cm']}cm  "
                    f"sta={d['sta']}")

        elif cmd == 0x06:  # 飞控状态
            return (f"模式={d['mode_str']}  解锁={'✓已解锁' if d['unlocked'] else '✗上锁'}  "
                    f"CID=0x{d['cmd_cid']:02X}  cmd=({d['cmd_0']},{d['cmd_1']})")

        elif cmd == 0x07:  # 速度
            return (f"Vx={d['vel_x_cms']:+5d}  Vy={d['vel_y_cms']:+5d}  "
                    f"Vz={d['vel_z_cms']:+5d} cm/s")

        elif cmd == 0x08:  # XY位移
            return f"Pos=({d['pos_x_cm']:+6d},{d['pos_y_cm']:+6d}) cm"

        elif cmd == 0x09:  # 风速
            return f"风速=({d['wind_x_cms']:+5d},{d['wind_y_cms']:+5d}) cm/s"

        elif cmd == 0x0F:  # 系统心跳
            return f"状态字=0x{d.get('status', 0):08X}"

        elif cmd == 0x30:  # 外部传感器
            return f"[原始] {d.get('raw', '?')}"

        elif cmd == 0x0A:  # 目标姿态
            return (f"目标 Roll={d['roll_deg']:+7.2f}°  Pitch={d['pitch_deg']:+7.2f}°  "
                    f"Yaw={d['yaw_deg']:+7.2f}°")

        elif cmd == 0x0D:  # 电池
            return f"电压={d['voltage_v']:.2f}V  电流={d['current_a']:.2f}A"

        elif cmd == 0x0E:  # 模块状态
            return (f"速度传感器={d.get('sta_gvel_str', d['sta_gvel'])}  "
                    f"位置传感器={d.get('sta_gpos_str', d['sta_gpos'])}  "
                    f"GPS={d.get('sta_gps_str', d['sta_gps'])}  "
                    f"高度辅助={d.get('sta_alt_str', d['sta_alt'])}")

        elif cmd == 0x20:  # 电机PWM
            motors = [f"{k}={v}" for k, v in sorted(d.items())]
            return "电机PWM: " + " ".join(motors)

        elif cmd == 0x21:  # 姿态控制量
            return (f"ctrl_roll={d['ctrl_roll']:+6d}  ctrl_pitch={d['ctrl_pitch']:+6d}  "
                    f"ctrl_yaw={d['ctrl_yaw']:+6d}  ctrl_thr={d['ctrl_thr']:+6d}")

        elif cmd == 0x40:  # 遥控器
            return (f"CH1~4=({d['ch_roll']},{d['ch_pitch']},{d['ch_throttle']},{d['ch_yaw']})  "
                    f"AUX1={d['ch_aux1']}  AUX2={d['ch_aux2']}")

        elif cmd == 0x41:  # 实时控制
            return (f"目标 Roll={d['roll_deg']:+.2f}° Pitch={d['pitch_deg']:+.2f}°  "
                    f"油门={d['thr_pct']:.1f}%  YawRate={d['yaw_rate']}°/s  "
                    f"Vel=({d['vel_x_cms']},{d['vel_y_cms']},{d['vel_z_cms']}) cm/s")

        elif cmd == 0xA0:  # 日志
            return f"[{d['color']}] {d['text']}"

        elif cmd == 0xE0:  # CMD命令
            return f"CID=0x{d['cid']:02X}  cmd_0={d['cmd_0']}  cmd_1={d['cmd_1']}"

        elif cmd == 0x00:  # CK应答
            return f"应答 for_cmd=0x{d['for_cmd']:02X}  sc={d['sc']}  ac={d['ac']}"

        else:
            # 未知帧：显示所有字段
            parts = [f"{k}={v}" for k, v in d.items()]
            return " ".join(parts)

    except Exception:
        return str(d)


# ═══════════════════════════════════════════════════════════════════════
# 数据记录器
# ═══════════════════════════════════════════════════════════════════════

class AnoDataLogger:
    """
    凌霄飞控串口数据记录器。

    实时显示飞控发来的数据帧，可选 CSV 文件记录。
    """

    def __init__(self, port: str, baud: int = 500000,
                 output_csv: Optional[str] = None,
                 filter_cmd: Optional[int] = None,
                 stats_interval: float = 5.0):
        """
        参数:
            port:           串口路径
            baud:           波特率
            output_csv:     CSV 输出文件路径（None = 不记录文件）
            filter_cmd:     只显示指定 CMD 的帧（None = 显示全部）
            stats_interval: 统计信息打印间隔（秒）
        """
        self.port = port
        self.baud = baud
        self.output_csv = output_csv
        self.filter_cmd = filter_cmd
        self.stats_interval = stats_interval

        # 传输层
        self._transport = SerialTransport(port, baud)

        # CSV 写入器
        self._csv_file = None
        self._csv_writer = None
        if output_csv:
            self._csv_file = open(output_csv, 'w', newline='', encoding='utf-8')
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow(['timestamp', 'cmd', 'cmd_name', 'data'])

        # 统计计时
        self._last_stats_time = 0.0
        self._running = True

        # 注册所有已知帧的回调
        for cmd in FRAME_REGISTRY:
            self._transport.register_callback(cmd, self._on_frame)

    # ── 帧回调 ────────────────────────────────────────────────────

    def _on_frame(self, decoded: dict) -> None:
        """收到帧时的回调（在传输层接收线程中执行）"""
        if not self._running:
            return

        # 提取帧信息
        cmd = decoded.get('cmd', None)
        if cmd is None:
            # 从 decoded 中反向查找 CMD（decode_frame 不返回 cmd）
            for known_cmd, latest in self._transport.get_all_latest().items():
                if latest is decoded:
                    cmd = known_cmd
                    break

        # 实际没法反向找，用另一种方式：回调注册时通过闭包绑定 cmd
        # 在 register_all 中解决这个问题
        pass

    def _make_callback(self, cmd: int):
        """为指定 cmd 创建闭包回调（携带 cmd 信息）"""
        def callback(decoded: dict):
            if not self._running:
                return
            now = time.time()
            ts_str = time.strftime('%H:%M:%S', time.localtime(now)) + f'.{int((now % 1) * 1000):03d}'

            # 过滤
            if self.filter_cmd is not None and cmd != self.filter_cmd:
                return

            # 格式化
            name = FRAME_NAME.get(cmd, f'0x{cmd:02X}')
            line = _fmt(decoded, cmd)

            # 终端输出
            print(f"[{ts_str}] 0x{cmd:02X} {name:<15s} | {line}")

            # CSV 记录
            if self._csv_writer:
                self._csv_writer.writerow([ts_str, f'0x{cmd:02X}', name, str(decoded)])
                # 定期刷新到磁盘
                if self._csv_file and cmd == 0x01:  # 高频帧，刷新频繁些
                    self._csv_file.flush()

        return callback

    def register_callbacks(self) -> None:
        """重新注册所有帧回调（带 cmd 信息的闭包）"""
        for cmd in FRAME_REGISTRY:
            self._transport.register_callback(cmd, self._make_callback(cmd))

    # ── 运行 ──────────────────────────────────────────────────────

    def run(self, duration: float = 0.0) -> None:
        """
        启动并运行数据记录器。

        参数:
            duration: 运行时长（秒），0 = 一直运行直到 Ctrl+C
        """
        # 先注册带 cmd 信息的回调
        self.register_callbacks()

        # 启动传输层
        if not self._transport.start():
            print(f"错误: 无法打开串口 {self.port}")
            return

        print(f"串口 {self.port} @ {self.baud} bps 已打开")
        print(f"按 Ctrl+C 停止...")
        if self.output_csv:
            print(f"数据记录到: {self.output_csv}")
        if self.filter_cmd is not None:
            name = FRAME_NAME.get(self.filter_cmd, f'0x{self.filter_cmd:02X}')
            print(f"仅显示: {name}")
        print()

        self._last_stats_time = time.monotonic()
        deadline = time.monotonic() + duration if duration > 0 else float('inf')

        try:
            while self._running and time.monotonic() < deadline:
                time.sleep(0.2)

                # 定期打印统计
                if self.stats_interval > 0:
                    now = time.monotonic()
                    if now - self._last_stats_time >= self.stats_interval:
                        self._print_stats()
                        self._last_stats_time = now

        except KeyboardInterrupt:
            print("\n收到中断信号，正在停止...")
        finally:
            self._running = False
            self._transport.stop()
            self._print_stats()
            if self._csv_file:
                self._csv_file.close()
                print(f"CSV 已保存: {self.output_csv}")
            print("数据记录器已停止")

    def _print_stats(self) -> None:
        """打印帧率统计"""
        stats, errors = self._transport.stats()
        uptime = self._transport.uptime()
        total = sum(stats.values())

        if total == 0:
            print("── 统计 ── 尚未收到任何帧")
            return

        print(f"\n── 统计 (运行 {uptime:.0f}s, 校验错误 {errors}) ──")

        # 按帧率降序排列
        sorted_stats = sorted(stats.items(), key=lambda x: x[1] / max(uptime, 0.1), reverse=True)

        for cmd, count in sorted_stats:
            hz = count / max(uptime, 0.1)
            name = FRAME_NAME.get(cmd, f'0x{cmd:02X}')
            bar = '█' * min(int(hz / 2), 40)
            print(f"  0x{cmd:02X} {name:<15s} {count:>6d} 帧  {hz:>6.1f} Hz  {bar}")

        print(f"  合计: {total} 帧\n")

    def stop(self) -> None:
        """停止数据记录器"""
        self._running = False
        self._transport.stop()


# ═══════════════════════════════════════════════════════════════════════
# CLI 入口
# ═══════════════════════════════════════════════════════════════════════

def list_frames() -> None:
    """列出所有已知帧类型"""
    print(f"\n{'CMD':>6s}  {'名称':<16s}  {'频率':>8s}  {'方向':<12s}  说明")
    print("-" * 80)
    for cmd in sorted(FRAME_REGISTRY.keys()):
        fd = FRAME_REGISTRY[cmd]
        freq = f'{fd.freq_hz:.0f} Hz' if fd.freq_hz > 0 else '不定'
        print(f"  0x{cmd:02X}   {fd.name:<16s}  {freq:>8s}  {fd.direction:<12s}  {fd.desc}")
    print()


def main() -> None:
    parser = argparse.ArgumentParser(
        description='凌霄飞控串口数据记录/调试工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s --port /dev/ttyAMA0                    # 实时显示所有帧
  %(prog)s --port /dev/ttyAMA0 --filter 0x04      # 只显示四元数
  %(prog)s --port /dev/ttyAMA0 --output log.csv   # 记录到 CSV
  %(prog)s --port /dev/ttyAMA0 --duration 30      # 运行30秒
  %(prog)s --list                                  # 列出所有帧类型
        """,
    )
    parser.add_argument('--port', default='/dev/ttyAMA0',
                        help='串口路径（默认: /dev/ttyAMA0）')
    parser.add_argument('--baud', type=int, default=500000,
                        help='波特率（默认: 500000）')
    parser.add_argument('--output', default=None,
                        help='CSV 输出文件路径（可选）')
    parser.add_argument('--filter', type=str, default=None,
                        help='只显示指定 CMD 的帧，如 0x04')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='运行时长（秒），0=一直运行直到 Ctrl+C')
    parser.add_argument('--stats', type=float, default=5.0,
                        help='统计打印间隔（秒），0=不打印，默认 5')
    parser.add_argument('--list', action='store_true',
                        help='列出所有已知帧类型并退出')

    args = parser.parse_args()

    if args.list:
        list_frames()
        return

    # 解析 --filter
    filter_cmd = None
    if args.filter:
        filter_cmd = int(args.filter, 16) if args.filter.startswith('0x') else int(args.filter)

    logger = AnoDataLogger(
        port=args.port,
        baud=args.baud,
        output_csv=args.output,
        filter_cmd=filter_cmd,
        stats_interval=args.stats,
    )

    # 优雅处理 SIGTERM
    signal.signal(signal.SIGTERM, lambda *_: logger.stop())

    logger.run(duration=args.duration)


if __name__ == '__main__':
    main()
