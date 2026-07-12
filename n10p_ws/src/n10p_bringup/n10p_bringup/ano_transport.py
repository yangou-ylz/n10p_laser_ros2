#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ano_transport.py — 凌霄匿名协议串口传输层
==========================================

职责：串口生命周期管理、后台接收线程、帧同步与提取、回调分发、统计。
特点：
  - 不依赖 ROS2，可在任何 Python 程序中使用
  - 后台线程自动读串口、找帧头、校验、解码、回调
  - 校验失败跳 1 字节重同步（不丢整帧）
  - 每帧类型保留最新解码值（线程安全）
  - 串口异常不崩溃，自动记录并继续
  - 支持发送帧（线程安全）

用法:
    from ano_transport import SerialTransport

    transport = SerialTransport('/dev/ttyAMA0', 500000)
    transport.register_callback(0x04, lambda d: print(f"Roll={d['roll_deg']:.2f}°"))
    transport.start()

    # 主线程做其他事，回调在后台自动触发
    import time
    time.sleep(10)

    stats, errors = transport.stats()
    print(f"收到 {sum(stats.values())} 帧, {errors} 校验错误")
    transport.stop()
"""

import time
import threading
import logging
from typing import Callable, Dict, Optional, Tuple, List

import serial

# ── 兼容包导入和直接运行 ───────────────────────────────────────────
try:
    from .ano_protocol import (
        FRAME_HEAD,
        ADDR_BROADCAST,
        verify_frame,
        decode_frame,
        build_frame,
        FRAME_NAME,
    )
except ImportError:
    from ano_protocol import (
        FRAME_HEAD,
        ADDR_BROADCAST,
        verify_frame,
        decode_frame,
        build_frame,
        FRAME_NAME,
    )

logger = logging.getLogger(__name__)


class SerialTransport:
    """
    凌霄匿名协议串口传输层。

    打开串口 → 后台线程持续读取 → 帧同步 → 校验 → 解码 → 回调分发。
    同时维护每帧类型的最新解码数据缓存（线程安全）。

    回调在接收线程中同步执行，应保持轻量（不阻塞 I/O）。
    如需在回调中做耗时操作，应将数据放入队列，由另一个线程处理。
    """

    # ── 构造与生命周期 ────────────────────────────────────────────────

    def __init__(self, port: str, baud: int = 500000, timeout: float = 0.02):
        """
        初始化传输层（不立即连接）。

        参数:
            port:    串口路径，如 '/dev/ttyAMA0' 或 '/dev/ttyUSB0'
            baud:    波特率，默认 500000（飞控 UART5 总线速率）
            timeout: 串口读取超时（秒），控制 _recv_loop 的轮询频率
        """
        self._port = port
        self._baud = baud
        self._timeout = timeout

        # 串口对象（start 后才有值）
        self._ser: Optional[serial.Serial] = None

        # 后台线程
        self._thread: Optional[threading.Thread] = None
        self._running = False

        # 接收缓冲区
        self._buf = bytearray()

        # 线程安全保护
        self._lock = threading.Lock()       # 保护 _latest, _callbacks, _stats
        self._send_lock = threading.Lock()  # 保护串口写入

        # 最新帧缓存: {cmd: (decoded_dict, timestamp_monotonic)}
        self._latest: Dict[int, Tuple[dict, float]] = {}

        # 回调注册: {cmd: [callable, ...]}
        self._callbacks: Dict[int, List[Callable[[dict], None]]] = {}

        # 统计: {cmd: count}
        self._stats: Dict[int, int] = {}
        self._err_cnt = 0           # 校验失败计数
        self._bytes_read = 0        # 累计读取字节数
        self._start_time = 0.0      # 启动时间 (monotonic)

    def start(self) -> bool:
        """
        打开串口并启动后台接收线程。

        返回:
            True = 启动成功, False = 串口打开失败
        """
        if self._running:
            logger.warning("传输层已在运行中")
            return True

        try:
            self._ser = serial.Serial(
                self._port, self._baud,
                timeout=self._timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff=False,
                rtscts=False,
            )
        except serial.SerialException as e:
            logger.error("无法打开串口 %s: %s", self._port, e)
            self._ser = None
            return False

        self._running = True
        self._start_time = time.monotonic()
        self._thread = threading.Thread(target=self._recv_loop, daemon=True,
                                        name=f"ano-transport-{self._port}")
        self._thread.start()
        logger.info("传输层已启动: %s @ %d bps", self._port, self._baud)
        return True

    def stop(self) -> None:
        """停止后台线程并关闭串口。可重复调用。"""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        if self._ser is not None and self._ser.is_open:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        logger.info("传输层已停止: %s", self._port)

    def is_running(self) -> bool:
        """返回传输层是否正在运行"""
        return self._running and self._ser is not None and self._ser.is_open

    # ── 回调注册 ──────────────────────────────────────────────────────

    def register_callback(self, cmd: int, fn: Callable[[dict], None]) -> None:
        """
        注册帧回调函数。

        每次收到指定 CMD 的帧并通过校验后，自动调用 fn(decoded_dict)。
        回调在接收线程中执行，应保持轻量，禁止长时间阻塞。

        参数:
            cmd: 帧功能码（如 0x04 四元数帧）
            fn:  回调函数，签名为 fn(decoded: dict) -> None
        """
        with self._lock:
            if cmd not in self._callbacks:
                self._callbacks[cmd] = []
            self._callbacks[cmd].append(fn)
        name = FRAME_NAME.get(cmd, f'0x{cmd:02X}')
        logger.debug("已注册 %s 帧回调: %s", name, fn.__name__)

    def unregister_callback(self, cmd: int, fn: Callable[[dict], None]) -> None:
        """取消注册帧回调函数。"""
        with self._lock:
            if cmd in self._callbacks:
                try:
                    self._callbacks[cmd].remove(fn)
                except ValueError:
                    pass

    # ── 数据获取（线程安全）───────────────────────────────────────────

    def get_latest(self, cmd: int) -> Optional[dict]:
        """
        获取指定帧类型的最新解码数据。

        返回:
            解码后的字典；如果从未收到此帧类型，返回 None。
            注意：返回的是内部字典的引用，不要修改它。
        """
        with self._lock:
            entry = self._latest.get(cmd)
        return entry[0] if entry else None

    def get_age(self, cmd: int) -> Optional[float]:
        """
        获取指定帧距离上次收到多少秒。

        返回:
            秒数（浮点）；None 表示从未收到。
        """
        with self._lock:
            entry = self._latest.get(cmd)
        return time.monotonic() - entry[1] if entry else None

    def get_all_latest(self) -> Dict[int, dict]:
        """
        获取所有已收到帧类型的最新数据。

        返回:
            {cmd: decoded_dict} 的浅拷贝。
        """
        with self._lock:
            return {cmd: entry[0] for cmd, entry in self._latest.items()}

    def stats(self) -> Tuple[Dict[int, int], int]:
        """
        获取统计信息。

        返回:
            ({cmd: 接收帧数}, 校验错误数)
        """
        with self._lock:
            return dict(self._stats), self._err_cnt

    def uptime(self) -> float:
        """返回传输层已运行时间（秒），未启动返回 0.0"""
        if self._start_time == 0.0:
            return 0.0
        return time.monotonic() - self._start_time

    # ── 发送（线程安全）───────────────────────────────────────────────

    def send(self, dest: int, cmd: int, payload: bytes = b'') -> bool:
        """
        发送一帧完整数据。

        参数:
            dest:    目标地址
            cmd:     帧功能码
            payload: DATA 区字节数据

        返回:
            True = 发送成功, False = 串口不可用
        """
        if not self.is_running() or self._ser is None:
            logger.warning("串口未打开，无法发送")
            return False

        frame = build_frame(dest, cmd, payload)

        with self._send_lock:
            try:
                self._ser.write(frame)
                return True
            except serial.SerialException as e:
                logger.error("串口写入失败: %s", e)
                return False

    def send_cmd(self, cid: int, cmd_0: int, cmd_1: int = 0) -> bool:
        """
        发送 0xE0 CMD 命令帧（广播地址）。

        参数:
            cid:   命令类别（如 0x10 = 飞行控制类）
            cmd_0: 命令参数0
            cmd_1: 命令参数1

        返回:
            True = 发送成功

        常用命令:
            send_cmd(0x10, 0x01)  # 解锁
            send_cmd(0x10, 0x02)  # 上锁
            send_cmd(0x10, 0x03)  # 一键起飞
            send_cmd(0x10, 0x04)  # 一键降落
        """
        return self.send(ADDR_BROADCAST, 0xE0, bytes([cid, cmd_0, cmd_1]))

    # ── 内部：接收线程 ────────────────────────────────────────────────

    def _recv_loop(self) -> None:
        """
        后台接收主循环。

        持续从串口读取字节 → 追加到缓冲区 → 尝试解析帧。
        串口读取异常不导致线程退出，仅记录错误并继续。
        """
        read_errors = 0
        while self._running:
            try:
                if self._ser is None or not self._ser.is_open:
                    time.sleep(0.1)
                    continue

                # 非阻塞读取：只读当前可用的字节
                waiting = self._ser.in_waiting
                if waiting > 0:
                    chunk = self._ser.read(min(waiting, 4096))
                    if chunk:
                        self._buf.extend(chunk)
                        self._bytes_read += len(chunk)
                        self._parse_buffer()
                        read_errors = 0  # 成功读取，重置错误计数
                else:
                    # 无数据时短暂休眠，避免空转消耗 CPU
                    time.sleep(0.002)

            except serial.SerialException as e:
                read_errors += 1
                if read_errors <= 1 or read_errors % 100 == 0:
                    logger.error("串口读取错误 (第%d次): %s", read_errors, e)
                time.sleep(0.05)
            except Exception as e:
                logger.error("接收线程异常: %s", e, exc_info=True)
                time.sleep(0.1)

    # ── 内部：帧解析 ──────────────────────────────────────────────────

    def _parse_buffer(self) -> None:
        """
        从接收缓冲区中提取并解析完整帧。

        算法:
            1. 查找帧头 0xAA 的位置
            2. 如果找到，且前面有垃圾字节则丢弃
            3. 检查是否有足够字节构成完整帧（LEN + 6）
            4. 提取帧 → 校验 → 解码 → 回调分发 + 缓存
            5. 校验失败：只跳 1 字节重新搜索帧头（而不是丢弃整帧）
            6. 校验通过：移除整帧，继续处理下一个
            7. 数据不足：保留在缓冲区等待更多字节
        """
        buf = self._buf
        while len(buf) >= 6:
            # ── 找帧头 ──────────────────────────────────────
            idx = buf.find(FRAME_HEAD)
            if idx == -1:
                # 缓冲区全是垃圾，清空
                buf.clear()
                return
            if idx > 0:
                # 丢帧头前的垃圾字节
                del buf[:idx]
                # buf 已变化，重新获取引用
                buf = self._buf

            # ── 检查数据完整性 ──────────────────────────────
            if len(buf) < 4:
                # 连 LEN 字段都不够，等待更多数据
                break

            payload_len = buf[3]
            frame_total = 4 + payload_len + 2  # HEAD + DEST + CMD + LEN(=4) + DATA + SC + AC

            if len(buf) < frame_total:
                # 帧数据不完整，等待更多字节
                break

            # ── 提取帧并校验 ────────────────────────────────
            frame = bytes(buf[:frame_total])

            if verify_frame(frame):
                # 校验通过 → 解码 + 分发
                cmd = frame[2]
                payload = frame[4:4 + payload_len]
                decoded = decode_frame(cmd, payload)
                ts = time.monotonic()

                # 更新缓存 + 统计（线程安全）
                with self._lock:
                    self._latest[cmd] = (decoded, ts)
                    self._stats[cmd] = self._stats.get(cmd, 0) + 1

                # 回调分发（在接收线程中同步执行）
                with self._lock:
                    callbacks = list(self._callbacks.get(cmd, []))
                for fn in callbacks:
                    try:
                        fn(decoded)
                    except Exception as e:
                        name = getattr(fn, '__name__', str(fn))
                        logger.error("回调 %s 异常 (CMD=0x%02X): %s", name, cmd, e)

                # 移除已处理帧
                del buf[:frame_total]
                buf = self._buf
            else:
                # 校验失败 → 跳 1 字节重新搜索帧头
                # 这是关键设计：不丢弃 frame_total 字节，
                # 因为可能 DATA 区中恰好出现了 0xAA
                self._err_cnt += 1
                del buf[:1]
                buf = self._buf

        self._buf = buf
