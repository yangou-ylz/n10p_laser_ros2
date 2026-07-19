# -*- coding: utf-8 -*-
"""Linux termios 串口 — 兼容 send_f5.py 的接口"""
from __future__ import annotations
import os, termios, fcntl, select, time


class LinuxSerial:
    def __init__(self, port: str, baud: int = 500000):
        self.port = port
        self.baud = baud
        self.fd = -1

    def open(self):
        self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_NDELAY)
        attr = termios.tcgetattr(self.fd)
        attr[0] = attr[0] & ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK |
                               termios.ISTRIP | termios.INLCR | termios.IGNCR |
                               termios.ICRNL | termios.IXON)
        attr[1] = attr[1] & ~termios.OPOST
        attr[2] = attr[2] & ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)
        attr[2] = attr[2] | termios.CS8 | termios.CREAD | termios.CLOCAL
        attr[3] = attr[3] & ~(termios.ICANON | termios.ECHO | termios.ISIG)
        attr[4] = termios.B460800  # will be overridden
        attr[5] = termios.B460800
        attr[6][termios.VMIN] = 1
        attr[6][termios.VTIME] = 0
        termios.tcflush(self.fd, termios.TCIFLUSH)

        baud_map = {230400: termios.B230400, 460800: termios.B460800,
                     500000: termios.B500000, 921600: termios.B921600}
        baud_const = baud_map.get(self.baud, termios.B460800)
        attr[4] = baud_const
        attr[5] = baud_const
        termios.tcsetattr(self.fd, termios.TCSANOW, attr)

    def close(self):
        if self.fd >= 0:
            os.close(self.fd)
            self.fd = -1

    def write(self, data: bytes):
        os.write(self.fd, data)

    def read_nonblocking(self, max_bytes: int = 4096, wait_s: float = 0.05) -> bytes:
        if wait_s > 0:
            r, _, _ = select.select([self.fd], [], [], wait_s)
            if not r:
                return b''
        try:
            return os.read(self.fd, max_bytes)
        except BlockingIOError:
            return b''
