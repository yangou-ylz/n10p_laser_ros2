#!/usr/bin/env python3
"""PGM→PNG 转换器 — 用法: python3 pgm2png.py <输入.pgm> <输出.png>"""
import sys
from PIL import Image

if len(sys.argv) != 3:
    print("用法: python3 pgm2png.py <输入.pgm> <输出.png>")
    sys.exit(1)

Image.open(sys.argv[1]).save(sys.argv[2])
print(f"✅ {sys.argv[1]} → {sys.argv[2]}")
