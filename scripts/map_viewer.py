#!/usr/bin/env python3
"""
PGM 地图查看器 — 可视化 occupancy grid 地图
用法:
    python3 map_viewer.py <地图.yaml>              # 交互显示
    python3 map_viewer.py <地图.yaml> --save map.png  # 保存为 PNG
    python3 map_viewer.py <地图.yaml> --no-show        # 只打印信息不显示
    python3 map_viewer.py <地图.yaml> --title "我的地图" # 自定义标题
"""

import argparse
import sys
import yaml
import numpy as np
import matplotlib
matplotlib.use('TkAgg')  # 避免后端冲突
import matplotlib.pyplot as plt
from pathlib import Path
from PIL import Image


def load_map(yaml_path):
    """加载 .yaml + .pgm, 返回 (meta, numpy_data)"""
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)

    # 查找 pgm 文件（兼容相对/绝对路径）
    pgm_path = Path(yaml_path).parent / meta['image']
    if not pgm_path.exists():
        pgm_path = Path(yaml_path).parent / Path(meta['image']).name

    img = Image.open(str(pgm_path))
    data = np.array(img).astype(np.float32)

    # 规范化: 标准值 254=自由, 0=占用, 205=未知
    occupied = data < 50
    free = data > 240
    unknown = ~occupied & ~free

    result = np.zeros_like(data)
    result[free] = 0      # 自由 → 白色
    result[occupied] = 100 # 占用 → 黑色
    result[unknown] = 50   # 未知 → 灰色

    return meta, result


def print_info(meta, data):
    """打印地图基本信息"""
    res = meta['resolution']
    h, w = data.shape
    free = (data == 0).sum()
    occupied = (data == 100).sum()
    unknown = (data == 50).sum()
    total = free + occupied + unknown

    print(f'分辨率:   {res} m/像素')
    print(f'像素尺寸: {w} × {h}')
    print(f'实际覆盖: {w * res:.2f}m × {h * res:.2f}m')
    print(f'原点:     ({meta["origin"][0]:.2f}, {meta["origin"][1]:.2f})')
    print(f'可通行:   {free} px ({free / total * 100:.1f}%)')
    print(f'障碍物:   {occupied} px ({occupied / total * 100:.1f}%)')
    print(f'未知:     {unknown} px ({unknown / total * 100:.1f}%)')


def plot_map(meta, data, title=None, save_path=None, show=True):
    """显示/保存地图"""
    res = meta['resolution']
    origin = meta['origin']
    h, w = data.shape
    width_m = w * res
    height_m = h * res

    fig, ax = plt.subplots(figsize=(10, 8))

    # 用灰度 + 阈值映射 (避免 BoundaryNorm 兼容问题)
    ax.imshow(data, cmap='gray', vmin=0, vmax=100, origin='upper',
              extent=[origin[0], origin[0] + width_m,
                      origin[1], origin[1] + height_m])

    # 原点标记
    ax.plot(0, 0, 'r+', markersize=14, markeredgewidth=2, label='origin (0,0)')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(title or f'PGM Map: {w}×{h} px @ {res}m/px')
    ax.legend()
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f'已保存: {save_path}')

    if show:
        plt.show()
    else:
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description='PGM 地图查看器')
    parser.add_argument('yaml', nargs='?', help='地图 .yaml 文件路径')
    parser.add_argument('--save', '-s', help='保存为 PNG 图片')
    parser.add_argument('--no-show', action='store_true', help='不显示 GUI 窗口')
    parser.add_argument('--title', '-t', default=None, help='自定义标题')
    args = parser.parse_args()

    yaml_path = args.yaml or input('请输入地图 .yaml 路径: ').strip()

    meta, data = load_map(yaml_path)
    print_info(meta, data)
    plot_map(meta, data, title=args.title, save_path=args.save, show=not args.no_show)


if __name__ == '__main__':
    main()
