#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
capture_and_analyze_v.py
------------------------
使用树莓派 Picamera2 拍一张照片 → 裁剪成正方形 → 提取 HSV 的 V 通道并保存：
  • v_channel_gray.png   —— 纯灰度亮度图
  • v_channel_color.png  —— 伪彩色 + 颜色条
  • v_values.csv         —— （可选）像素 V 值表
可选同时保留原图 original.jpg 与裁剪图 cropped.jpg 以便调试。
"""

from typing import Tuple
import time
import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")              # 关掉 GUI
import matplotlib.pyplot as plt
from picamera2 import Picamera2
from pathlib import Path
import argparse
import sys

# ---------- 工具函数 ----------
def crop_image(img: np.ndarray, center: Tuple[int, int], width: int, height: int) -> np.ndarray:
    """在 img 上按 (center, width, height) 裁剪并返回 ROI（新数组，不共享内存）"""
    x, y = center
    h, w = img.shape[:2]
    w2, h2 = width // 2, height // 2
    x1, y1 = max(0, x - w2), max(0, y - h2)
    x2, y2 = min(w, x + w2), min(h, y + h2)
    return img[y1:y2, x1:x2].copy()

def capture_image(delay: float = 2.0) -> np.ndarray:
    """使用 Picamera2 拍照并返回 ndarray (BGR)"""
    cam = Picamera2()
    cam.configure(cam.create_still_configuration())
    cam.start()
    time.sleep(delay)              # 让传感器和 AE 稳定
    frame = cam.capture_array()    # 默认 RGB888
    cam.close()
    return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

# ---------- 主逻辑 ----------
def main():
    ap = argparse.ArgumentParser(description="Capture image, analyze V channel, save outputs.")
    ap.add_argument("-o", "--outdir", default=".",
                    help="输出目录（默认当前目录）")
    ap.add_argument("--csv", action="store_true",
                    help="同时保存 v_values.csv")
    ap.add_argument("--keep-src", action="store_true",
                    help="保留 original.jpg / cropped.jpg 以便调试")
    ap.add_argument("--resize", type=int, default=480,
                    help="裁剪后缩放到的边长像素，0=不缩放（默认 480）")

    ap.add_argument("--v-th", type=int, default=240,
                    help="阈值 (0–255)。V 值小于此值的像素会被置黑并另存图片")

    args = ap.parse_args()

    out_dir = Path(args.outdir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # 1) 拍照
    print("📷 Capturing image...")
    original = capture_image()

    # 2) 裁剪成正方形（取最短边）
    h, w = original.shape[:2]
    side = min(h, w)
    cropped = crop_image(original, (w // 2, h // 2), side, side)

    # 3) 可选缩放
    if args.resize > 0:
        cropped = cv2.resize(cropped, (args.resize, args.resize))

    # 4) 保存调试图（可选）
    if args.keep_src:
        cv2.imwrite(str(out_dir / "original.jpg"), original)
        cv2.imwrite(str(out_dir / "cropped.jpg"), cropped)

    # 5) HSV ➜ V
    v = cv2.cvtColor(cropped, cv2.COLOR_BGR2HSV)[:, :, 2]  # uint8 (0–255)

    # 6) 保存灰度 PNG（V 值即亮度）
    gray_png = out_dir / "v_channel_gray.png"
    cv2.imwrite(str(gray_png), v)

    # 7) 保存伪彩色 PNG（含颜色条）
    plt.figure(figsize=(6, 6))
    plt.imshow(v, cmap="viridis", vmin=0, vmax=255)
    plt.axis("off")
    cbar = plt.colorbar()
    cbar.set_label("V value (0–255)")
    plt.tight_layout()
    color_png = out_dir / "v_channel_color.png"
    plt.savefig(color_png, dpi=300)
    plt.close()

    # 7.5) 生成“低亮度置黑”图像
    v_th = args.v_th
    # 创建原图副本并在 BGR 空间掩蔽
    masked_bgr = cropped.copy()
    masked_bgr[v < v_th] = (0, 0, 0)          # BGR 置黑
    th_png = out_dir / f"v_mask_below_{v_th}.png"
    cv2.imwrite(str(th_png), masked_bgr)



    # 8) 可选 CSV
    extra = ""
    if args.csv:
        csv_path = out_dir / "v_values.csv"
        np.savetxt(csv_path, v, fmt="%d", delimiter=",")
        extra = f"、{csv_path.name}"

    print(f"✅ 已保存 {gray_png.name}、{color_png.name}{extra} 到 {out_dir}")
    print(f"✅ 已保存 {gray_png.name}、{color_png.name}、{th_png.name}{extra} 到 {out_dir}")


if __name__ == "__main__":
    main()
