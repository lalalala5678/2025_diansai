#!/usr/bin/env python3
"""
拍照并按给定中心点和宽高进行裁剪（单独封装裁剪函数，裁剪函数不保存文件）
"""
from typing import Tuple
import time
import cv2
from picamera2 import Picamera2
import numpy as np


def crop_image(img: np.ndarray, center: Tuple[int, int], width: int, height: int) -> np.ndarray:
    """
    在给定图像上按 (center, width, height) 裁剪并返回 ROI。
    * center: (x, y) 中心点像素坐标
    * width, height: 裁剪框的宽、高（像素）
    """
    x, y = center
    h, w = img.shape[:2]

    # 半宽/半高
    w2, h2 = width // 2, height // 2

    # 计算不越界的四个顶点
    x1, y1 = max(0, x - w2), max(0, y - h2)
    x2, y2 = min(w, x + w2), min(h, y + h2)

    # 返回裁剪后的图像（新数组，不共享内存）
    return img[y1:y2, x1:x2].copy()


def capture_image() -> np.ndarray:
    """使用 Picamera2 拍一张照片并返回 ndarray"""
    cam = Picamera2()
    cam.configure(cam.create_still_configuration())
    cam.start()
    time.sleep(2)                     # 预热
    frame = cam.capture_array()
    cam.close()
    return frame


if __name__ == "__main__":
    # 1) 拍摄原始大图
    original = capture_image()
    cv2.imwrite("original.jpg", original)  # 如需调试保留

    # 2) 计算中心点 & 目标大小（此处示例：取最短边，做正方形裁剪）
    h, w = original.shape[:2]
    center_pt = (w // 2, h // 2)
    side_len  = min(w, h)

    # 3) 调用裁剪函数
    cropped = crop_image(original, center_pt, side_len, side_len)
    resize_cropped = cv2.resize(cropped, (480, 480))  # 可选：调整大小
    # 4) 需要时再保存或进一步处理
    cv2.imwrite("cropped.jpg", resize_cropped)
