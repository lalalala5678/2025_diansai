#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
黑色矩形检测模块 detect_rect.py
提供函数用于捕获图像、裁剪ROI、检测由黑色胶带构成的矩形，并返回其四个角点的归一化坐标。
"""
from typing import Tuple, List
import time
import cv2
import numpy as np
from picamera2 import Picamera2

def capture_image() -> np.ndarray:
    """
    使用 Picamera2 拍摄一张照片并返回图像数据（ndarray）。
    """
    cam = Picamera2()
    cam.configure(cam.create_still_configuration())
    cam.start()
    time.sleep(2)  # 摄像头预热
    frame = cam.capture_array()
    cam.close()
    return frame

def crop_image(img: np.ndarray, center: Tuple[int, int], width: int, height: int) -> np.ndarray:
    """
    从给定图像 `img` 中按指定中心点和宽高裁剪出 ROI 子图像并返回。
    参数:
        img: 原始图像数组。
        center: 裁剪区域中心点 (x, y) 像素坐标。
        width, height: 裁剪区域的宽度和高度（像素）。
    返回:
        裁剪后的图像子区域（为原图的拷贝）。
    """
    x, y = center
    h, w = img.shape[:2]
    # 计算半宽和半高
    half_w, half_h = width // 2, height // 2
    # 确定裁剪区域的边界，确保不越界
    x1, y1 = max(0, x - half_w), max(0, y - half_h)
    x2, y2 = min(w, x + half_w), min(h, y + half_h)
    # 裁剪ROI区域并返回副本
    return img[y1:y2, x1:x2].copy()

def detect_rectangle_corners(image: np.ndarray) -> List[Tuple[float, float]]:
    """
    对输入图像进行处理，检测最大黑色矩形轮廓，返回其4个角点的归一化坐标列表。
    要求图像应为正方形（如480x480）以简化计算。
    """
    # 转灰度并模糊，降低噪声影响
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)
    # Otsu阈值分割，获取黑色区域（使用INV将黑色变为白色）
    _, thresh = cv2.threshold(gray_blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    # 形态学闭操作，填充胶带矩形内部的空隙，平滑边缘
    kernel = np.ones((3, 3), np.uint8)  # 使用3x3核，避免过多膨胀
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=1)
    # 查找外部轮廓
    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return []  # 未找到任何轮廓，返回空列表
    # 选择最大面积的轮廓作为黑色胶带矩形
    contour = max(contours, key=cv2.contourArea)
    # 多边形近似，逼近为四边形
    peri = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
    if len(approx) == 4:
        corners_pts = [tuple(pt[0]) for pt in approx]  # 提取近似的四个角点坐标
    else:
        # 若近似结果不是四边形，用最小外接矩形获取角点
        rot_rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rot_rect)
        corners_pts = [tuple(pt) for pt in box]
    # 对角点进行排序（顺时针排序：左上、右上、右下、左下）
    def sort_corners_clockwise(pts: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        pts = sorted(pts, key=lambda p: p[0] + p[1])  # 先按 x+y 升序排序，tl 最小，br 最大
        tl = pts[0]
        br = pts[-1]
        # 剩余两个点分别为tr和bl，按照x大小区分
        other = pts[1:-1]
        tr, bl = (other[0], other[1]) if other[0][0] > other[1][0] else (other[1], other[0])
        return [tl, tr, br, bl]
    corners_pts = sort_corners_clockwise(corners_pts)
    # 归一化角点坐标到 (0, 0) - (100, 100) 范围
    height, width = image.shape[:2]
    result_points: List[Tuple[float, float]] = []
    for (px, py) in corners_pts:
        norm_x = (px / (width - 1)) * 100.0
        norm_y = ((height - 1 - py) / (height - 1)) * 100.0
        result_points.append((round(norm_x, 4), round(norm_y, 4)))
    return result_points

def find_black_rectangle_corners(debug: bool = False, debug_path: str = "debug_rect.jpg") -> List[Tuple[float, float]]:
    """
    拍摄图像并检测黑色矩形的主函数。返回矩形四角的归一化坐标列表。
    当 debug=True 时，会保存带有标记的检测结果图以供调试。
    """
    # 1) 拍摄原始图像
    original_img = capture_image()
    # 2) 确定中心点和裁剪尺寸（取图像短边长度作为正方形边长）
    h, w = original_img.shape[:2]
    side_len = min(w, h)
    center_pt = (w // 2, h // 2)
    # 3) 裁剪得到中心的正方形区域，并缩放到480x480大小
    cropped = crop_image(original_img, center_pt, side_len, side_len)
    resized_img = cv2.resize(cropped, (480, 480))
    # 4) 检测黑色矩形的四个角点
    corners = detect_rectangle_corners(resized_img)
    # 如果需要调试，绘制角点和连线并保存标记后的图像
    if debug and corners:
        debug_img = resized_img.copy()
        # 将浮点归一化坐标转换回图像像素坐标用于标记
        pts_int = np.array([[int(p[0] / 100.0 * (480 - 1)), int((100.0 - p[1]) / 100.0 * (480 - 1))] for p in corners])
        # 绘制多边形
        cv2.polylines(debug_img, [pts_int.reshape(-1, 1, 2)], True, (0, 255, 0), 2)
        # 标记每个角点
        font = cv2.FONT_HERSHEY_SIMPLEX
        for idx, (px, py) in enumerate(pts_int):
            cv2.circle(debug_img, (px, py), 5, (0, 0, 255), -1)
            norm_x, norm_y = corners[idx]
            cv2.putText(debug_img, f"{idx}:({norm_x:.1f},{norm_y:.1f})", (px + 5, py + 15),
                        font, 0.5, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.imwrite(debug_path, debug_img)
        print(f"[DEBUG] 矩形检测结果已保存: {debug_path}")
    return corners

# 模块测试
if __name__ == "__main__":
    coords = find_black_rectangle_corners(debug=True)
    print("检测到的矩形四角坐标:", coords)
