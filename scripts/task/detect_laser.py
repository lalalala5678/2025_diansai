#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
激光光斑检测模块 detect_laser.py
提供函数用于检测图像中的红色或绿色激光光斑，返回其归一化坐标。
要求输入图像应为已裁剪对准的正方形区域图（如480x480），以匹配坐标系。
"""
import cv2
import numpy as np
from typing import Tuple, Optional

def detect_red_spot(img: np.ndarray) -> Optional[Tuple[float, float]]:
    """
    检测图像中的红色激光光斑，返回其 (X, Y) 归一化坐标。
    如未检测到红色光斑，则返回 None。
    """
    # 将 BGR 图像转换到 HSV 色彩空间
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # 定义红色的HSV阈值范围（包含低H和高H两段）
    lower_red1 = np.array([0, 100, 100])    # 红色低端 (接近0度)
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 100, 100])  # 红色高端 (接近360度，在OpenCV中约179)
    upper_red2 = np.array([180, 255, 255])
    # 生成红色掩膜
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    # 可选：进行一次形态学操作去噪（如开操作去除小点）
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    # 查找红色光斑的轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    # 找到面积最大的候选轮廓
    largest = max(contours, key=cv2.contourArea)
    # 计算该轮廓的中心
    M = cv2.moments(largest)
    if M["m00"] == 0:  # 避免除数为零
        return None
    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]
    # 将中心坐标转换为归一化的百分比坐标 (0~100)
    h, w = img.shape[:2]
    norm_x = (cx / (w - 1)) * 100.0
    norm_y = ((h - 1 - cy) / (h - 1)) * 100.0
    return (round(norm_x, 4), round(norm_y, 4))

def detect_green_spot(img: np.ndarray) -> Optional[Tuple[float, float]]:
    """
    检测图像中的绿色激光光斑，返回其 (X, Y) 归一化坐标。
    如未检测到绿色光斑，则返回 None。
    """
    # 转换为 HSV 色彩空间
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # 定义绿色的HSV范围
    lower_green = np.array([35, 100, 100])
    upper_green = np.array([85, 255, 255])
    mask = cv2.inRange(hsv, lower_green, upper_green)
    # 形态学开操作，去除噪点
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    # 查找绿色光斑轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    M = cv2.moments(largest)
    if M["m00"] == 0:
        return None
    cx = M["m10"] / M["m00"]
    cy = M["m01"] / M["m00"]
    # 转换为归一化坐标
    h, w = img.shape[:2]
    norm_x = (cx / (w - 1)) * 100.0
    norm_y = ((h - 1 - cy) / (h - 1)) * 100.0
    return (round(norm_x, 4), round(norm_y, 4))

# 模块测试
if __name__ == "__main__":
    # 从当前目录读取测试图片 (cropped.jpg)
    test_img = cv2.imread("cropped.jpg")
    if test_img is None:
        print("未找到 cropped.jpg 测试文件！")
    else:
        red_coord = detect_red_spot(test_img)
        green_coord = detect_green_spot(test_img)
        print("红色光斑坐标:", red_coord)
        print("绿色光斑坐标:", green_coord)
