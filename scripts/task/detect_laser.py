import cv2
import numpy as np
import time
from picamera2 import Picamera2
import os

# 定义HSV颜色阈值范围
# 绿色激光HSV范围 (H约60度，S和V要求较高)
LOWER_GREEN = np.array([40, 40, 235])   # 下界: H>=40, S>=100, 
UPPER_GREEN = np.array([80, 255, 255])  # 上界: H<=80

# 红色激光HSV范围需要考虑红色环绕Hue轴两端的情况
LOWER_RED1 = np.array([0, 100, 235])     # 红色低Hue区间
UPPER_RED1 = np.array([40, 255, 255])
LOWER_RED2 = np.array([140, 100, 235])   # 红色高Hue区间
UPPER_RED2 = np.array([180, 255, 255])

def _detect_color_spot(frame, M, lower_bounds, upper_bounds, debug=False):
    """
    通用的颜色光斑检测函数。
    根据提供的HSV颜色范围，在图像中检测相应颜色的光斑，并计算其中心位置。
    参数:
        frame (numpy.ndarray): BGR图像帧。
        M (numpy.ndarray): 3x3透视变换矩阵，用于将像素坐标转换到0-100坐标系。
        lower_bounds (list): HSV下界列表（可包含一个或多个区间）。
        upper_bounds (list): HSV上界列表，与lower_bounds对应。
        debug (bool): 若为True，输出调试信息或显示检测可视化。
    返回:
        (x, y) 元组表示光斑中心在0-100坐标系下的坐标。未检测到则返回 None。
    """
    # 转换到HSV色域
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # 构建颜色掩膜（支持多个区间，针对红色）
    mask_total = None
    for lower, upper in zip(lower_bounds, upper_bounds):
        mask = cv2.inRange(hsv, lower, upper)
        if mask_total is None:
            mask_total = mask
        else:
            mask_total = cv2.bitwise_or(mask_total, mask)
    # 可选: 进行形态学操作去除噪点（这里省略，以免滤除小光斑）

    # 查找掩膜中的轮廓
    contours, _ = cv2.findContours(mask_total, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        if debug:
            print("[_detect_color_spot] No contour found for specified color")
        return None

    # 选取最大面积的轮廓来代表激光光斑
    largest = max(contours, key=cv2.contourArea)
    # 计算轮廓的中心(M00为0的情况用点坐标代替)
    M_moments = cv2.moments(largest)
    if M_moments["m00"] == 0:
        cx, cy = largest[0][0][0], largest[0][0][1]
    else:
        cx = M_moments["m10"] / M_moments["m00"]
        cy = M_moments["m01"] / M_moments["m00"]
    # 将中心点从图像像素坐标系映射到0-100坐标系
    pts = np.array([[[cx, cy]]], dtype=np.float32)
    if M is None:
        # 不需要透视变换，直接返回像素坐标
        return (cx, cy)
    norm_pt = cv2.perspectiveTransform(pts, M)
    x_norm = float(norm_pt[0][0][0])
    y_norm = float(norm_pt[0][0][1])

    if debug:
        # 调试模式: 在图像上标记光斑中心并显示
        debug_img = frame.copy()
        cv2.circle(debug_img, (int(cx), int(cy)), 5, (255, 255, 255), 2)  # 用白色圆圈标出检测中心
        cv2.imshow("laser_debug", debug_img)
        cv2.waitKey(1)  # 显示窗口，稍纵即逝; 按需修改暂停时间或按键退出
        # 打印检测到的中心位置
        print(f"[_detect_color_spot] Center (pixel)=({cx:.1f}, {cy:.1f}), (normalized)=({x_norm:.1f}, {y_norm:.1f})")

    return (x_norm, y_norm)

def detect_green(frame, M, debug=False):
    """检测图像中的绿色激光光斑，返回其在0-100坐标系下的中心坐标(tuple)。"""
    return _detect_color_spot(frame, M, [LOWER_GREEN], [UPPER_GREEN], debug=debug)

def detect_red(frame, M, debug=False):
    """检测图像中的红色激光光斑，返回其在0-100坐标系下的中心坐标(tuple)。"""
    return _detect_color_spot(frame, M, [LOWER_RED1, LOWER_RED2], [UPPER_RED1, UPPER_RED2], debug=debug)

# 如果直接运行本模块，简单测试摄像头画面中的红/绿光斑检测
# 如果直接运行本模块，采集 2592x2592 图像并测试红/绿光斑检测
# 如果直接运行本模块，采集 2592x2592 图像并测试红/绿光斑检测
if __name__ == "__main__":
    from picamera2 import Picamera2
    import time, os, cv2, numpy as np

    OUT_IMG = "laser_debug.jpg"

    # ---------- 相机抓帧 ----------
    cam = Picamera2()
    cam.configure(cam.create_still_configuration(
        main={"format": "RGB888", "size": (2592, 2592)}
    ))
    cam.start(); time.sleep(1.0)
    frame_rgb = cam.capture_array(); cam.close()

    frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    vis   = frame.copy()

    # ---------- 透视矩阵 ----------
    M = np.eye(3, dtype=np.float32)   # demo，用真实 M 可输出 0-100 坐标

    # ---------- 光斑检测 ----------
    g_pos = detect_green(frame, M)
    r_pos = detect_red  (frame, M)
    print("green:", g_pos, "red:", r_pos)

    H, W = vis.shape[:2]

    def plot(pt, color, text):
        if pt is None:
            return
        x, y = pt
        # 判定是否为归一化坐标
        if 0 <= x <= 100 and 0 <= y <= 100:
            px = int(x / 100 * (W - 1))
            py = int((100 - y) / 100 * (H - 1))
        else:                       # 直接像素
            px, py = int(x), int(y)
        cv2.circle(vis, (px, py), 20, color, 3)
        cv2.putText(vis, text, (px + 25, py),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3, cv2.LINE_AA)

    plot(g_pos, (0, 255, 0), f"G:{g_pos}")
    plot(r_pos, (0,   0,255), f"R:{r_pos}")

    cv2.imwrite(OUT_IMG, vis)
    print("[INFO] saved →", os.path.abspath(OUT_IMG))


