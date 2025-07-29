#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
综合控制模块 main_control.py
启动后：
1. 检测黑色胶带矩形四角坐标（仅首次执行）。
2. 连续捕获图像，检测绿色激光光斑位置。
3. 利用PID控制算法将激光光斑沿着黑色胶带依次移动到矩形的四个角。
4. 运行指定时间后结束。
"""
import time
import cv2
from simple_pid import PID  # 使用simple_pid库实现PID控制
# 导入我们编写的矩形检测和激光检测模块
import detect_rect
import detect_laser

# 控制参数配置
MAX_RUNTIME = 30.0  # 最大运行时间（秒）
# PID参数（需根据实际系统调试）
KP = 0.5
KI = 0.0
KD = 0.05
TOLERANCE = 1.0  # 目标判定阈值 (在归一化坐标下，误差小于此视为到达目标)

if __name__ == "__main__":
    # 1) 初始化：获取黑色矩形的四个角点坐标
    corners = detect_rect.find_black_rectangle_corners(debug=False)
    if not corners or len(corners) != 4:
        print("未能检测到有效的黑色矩形，程序退出。")
        exit(1)
    # 将角点按照顺时针顺序赋值便于引用
    tl, tr, br, bl = corners  # 左上，右上，右下，左下
    print("黑色矩形角点坐标:", corners)
    # 建议：确保激光光斑初始位于 tl 点处，以便后续沿边运行

    # 2) 准备PID控制器，分别控制X和Y轴方向
    pid_x = PID(KP, KI, KD, setpoint=tl[0])  # 初始目标X设为左上角X
    pid_y = PID(KP, KI, KD, setpoint=tl[1])  # 初始目标Y设为左上角Y
    # 如有需要，可设置PID输出限制，例如伺服转动范围等
    # pid_x.output_limits = (-1.0, 1.0)
    # pid_y.output_limits = (-1.0, 1.0)

    # 3) 启动相机视频模式（持续获取帧）
    cam = detect_rect.Picamera2()
    cam.configure(cam.create_still_configuration())  # 使用静态捕获配置
    cam.start()
    time.sleep(1)  # 稍微等待摄像头稳定
    print("开始激光光斑追踪控制...")

    start_time = time.time()
    # 当前目标索引（依次为0:tl->1:tr->2:br->3:bl->再回到0循环）
    target_index = 1  # 我们假定激光从tl开始，所以下一个目标是tr（索引1）
    current_target = tr  # 当前目标坐标初始化为右上角
    pid_x.setpoint = current_target[0]
    pid_y.setpoint = current_target[1]

    try:
        while True:
            # 获取当前帧并预处理（裁剪为与初始相同的ROI，并缩放480x480）
            frame = cam.capture_array()
            # 裁剪成与初始相同的中心正方形区域
            h, w = frame.shape[:2]
            side_len = min(w, h)
            center_pt = (w // 2, h // 2)
            cropped_frame = detect_rect.crop_image(frame, center_pt, side_len, side_len)
            resized_frame = cv2.resize(cropped_frame, (480, 480))
            # 检测当前帧中绿色激光光斑位置
            green_pos = detect_laser.detect_green_spot(resized_frame)
            if green_pos is None:
                # 如果未检测到光斑，可以跳过本次循环或采取其他措施
                print("[WARN] 当前帧未找到绿色光斑")
                continue
            current_x, current_y = green_pos
            # 计算控制输出
            control_x = pid_x(current_x)
            control_y = pid_y(current_y)
            # 将控制输出应用于执行机构（例如电机转动）。这里用伪代码表示：
            # motor_x.move(control_x)
            # motor_y.move(control_y)
            # 或根据具体硬件API调用控制。例如:
            # pan_servo.angle += control_x
            # tilt_servo.angle += control_y

            # 检查是否接近当前目标点
            error_x = abs(current_target[0] - current_x)
            error_y = abs(current_target[1] - current_y)
            if error_x < TOLERANCE and error_y < TOLERANCE:
                # 达到目标附近，切换到下一个目标
                target_index = (target_index + 1) % 4
                if target_index == 0:
                    next_target = tl
                elif target_index == 1:
                    next_target = tr
                elif target_index == 2:
                    next_target = br
                else:
                    next_target = bl
                # 更新PID目标值
                # 如果矩形边平行于轴，可固定一轴防止偏离路径
                if current_target[1] == next_target[1]:
                    # 水平移动，保持Y不变
                    pid_y.setpoint = current_target[1]
                    pid_x.setpoint = next_target[0]
                elif current_target[0] == next_target[0]:
                    # 垂直移动，保持X不变
                    pid_x.setpoint = current_target[0]
                    pid_y.setpoint = next_target[1]
                else:
                    # 对角情况（一般不存在于矩形），直接设置目标
                    pid_x.setpoint = next_target[0]
                    pid_y.setpoint = next_target[1]
                current_target = next_target
                print(f"切换下一个目标点: {current_target}")
            # 退出条件检查
            if time.time() - start_time > MAX_RUNTIME:
                print(f"运行时间已达{MAX_RUNTIME}秒，停止控制。")
                break
    finally:
        # 确保程序结束时关闭摄像头
        cam.close()
