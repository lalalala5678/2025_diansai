import cv2
import logging
import time
import math
# 导入自定义模块
import detectrect
import detect_laser
import xy_to_plus  # 假定已有舵机控制封装模块
import numpy as np
from picamera2 import Picamera2     # 放在文件顶部 import 区


# ==== 可配置参数 ====
FRAME_RATE = 15          # 帧率（FPS）
PID_KP = 0.5             # PID 比例系数
PID_KI = 0.1             # PID 积分系数
PID_KD = 0.05            # PID 微分系数
CORNER_THRESHOLD = 2.0   # 判定到达角点的距离阈值 (0-100坐标系下)

# 初始化日志系统（打印消息）
logging.basicConfig(level=logging.INFO, format='%(message)s')

def main():
    # # 打开摄像头
    # cap = cv2.VideoCapture(0)

    # if not cap.isOpened():
    #     logging.error("Failed to open camera.")
    #     return



    # ---------- 打开树莓派相机 ----------
    try:
        cam = Picamera2()
        cam.configure(
            cam.create_preview_configuration(          # 预览流，速度快
                main={"format": "RGB888", "size": (640, 480)}
            )
        )
        cam.start()
        time.sleep(0.4)                               # 预热
    except Exception as e:
        logging.error(f"Failed to open Picamera2: {e}")
        return

    # 封装一个函数，后面循环直接调用 read_frame() 代替 cap.read()
    def read_frame() -> tuple[bool, cv2.Mat]:
        """读取一帧并转换为 BGR，接口与 cap.read() 类似"""
        try:
            rgb = cam.capture_array()                 # ndarray, RGB
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            return True, bgr
        except Exception:
            return False, None

    # --------- 首帧用于矩形标定 ---------
    ret, frame = read_frame()
    if not ret:
        logging.error("Failed to capture initial frame for rectangle detection.")
        cam.close()
        return

    

 

    # 检测黑色胶带矩形
    corners, M = detectrect.detect_rectangle(frame)
    if corners is None or M is None:
        logging.error("Black rectangle not found. Exiting.")
        cap.release()
        return

    logging.info(f"Rectangle corners (normalized): {corners.tolist()}")

    # 确定循迹的路径点顺序（按检测到的角点顺时针排序）
    path_points = corners.tolist()  # 转为普通列表便于处理
    # 初始目标点设为第一个角点的坐标
    target_index = 0
    target_x, target_y = float(path_points[target_index][0]), float(path_points[target_index][1])

    # 将绿色激光点初始定位到第一个角点上（发出初始舵机控制命令）
    servo_x_cmd = target_x
    servo_y_cmd = target_y

    xy_to_plus.set_xy(servo_x_cmd, servo_y_cmd,1000)  # 控制舵机指向初始角点

    time.sleep(1)  # 短暂等待舵机到位

    # PID控制相关变量初始化
    prev_err_x = None
    prev_err_y = None
    integral_x = 0.0
    integral_y = 0.0

    start_time = time.time()
    # 主循环，运行30秒循迹
    while time.time() - start_time < 30:
        ret, frame = read_frame()

        if not ret:
            logging.warning("Failed to read frame from camera.")
            break

        # 检测绿色激光光斑位置
        pos = detect_laser.detect_green(frame, M)
        if pos is None:
            # 未检测到绿色光斑，本帧跳过控制
            logging.warning("Green laser spot not found in frame.")
            # 不更新PID误差，直接进入下一帧
            continue

        current_x, current_y = pos  # 当前激光点的位置 (0-100坐标系)
        # 计算当前误差 (目标点坐标 - 当前点坐标)
        err_x = target_x - current_x
        err_y = target_y - current_y

        # 判断是否到达当前目标点附近，如是则切换到下一个目标
        dist_to_target = math.hypot(err_x, err_y)
        if dist_to_target < CORNER_THRESHOLD:
            # 切换到下一路径点（顺时针循迹）
            target_index = (target_index + 1) % len(path_points)
            target_x, target_y = float(path_points[target_index][0]), float(path_points[target_index][1])
            # 重新计算误差（切换目标后立即更新）
            err_x = target_x - current_x
            err_y = target_y - current_y
            # 重置PID积分和前误差，避免切换目标时的积分累积/导数剧烈变化
            prev_err_x = None
            prev_err_y = None
            integral_x = 0.0
            integral_y = 0.0
            logging.info(f"Switched to next target corner: ({target_x:.1f}, {target_y:.1f})")

        # 计算时间步长 dt（用于PID积分和微分）
        current_time = time.time()
        dt = current_time - start_time if prev_err_x is None else current_time - last_time
        # 防止dt过大导致的积算误差
        if dt <= 0:
            dt = 1.0 / FRAME_RATE

        # PID积分项更新（仅在检测到光斑时累积）
        integral_x += err_x * dt
        integral_y += err_y * dt

        # PID微分项计算
        derivative_x = 0.0
        derivative_y = 0.0
        if prev_err_x is not None and prev_err_y is not None:
            derivative_x = (err_x - prev_err_x) / dt
            derivative_y = (err_y - prev_err_y) / dt

        # PID控制输出（位置增量）
        output_x = PID_KP * err_x + PID_KI * integral_x + PID_KD * derivative_x
        output_y = PID_KP * err_y + PID_KI * integral_y + PID_KD * derivative_y

        # 更新舵机指令位置（加上增量，并限制在0-100范围）
        servo_x_cmd += output_x
        servo_y_cmd += output_y
        if servo_x_cmd < 0: servo_x_cmd = 0.0
        if servo_x_cmd > 100: servo_x_cmd = 100.0
        if servo_y_cmd < 0: servo_y_cmd = 0.0
        if servo_y_cmd > 100: servo_y_cmd = 100.0

        # 发出舵机控制命令
        # 发出舵机控制命令（自适应时间）
        dist  = math.hypot(output_x, output_y)          # 0-100
        move_ms = max(30, min(60, 4.5 * dist))          # 公式
        xy_to_plus.set_xy(servo_x_cmd, servo_y_cmd, int(move_ms))


        # 通过日志输出当前帧的控制数据
        logging.info(f"Err: ({err_x:.1f}, {err_y:.1f}), Target: ({target_x:.1f}, {target_y:.1f}), "
                     f"Pos: ({current_x:.1f}, {current_y:.1f}), PWM: ({servo_x_cmd:.1f}, {servo_y_cmd:.1f})")

        # 保存本次误差和时间用于下次计算
        prev_err_x = err_x
        prev_err_y = err_y
        last_time = current_time

        # 按帧率等待下一个循环（精确睡眠）
        loop_duration = time.time() - current_time
        sleep_time = (1.0 / FRAME_RATE) - loop_duration
        if sleep_time > 0:
            time.sleep(sleep_time)

    # 循环结束，释放资源

    cam.close()

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
