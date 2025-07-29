import cv2
import logging
import time
# 导入自定义模块
import detectrect
import detect_laser
import xy_to_plus

# ==== 可配置参数 ====
FRAME_RATE = 15         # 帧率（FPS）
PID_KP = 0.5            # PID 比例系数
PID_KI = 0.1            # PID 积分系数
PID_KD = 0.05           # PID 微分系数

logging.basicConfig(level=logging.INFO, format='%(message)s')

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        logging.error("Failed to open camera.")
        return

    # 获取透视变换矩阵M（通过检测黑色矩形；若已有M可省略此步骤）
    ret, frame = cap.read()
    if not ret:
        logging.error("Failed to capture frame for calibration.")
        cap.release()
        return
    corners, M = detectrect.detect_rectangle(frame)
    if M is None:
        logging.warning("Calibration rectangle not found. Using identity transform.")
        M = np.eye(3, dtype=np.float32)  # 若未检测到矩形，用单位矩阵作为近似映射（可能影响精度）

    # 检测初始位置
    red_pos = None
    green_pos = None
    # 尝试获取初始红、绿光斑位置（等待两者都检测到）
    for _ in range(10):
        ret, frame = cap.read()
        if not ret:
            continue
        # 获取红色和绿色光斑的位置
        g = detect_laser.detect_green(frame, M)
        r = detect_laser.detect_red(frame, M)
        if g is not None:
            green_pos = g
        if r is not None:
            red_pos = r
        if green_pos is not None and red_pos is not None:
            break

    if green_pos is None or red_pos is None:
        logging.error("Initial laser spot positions not found (Green or Red missing).")
        cap.release()
        return

    # 初始当前位置设定
    current_x, current_y = green_pos  # 绿激光初始坐标
    target_x, target_y = red_pos     # 红激光初始坐标（目标）
    # 将舵机当前位置设为绿光初始位置，确保PID初始误差正确
    servo_x_cmd = current_x
    servo_y_cmd = current_y
    # 不立即移动舵机，让其保持当前指向（因为绿光已经在那里）

    # PID控制变量初始化
    prev_err_x = None
    prev_err_y = None
    integral_x = 0.0
    integral_y = 0.0

    start_time = time.time()
    while time.time() - start_time < 30:
        ret, frame = cap.read()
        if not ret:
            logging.warning("Failed to read frame from camera.")
            break

        # 检测当前红色和绿色激光位置
        pos_green = detect_laser.detect_green(frame, M)
        pos_red = detect_laser.detect_red(frame, M)
        if pos_green is None or pos_red is None:
            # 如有任一未检测到，跳过本帧（保持上一控制输出）
            if pos_green is None:
                logging.warning("Green laser spot lost.")
            if pos_red is None:
                logging.warning("Red laser spot lost.")
            continue

        current_x, current_y = pos_green  # 绿色光斑当前坐标
        target_x, target_y = pos_red      # 红色光斑当前坐标（作为目标）
        # 计算位置误差（让绿色光点追踪红色光点）
        err_x = target_x - current_x
        err_y = target_y - current_y

        # 计算时间间隔 dt
        current_time = time.time()
        dt = current_time - start_time if prev_err_x is None else current_time - last_time
        if dt <= 0:
            dt = 1.0 / FRAME_RATE

        # 更新PID积分
        integral_x += err_x * dt
        integral_y += err_y * dt
        # 计算PID微分项
        derivative_x = 0.0
        derivative_y = 0.0
        if prev_err_x is not None and prev_err_y is not None:
            derivative_x = (err_x - prev_err_x) / dt
            derivative_y = (err_y - prev_err_y) / dt

        # PID输出计算
        output_x = PID_KP * err_x + PID_KI * integral_x + PID_KD * derivative_x
        output_y = PID_KP * err_y + PID_KI * integral_y + PID_KD * derivative_y

        # 更新舵机控制指令，并限制在0-100范围
        servo_x_cmd += output_x
        servo_y_cmd += output_y
        if servo_x_cmd < 0: servo_x_cmd = 0.0
        if servo_x_cmd > 100: servo_x_cmd = 100.0
        if servo_y_cmd < 0: servo_y_cmd = 0.0
        if servo_y_cmd > 100: servo_y_cmd = 100.0

        # 发送舵机控制信号
        # 根据本帧增量自适应估算舵机运动时间
        step     = math.hypot(output_x, output_y)          # Δd  (0-100 范围)
        move_ms  = int(max(30, min(60, 4.5 * step)))       # 30 ms ≤ t ≤ 60 ms
        xy_to_plus.set_xy(servo_x_cmd, servo_y_cmd, move_ms)


        # 输出当前状态日志
        logging.info(f"Err: ({err_x:.1f}, {err_y:.1f}), Target: ({target_x:.1f}, {target_y:.1f}), "
                     f"Pos: ({current_x:.1f}, {current_y:.1f}), PWM: ({servo_x_cmd:.1f}, {servo_y_cmd:.1f})")

        # 更新前一循环误差和时间
        prev_err_x = err_x
        prev_err_y = err_y
        last_time = current_time

        # 按设定帧率等待
        loop_duration = time.time() - current_time
        sleep_time = (1.0 / FRAME_RATE) - loop_duration
        if sleep_time > 0:
            time.sleep(sleep_time)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
