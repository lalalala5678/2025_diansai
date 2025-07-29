import cv2
import logging
import time
import math
import numpy as np
from picamera2 import Picamera2
# 导入自定义模块
import detectrect
import detect_laser
import xy_to_plus

# ==== 可配置参数 ====
FRAME_RATE = 15           # 帧率（FPS）
PID_KP = 0.5              # PID 比例系数
PID_KI = 0.1              # PID 积分系数
PID_KD = 0.05             # PID 微分系数
CORNER_THRESHOLD = 2.0    # 判定到达角点的距离阈值 (0-100 坐标系)
PREVIEW_RES = (640, 480)  # 连续跟踪使用的分辨率（较低分辨率提高速度）
CALIBRATION_RES = (2592, 2592)  # 标定用高分辨率照片

# 卡尔曼滤波参数
KF_PROCESS_VAR = 50.0     # 过程噪声方差（加速度不确定性）
KF_MEASURE_VAR = 4.0      # 测量噪声方差

logging.basicConfig(level=logging.INFO, format='%(message)s')

# 定义一个简易的 PID 控制器类以提高代码清晰度
class PIDController:
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = None
        self.integral = 0.0

    def reset(self):
        """重置 PID 控制器状态。"""
        self.prev_error = None
        self.integral = 0.0

    def compute(self, error: float, dt: float) -> float:
        """计算 PID 输出。"""
        # 累计积分误差
        self.integral += error * dt
        # 计算微分项
        derivative = 0.0
        if self.prev_error is not None:
            derivative = (error - self.prev_error) / dt
        # 计算 PID 输出
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        # 保存本次误差用于下次计算微分
        self.prev_error = error
        return output

# 定义一维位置（含速度）卡尔曼滤波器类
class KalmanFilter1D:
    def __init__(self, initial_pos: float, initial_vel: float = 0.0,
                 process_var: float = 1.0, meas_var: float = 1.0):
        # 状态向量：[位置; 速度]
        self.x = np.array([[initial_pos], [initial_vel]], dtype=float)
        # 状态协方差矩阵
        self.P = np.eye(2, dtype=float) * 1e-3  # 初始不确定性设为很小
        # 观测矩阵（直接测量位置）
        self.H = np.array([[1.0, 0.0]], dtype=float)
        # 测量噪声协方差 R
        self.R = np.array([[meas_var]], dtype=float)
        # 过程噪声强度（加速度方差）
        self.process_var = process_var

    def predict(self, dt: float):
        # 常速模型的状态转移矩阵
        F = np.array([[1.0, dt], [0.0, 1.0]], dtype=float)
        # 常加速模型的过程噪声协方差矩阵 Q
        Q = np.array([[ (dt**4)/4.0, (dt**3)/2.0 ],
                      [ (dt**3)/2.0, (dt**2)      ]], dtype=float) * self.process_var
        # 预测状态和协方差
        self.x = F.dot(self.x)
        self.P = F.dot(self.P).dot(F.T) + Q

    def update(self, measured_pos: float):
        # 测量更新（卡尔曼增益和校正）
        z = np.array([[measured_pos]], dtype=float)  # 测量位置
        # 创新（残差）y = z - Hx
        y = z - self.H.dot(self.x)
        # 创新协方差 S = H P H^T + R
        S = self.H.dot(self.P).dot(self.H.T) + self.R
        # 卡尔曼增益 K = P H^T S^-1
        K = self.P.dot(self.H.T).dot(np.linalg.inv(S))
        # 状态更新 x = x + K y
        self.x = self.x + K.dot(y)
        # 协方差更新 P = (I - K H) P
        I = np.eye(2, dtype=float)
        self.P = (I - K.dot(self.H)).dot(self.P)
        # （K 和 y 不需要保存）

def main():
    # ----- 使用高分辨率拍摄图像进行标定 -----
    try:
        cam_calib = Picamera2()
        # 配置相机为高分辨率静态拍摄（以提高矩形检测的精度）
        cam_calib.configure(cam_calib.create_still_configuration(
            main={"format": "RGB888", "size": CALIBRATION_RES}
        ))
        cam_calib.start()
        time.sleep(0.5)  # 预热
        frame_rgb = cam_calib.capture_array()
        cam_calib.close()
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    except Exception as e:
        logging.error(f"Failed to capture calibration image: {e}")
        return

    # 检测黑色胶带矩形进行标定
    # detect_rectangle 预计返回归一化角点坐标和透视变换矩阵 M
    corners, M = detectrect.detect_rectangle(frame_bgr)
    if corners is None or M is None:
        logging.error("Black rectangle not found during calibration. Exiting.")
        return
    logging.info(f"Rectangle corners (normalized): {corners.tolist()}")


    # 可选：将标定结果绘制在图像上并保存
    # 这里需要将标定后的数据绘制在图片上，储存进行调试使用
    cv2.polylines(frame_bgr, [corners.astype(int)], True, (0, 255, 0), 2) # 绘制矩形边框
    for (x, y) in corners.astype(int):
        cv2.circle(frame_bgr, (x, y), 6, (0, 0, 255), -1)
        # 标注坐标
        cv2.putText(frame_bgr, f"({x},{y})", (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.imwrite("calibration_result.jpg", frame_bgr)



    # 确定循迹路径的角点顺序（假设 detect_rectangle 返回的角点按顺时针顺序）
    path_points = corners.tolist()
    target_index = 0
    target_x = float(path_points[target_index][0])
    target_y = float(path_points[target_index][1])

    # 初始化舵机指向第一个目标角点
    servo_x_cmd = target_x
    servo_y_cmd = target_y
    xy_to_plus.set_xy(servo_x_cmd, servo_y_cmd, 1000)  # 用1秒时间指向初始角点
    time.sleep(1.0)  # 等待舵机到位

    # 初始化 X 和 Y 轴的 PID 控制器
    pid_x = PIDController(PID_KP, PID_KI, PID_KD)
    pid_y = PIDController(PID_KP, PID_KI, PID_KD)

    # 初始化绿色激光点位置的卡尔曼滤波器（假定起始位置即初始目标位置）
    kf_green_x = KalmanFilter1D(initial_pos=servo_x_cmd, initial_vel=0.0,
                                process_var=KF_PROCESS_VAR, meas_var=KF_MEASURE_VAR)
    kf_green_y = KalmanFilter1D(initial_pos=servo_y_cmd, initial_vel=0.0,
                                process_var=KF_PROCESS_VAR, meas_var=KF_MEASURE_VAR)

    # 打开相机预览模式获取连续帧（降低分辨率以提高速度）
    try:
        cam = Picamera2()
        cam.configure(cam.create_preview_configuration(
            main={"format": "RGB888", "size": PREVIEW_RES}
        ))
        cam.start()
        time.sleep(0.2)  # 预热略微等待
    except Exception as e:
        logging.error(f"Failed to open Picamera2 for tracking: {e}")
        return

    start_time = time.time()
    last_time = None

    # 主循环 - 运行 30 秒或直至中断
    while time.time() - start_time < 30:
        # 采集一帧图像
        try:
            frame_rgb = cam.capture_array()
        except Exception as e:
            logging.warning(f"Camera capture error: {e}")
            break
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        # 当前时间和时间步长 dt
        current_time = time.time()
        if last_time is None:
            dt = 1.0 / FRAME_RATE
        else:
            dt = current_time - last_time
        if dt <= 0 or dt > 0.2:  # 限制 dt，避免异常值
            dt = 1.0 / FRAME_RATE

        # 检测本帧中的绿色激光光点位置（应用透视矩阵 M 得到归一化坐标）
        pos = detect_laser.detect_green(frame_bgr, M)
        if pos is None:
            # 本帧未检测到绿色光点；跳过 PID 更新，但卡尔曼滤波器预测一步
            logging.warning("Green laser spot not found in frame.")
            # 根据模型预测下一步（避免滤波器状态停止更新）
            kf_green_x.predict(dt)
            kf_green_y.predict(dt)
            # 本次循环不改变舵机指令（保持上一命令）
            last_time = current_time
            continue

        # 如检测到，则用测量值更新卡尔曼滤波器
        current_x, current_y = pos
        # 卡尔曼滤波器预测并更新当前激光点位置
        kf_green_x.predict(dt)
        kf_green_y.predict(dt)
        kf_green_x.update(current_x)
        kf_green_y.update(current_y)
        # 使用滤波后的位置作为当前激光点的位置估计
        filtered_x = float(kf_green_x.x[0])
        filtered_y = float(kf_green_y.x[0])

        # 计算目标位置与当前（滤波后）位置之间的误差
        err_x = target_x - filtered_x
        err_y = target_y - filtered_y

        # 判断是否已到达当前目标角点附近
        dist_to_target = math.hypot(err_x, err_y)
        if dist_to_target < CORNER_THRESHOLD:
            # 切换到下一个路径角点（循环）
            target_index = (target_index + 1) % len(path_points)
            target_x = float(path_points[target_index][0])
            target_y = float(path_points[target_index][1])
            # 切换目标时重置 PID 控制器状态，以避免积分累积或微分突变
            pid_x.reset()
            pid_y.reset()
            # 可将滤波器的位置重新初始化为当前值（因为已到达角点）
            # 或保持滤波器连续跟踪 - 这里已重置了 PID 积分，滤波器可继续使用之前状态
            logging.info(f"Switched to next target corner: ({target_x:.1f}, {target_y:.1f})")
            # 立即计算新目标误差（避免等待下一帧）
            err_x = target_x - filtered_x
            err_y = target_y - filtered_y
            # 注意：未重置卡尔曼滤波器，以保持跟踪的连续性

        # PID 控制计算舵机运动
        output_x = pid_x.compute(err_x, dt)
        output_y = pid_y.compute(err_y, dt)

        # 更新舵机指令位置（加入 PID 输出增量，并限制在 0-100 范围）
        servo_x_cmd += output_x
        servo_y_cmd += output_y
        if servo_x_cmd < 0.0: servo_x_cmd = 0.0
        if servo_x_cmd > 100.0: servo_x_cmd = 100.0
        if servo_y_cmd < 0.0: servo_y_cmd = 0.0
        if servo_y_cmd > 100.0: servo_y_cmd = 100.0

        # 计算本次舵机移动的距离（归一化坐标系）
        step_dist = math.hypot(output_x, output_y)
        # 自适应移动时间：随距离变化，在 30 到 60 毫秒之间
        move_ms = int(max(30.0, min(60.0, 4.5 * step_dist)))
        # 发送舵机控制命令移动到新位置
        xy_to_plus.set_xy(servo_x_cmd, servo_y_cmd, move_ms)

        # 日志输出当前状态
        logging.info(f"Err: ({err_x:.1f}, {err_y:.1f}), "
                     f"Target: ({target_x:.1f}, {target_y:.1f}), "
                     f"Pos: ({filtered_x:.1f}, {filtered_y:.1f}), "
                     f"ServoCmd: ({servo_x_cmd:.1f}, {servo_y_cmd:.1f})")

        # 更新上一循环时间
        last_time = current_time

        # 保持循环帧率
        loop_time = time.time() - current_time
        sleep_time = (1.0 / FRAME_RATE) - loop_time
        if sleep_time > 0:
            time.sleep(sleep_time)

    # 循环结束后清理资源
    cam.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
