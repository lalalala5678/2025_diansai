#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
task1.py - 激光舵机系统标定脚本。
通过控制舵机扫描特定角度位置，采集图像并检测激光点位置，计算校准参数:
  ox, oy: 激光中心偏移 (逻辑坐标偏移量)
  angle_h, angle_v: 水平/垂直最大扫描角度 (度)
  pulse1_min, pulse1_max, pulse2_min, pulse2_max: 两通道舵机脉宽极值
标定过程:
  - 固定垂直舵机角度于中心，水平舵机从左到右扫描（及固定水平中心、垂直上下扫描），获取激光点对应图像坐标。
  - 将图像坐标映射为逻辑坐标（以0-100表示视场范围）。
  - 估计中心偏移ox, oy（舵机中心位置对应逻辑坐标相对50的偏差）。
  - 利用多点数据和几何关系，拟合水平和垂直扫描角度 angle_h, angle_v。
  - 保存结果到 plus.txt 文件。
"""

import time
import math
import logging
from statistics import median
# 假定存在 detect_laser 模块，提供 detect_green(frame) -> 返回 (x, y) 或 None
import detect_laser
# Picamera2 摄像头库
from picamera2 import Picamera2

# -------------------- 配置与初始化 --------------------
# 舵机控制参数 (初始值或上次校准值)，需根据硬件设置调整
# 这里假定已知舵机允许的脉宽范围，如500-2500微秒，或根据上次校准值预填
pulse1_min = 500    # 水平舵机最小脉宽 (左边界位置)
pulse1_max = 2500   # 水平舵机最大脉宽 (右边界位置)
pulse2_min = 500    # 垂直舵机最小脉宽 (下边界位置)
pulse2_max = 2500   # 垂直舵机最大脉宽 (上边界位置)

# 设置摄像头
picam2 = Picamera2()
# 创建全分辨率still配置 (这里假定相机模块可以支持2592x2592分辨率)
config = picam2.create_still_configuration(main={"size": (2592, 2592)})
picam2.configure(config)
picam2.start()
time.sleep(2)  # 给相机预热时间

# 日志配置
logging.basicConfig(filename='calibration.log', level=logging.INFO,
                    format='%(asctime)s [calibration] %(levelname)s: %(message)s')

logging.info("=== 开始激光舵机系统标定 ===")

# 舵机控制接口假定
def move_servos(pulse1, pulse2):
    """将水平舵机移动到pulse1，垂直舵机移动到pulse2，并等待运动完成。"""
    try:
        # 这里假设ServoControl.setPWMServoMove可以同时或分别移动两个舵机
        ServoControl.setPWMServoMove(1, int(pulse1))
        ServoControl.setPWMServoMove(2, int(pulse2))
    except Exception as e:
        logging.error(f"舵机移动命令失败: {e}")
        # 如有需要，可重试或退出
        time.sleep(0.5)
    # 等待舵机运动稳定 (根据舵机速度设定适当延时)
    time.sleep(0.2)

def capture_and_detect(num_frames=5):
    """
    拍摄num_frames帧图像，检测激光点位置，返回平均像素坐标 (x, y)。
    使用多帧取中值方式过滤噪声和偶发误检。
    如未检测到激光点，返回None。
    """
    coords_x = []
    coords_y = []
    for i in range(num_frames):
        # 拍照获取数组形式的图像
        frame = picam2.capture_array()
        result = detect_laser.detect_green(frame)
        if result is not None:
            x, y = result
            coords_x.append(x)
            coords_y.append(y)
            logging.debug(f"detect_green: 第{i+1}帧检测到激光点像素坐标 ({x}, {y})")
        else:
            logging.debug(f"detect_green: 第{i+1}帧未检测到激光点")
        # 稍作延时再采集下一帧
        time.sleep(0.1)
    if len(coords_x) == 0:
        # 所有帧都未检测到
        return None
    # 取中值滤波，减少异常值影响
    mx = median(coords_x)
    my = median(coords_y)
    logging.debug(f"激光点坐标集合 X={coords_x}, Y={coords_y}, 中值=({mx}, {my})")
    return (mx, my)

# 辅助函数: 将像素坐标转换为逻辑坐标(0-100)
def pixel_to_logic(x_pixel, y_pixel, left_ref, right_ref, bottom_ref, top_ref):
    """
    将激光点的图像像素坐标转换为逻辑坐标(0-100)。
    需要提供参考的左、右、下、上边界像素坐标。
    """
    # 防止除零
    if right_ref == left_ref or top_ref == bottom_ref:
        return None
    # x逻辑坐标: 相对于测得的左->右范围线性映射到0-100
    x_logic = (x_pixel - left_ref) / (right_ref - left_ref) * 100.0
    # y逻辑坐标: 相对于测得的下->上范围映射到0-100 (注意图像坐标y向下增大，逻辑坐标y向上增大)
    y_logic = (bottom_ref - y_pixel) / (bottom_ref - top_ref) * 100.0
    return (x_logic, y_logic)

# -------------------- 采集标定数据 --------------------
# 固定垂直居中，扫描水平方向
logging.info("1. 水平扫描: 固定垂直舵机在中心位置，扫描水平舵机从左到右边界")
# 计算水平和垂直中心脉宽
pulse1_center = (pulse1_min + pulse1_max) / 2.0
pulse2_center = (pulse2_min + pulse2_max) / 2.0

# 移动舵机到左边界位置 (pulse1_min, 垂直中心)
move_servos(pulse1_min, pulse2_center)
time.sleep(0.5)
left_frame = capture_and_detect()
if left_frame is None:
    logging.error("左边界激光点检测失败！")
    # 终止标定流程
    raise RuntimeError("左边界未检测到激光点，请调整相机或舵机范围")
left_x_pixel, left_y_pixel = left_frame
logging.info(f"左边界像素坐标: ({left_x_pixel:.1f}, {left_y_pixel:.1f})")

# 移动舵机到右边界位置 (pulse1_max, 垂直中心)
move_servos(pulse1_max, pulse2_center)
time.sleep(0.5)
right_frame = capture_and_detect()
if right_frame is None:
    logging.error("右边界激光点检测失败！")
    raise RuntimeError("右边界未检测到激光点，请调整相机或舵机范围")
right_x_pixel, right_y_pixel = right_frame
logging.info(f"右边界像素坐标: ({right_x_pixel:.1f}, {right_y_pixel:.1f})")

# 移动舵机到水平中心位置 (pulse1_center, 垂直中心)
move_servos(pulse1_center, pulse2_center)
time.sleep(0.3)
center_frame = capture_and_detect()
if center_frame is None:
    logging.warning("中心位置激光点检测失败，尝试继续标定")
    center_x_pixel, center_y_pixel = None, None
else:
    center_x_pixel, center_y_pixel = center_frame
    logging.info(f"中心位置像素坐标: ({center_x_pixel:.1f}, {center_y_pixel:.1f})")

# 水平方向校正计算
# 将像素坐标转换为逻辑坐标（先以左0、右100线性映射）
left_logic_x = 0.0
right_logic_x = 100.0
# 中心逻辑坐标计算（若检测到）
if center_x_pixel is not None:
    center_logic_x = (center_x_pixel - left_x_pixel) / (right_x_pixel - left_x_pixel) * 100.0
else:
    # 若未检测中心，用50作为默认中心逻辑值
    center_logic_x = 50.0

ox = center_logic_x - 50.0  # 水平原点偏移: 中心相对50的差值
logging.info(f"水平原点偏移ox = {ox:.3f} (逻辑坐标)")

# 采集几个水平中间点用于拟合水平扫描角度
# 选择在左右边界1/4和3/4处的位置
logging.info("采集水平1/4、3/4位置的激光点...")
quarter_pulse = pulse1_min + 0.25 * (pulse1_max - pulse1_min)
three_quarter_pulse = pulse1_min + 0.75 * (pulse1_max - pulse1_min)

move_servos(quarter_pulse, pulse2_center)
time.sleep(0.3)
quarter_frame = capture_and_detect()
move_servos(three_quarter_pulse, pulse2_center)
time.sleep(0.3)
three_quarter_frame = capture_and_detect()

if quarter_frame and three_quarter_frame:
    q_x_pixel, q_y_pixel = quarter_frame
    tq_x_pixel, tq_y_pixel = three_quarter_frame
    # 转为逻辑坐标
    q_logic_x = (q_x_pixel - left_x_pixel) / (right_x_pixel - left_x_pixel) * 100.0
    tq_logic_x = (tq_x_pixel - left_x_pixel) / (right_x_pixel - left_x_pixel) * 100.0
    # 将逻辑坐标修正偏移
    q_logic_x_adj = q_logic_x - ox
    tq_logic_x_adj = tq_logic_x - ox
    logging.info(f"水平1/4位置逻辑坐标(校正后) = {q_logic_x_adj:.2f}, 3/4位置逻辑坐标(校正后) = {tq_logic_x_adj:.2f}")
    # 计算相对于中心50的差值
    diff_left = 50.0 - q_logic_x_adj
    diff_right = tq_logic_x_adj - 50.0
    # 平均偏离
    avg_diff = (diff_left + diff_right) / 2.0
    # 通过迭代/二分求解 angle_h 使 tan(angle_h/4)/tan(angle_h/2) = avg_diff/50:contentReference[oaicite:2]{index=2}
    # 定义方程 f(a) = tan(a/4)/tan(a/2) - (avg_diff/50) = 0，求a的根 (单位：度)
    target_ratio = avg_diff / 50.0
    logging.info(f"水平角度拟合: target_ratio = {target_ratio:.4f}")
    # 二分法求解 angle_h
    low, high = 1.0, 179.0
    angle_h_est = None
    for _ in range(30):  # 迭代30次足够精度
        mid = (low + high) / 2.0
        mid_rad = math.radians(mid)
        ratio = math.tan(mid_rad / 4.0) / math.tan(mid_rad / 2.0)
        if ratio > target_ratio:
            high = mid
        else:
            low = mid
        angle_h_est = mid
    angle_h_calibrated = angle_h_est if angle_h_est is not None else (pulse1_max - pulse1_min) / (pulse1_max - pulse1_min) * 180.0
    logging.info(f"拟合得到水平最大扫描角度 angle_h ≈ {angle_h_calibrated:.2f} 度")
else:
    # 若中间点检测不足，则根据舵机额定转角估计角度范围
    angle_h_calibrated =  math.fabs(pulse1_max - pulse1_min) * 0.1  # 粗略估计: 10us ~ 1度:contentReference[oaicite:3]{index=3}:contentReference[oaicite:4]{index=4}
    logging.warning(f"水平中间点数据不足，采用默认估计 angle_h = {angle_h_calibrated:.2f} 度")

# 垂直方向扫描（固定水平居中，垂直扫描上下）
logging.info("2. 垂直扫描: 固定水平舵机在中心位置，扫描垂直舵机从下到上边界")
# 移动到下边界 (pulse2_min)
move_servos(pulse1_center, pulse2_min)
time.sleep(0.5)
bottom_frame = capture_and_detect()
if bottom_frame is None:
    logging.error("下边界激光点检测失败！")
    raise RuntimeError("下边界未检测到激光点，请调整相机或舵机范围")
bottom_x_pixel, bottom_y_pixel = bottom_frame
logging.info(f"下边界像素坐标: ({bottom_x_pixel:.1f}, {bottom_y_pixel:.1f})")

# 上边界
move_servos(pulse1_center, pulse2_max)
time.sleep(0.5)
top_frame = capture_and_detect()
if top_frame is None:
    logging.error("上边界激光点检测失败！")
    raise RuntimeError("上边界未检测到激光点，请调整相机或舵机范围")
top_x_pixel, top_y_pixel = top_frame
logging.info(f"上边界像素坐标: ({top_x_pixel:.1f}, {top_y_pixel:.1f})")

# 中心（水平中心，垂直中心在上面已取）
# （水平中心脉宽已在中间位置，无需移动，直接利用之前的center_frame数据）
if center_frame is None:
    # 若之前中心没取到，这里再尝试以垂直中心获取
    move_servos(pulse1_center, pulse2_center)
    time.sleep(0.3)
    center_frame = capture_and_detect()
    if center_frame is None:
        logging.warning("中心位置(垂直)激光点再次未检测到，继续标定")
        center_x_pixel_v = None
        center_y_pixel_v = None
    else:
        center_x_pixel_v, center_y_pixel_v = center_frame
else:
    # 如果之前中心frame有数据，那么水平和垂直中心都在那里
    center_x_pixel_v, center_y_pixel_v = center_x_pixel, center_y_pixel

# 垂直方向校正计算
if center_y_pixel_v is not None:
    center_logic_y = (bottom_y_pixel - center_y_pixel_v) / (bottom_y_pixel - top_y_pixel) * 100.0
else:
    center_logic_y = 50.0
oy = center_logic_y - 50.0
logging.info(f"垂直原点偏移oy = {oy:.3f} (逻辑坐标)")

# 垂直1/4和3/4位置
logging.info("采集垂直1/4、3/4位置的激光点...")
quarter_pulse_v = pulse2_min + 0.25 * (pulse2_max - pulse2_min)
three_quarter_pulse_v = pulse2_min + 0.75 * (pulse2_max - pulse2_min)

move_servos(pulse1_center, quarter_pulse_v)
time.sleep(0.3)
quarter_v_frame = capture_and_detect()
move_servos(pulse1_center, three_quarter_pulse_v)
time.sleep(0.3)
three_quarter_v_frame = capture_and_detect()

if quarter_v_frame and three_quarter_v_frame:
    qv_x_pixel, qv_y_pixel = quarter_v_frame
    tqv_x_pixel, tqv_y_pixel = three_quarter_v_frame
    # 垂直逻辑坐标
    q_logic_y = (bottom_y_pixel - qv_y_pixel) / (bottom_y_pixel - top_y_pixel) * 100.0
    tq_logic_y = (bottom_y_pixel - tqv_y_pixel) / (bottom_y_pixel - top_y_pixel) * 100.0
    # 校正偏移
    q_logic_y_adj = q_logic_y - oy
    tq_logic_y_adj = tq_logic_y - oy
    logging.info(f"垂直1/4位置逻辑坐标(校正后) = {q_logic_y_adj:.2f}, 3/4位置逻辑坐标(校正后) = {tq_logic_y_adj:.2f}")
    diff_bottom = 50.0 - q_logic_y_adj
    diff_top = tq_logic_y_adj - 50.0
    avg_diff_v = (diff_bottom + diff_top) / 2.0
    target_ratio_v = avg_diff_v / 50.0
    logging.info(f"垂直角度拟合: target_ratio = {target_ratio_v:.4f}")
    # 二分法求解 angle_v（度）
    low, high = 1.0, 179.0
    angle_v_est = None
    for _ in range(30):
        mid = (low + high) / 2.0
        mid_rad = math.radians(mid)
        ratio = math.tan(mid_rad / 4.0) / math.tan(mid_rad / 2.0)
        if ratio > target_ratio_v:
            high = mid
        else:
            low = mid
        angle_v_est = mid
    angle_v_calibrated = angle_v_est if angle_v_est is not None else (pulse2_max - pulse2_min) / (pulse2_max - pulse2_min) * 180.0
    logging.info(f"拟合得到垂直最大扫描角度 angle_v ≈ {angle_v_calibrated:.2f} 度")
else:
    angle_v_calibrated = math.fabs(pulse2_max - pulse2_min) * 0.1
    logging.warning(f"垂直中间点数据不足，采用默认估计 angle_v = {angle_v_calibrated:.2f} 度")

# -------------------- 保存结果 --------------------
logging.info("3. 保存标定结果到配置文件 plus.txt")
try:
    with open("plus.txt", "w") as f:
        f.write(f"ox={ox:.4f}\n")
        f.write(f"oy={oy:.4f}\n")
        f.write(f"angle_h={angle_h_calibrated:.4f}\n")
        f.write(f"angle_v={angle_v_calibrated:.4f}\n")
        f.write(f"pulse1_min={pulse1_min:.0f}\n")
        f.write(f"pulse1_max={pulse1_max:.0f}\n")
        f.write(f"pulse2_min={pulse2_min:.0f}\n")
        f.write(f"pulse2_max={pulse2_max:.0f}\n")
    logging.info("标定参数已成功写入 plus.txt")
    print("Calibration completed. Parameters saved to plus.txt.")
except Exception as e:
    logging.error(f"写入 plus.txt 时发生错误: {e}")
    raise

# 结束日志
logging.info(f"=== 标定完成: ox={ox:.3f}, oy={oy:.3f}, angle_h={angle_h_calibrated:.2f}, angle_v={angle_v_calibrated:.2f} ===")
