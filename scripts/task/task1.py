# task1.py – 激光光斑舵机系统标定脚本
# 用于标定 xy_to_plus.py 模块的参数 (p1, p2, p3, p4)，自动更新 plus.txt。

import os
import time
import logging
import numpy as np
import cv2
import ServoControl
import detect_laser

# ---------- 参数配置 ----------
MATRIX_FILE = "perspective.npy"    # 透视变换矩阵文件名（如存在则加载）
SAFE_MARGIN = 5.0                 # 光斑距离画面边缘的安全余量 (逻辑坐标值)
INIT_STEP = 25                    # 初始扫描步进的脉宽值
FINE_STEP = 5                     # 接近目标时的细调步进值
SERVO_MIN_PULSE = 500             # 舵机脉宽安全下限
SERVO_MAX_PULSE = 2500            # 舵机脉宽安全上限

# ---------- plus.txt 参数读写 ----------
def load_plus_params():
    """读取当前 plus.txt 参数，如不存在则返回默认参数。"""
    defaults = {"p1": 300, "p2": 400, "p3": 300, "p4": 400}
    params = {}
    cfg_path = os.path.join(os.path.dirname(__file__), "plus.txt")
    if os.path.isfile(cfg_path):
        with open(cfg_path, "r", encoding="utf-8") as f:
            text = f.read().replace("\n", ",")
            for token in text.split(","):
                token = token.strip()
                if not token:
                    continue
                try:
                    key, val = token.split("=", 1)
                    params[key.strip()] = int(val)
                except ValueError:
                    # 跳过无法解析的部分
                    continue
    for k, v in defaults.items():
        params.setdefault(k, v)
    return params

def save_plus_params(p1, p2, p3, p4):
    """将标定得到的参数保存回 plus.txt 文件。"""
    cfg_path = os.path.join(os.path.dirname(__file__), "plus.txt")
    with open(cfg_path, "w", encoding="utf-8") as f:
        f.write(f"p1={p1}\n")
        f.write(f"p2={p2}\n")
        f.write(f"p3={p3}\n")
        f.write(f"p4={p4}\n")

# ---------- 获取透视矩阵 ----------
def get_perspective_matrix(width, height):
    """
    获取用于激光点坐标转换的透视矩阵 M (3x3)。
    若存在 MATRIX_FILE 则尝试加载，否则根据图像尺寸构造默认线性矩阵（将整个画面映射为0-100坐标）。
    """
    M = None
    cfg_path = os.path.join(os.path.dirname(__file__), MATRIX_FILE)
    if os.path.isfile(cfg_path):
        try:
            M = np.load(cfg_path)
            if M.shape != (3, 3):
                logging.warning(f"{MATRIX_FILE} 格式不正确，将使用默认矩阵")
                M = None
        except Exception as e:
            logging.warning(f"加载 {MATRIX_FILE} 出错: {e}，将使用默认矩阵")
            M = None
    if M is None:
        # 默认矩阵: 将像素坐标线性映射到0-100逻辑坐标（假定摄像头画面无透视畸变）
        sx = 100.0 / (width - 1)
        sy = 100.0 / (height - 1)
        # y轴翻转: top->100, bottom->0
        M = np.array([[ sx,   0,      0 ],
                      [  0,  -sy,   100 ],
                      [  0,   0,      1 ]], dtype=np.float32)
        logging.info("未找到透视矩阵文件，使用默认线性矩阵")
    else:
        logging.info(f"透视矩阵已加载自 {MATRIX_FILE}")
    return M

# ---------- 激光点检测辅助函数 ----------
def capture_and_detect(cam, M, color='green', max_retries=2):
    """
    捕获当前相机帧并检测指定颜色激光光斑的归一化坐标。
    color 可为 'green' 或 'red'，max_retries 指定检测失败时重试次数。
    成功返回 (x, y) 坐标元组，未检测到返回 None。
    """
    frame_rgb = cam.capture_array()
    frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    if color == 'green':
        coord = detect_laser.detect_green(frame_bgr, M, debug=False)
    else:
        coord = detect_laser.detect_red(frame_bgr, M, debug=False)
    if coord is None and max_retries > 0:
        # 若未检测到，等待片刻重试
        time.sleep(0.1)
        frame_rgb = cam.capture_array()
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
        if color == 'green':
            coord = detect_laser.detect_green(frame_bgr, M, debug=False)
        else:
            coord = detect_laser.detect_red(frame_bgr, M, debug=False)
    return coord

# ---------- 标定流程 ----------
def calibrate():
    logging.info("==== 开始标定舵机与激光光斑坐标关系 ====")
    params = load_plus_params()
    old_p1, old_p2, old_p3, old_p4 = params["p1"], params["p2"], params["p3"], params["p4"]
    logging.info(f"当前 plus 参数: p1={old_p1}, p2={old_p2}, p3={old_p3}, p4={old_p4}")
    # 计算并移动到初始中心位置
    center_x_pulse = int(round((params["p1"] + params["p2"]) * 5 / 2))
    center_y_pulse = int(round((params["p3"] + params["p4"]) * 5 / 2))
    # 若参数为默认值(300/400等)，上述计算约为 1750; 初次标定也可直接使用1500作为中点
    ServoControl.setPWMServoMove(1, center_x_pulse, 500)
    ServoControl.setPWMServoMove(2, center_y_pulse, 500)
    time.sleep(0.5)
    logging.info(f"舵机移至初始位置 (pulse1={center_x_pulse}, pulse2={center_y_pulse})")
    # 初始化相机 (使用高分辨率全画幅捕获)
    cam = detect_laser.Picamera2()
    cam.configure(cam.create_still_configuration(main={"format": "RGB888", "size": (2592, 2592)}))
    cam.start()
    time.sleep(1.0)  # 等待相机预热
    M = get_perspective_matrix(2592, 2592)
    # 检测中心点光斑坐标
    center_coord = capture_and_detect(cam, M, color='green')
    if center_coord is None:
        cam.close()
        logging.error("中心位置未检测到激光光斑，请检查激光器和摄像头！标定中止。")
        return
    cx, cy = center_coord
    logging.info(f"中心位置光斑坐标: ({cx:.1f}, {cy:.1f})")
    # 判定舵机旋转方向（水平和垂直）
    # 水平方向: 增大通道1脉宽的影响
    test_pulse = min(center_x_pulse + 50, SERVO_MAX_PULSE)
    ServoControl.setPWMServoMove(1, test_pulse, 300)
    time.sleep(0.3)
    coord2 = capture_and_detect(cam, M, color='green')
    if coord2 is None:
        logging.warning("水平方向小幅移动后光斑未检测到，假定舵机方向为默认")
        horizontal_normal = True
    else:
        horizontal_normal = (coord2[0] < cx)  # 脉宽增→x减小，则为正常方向
    # 将舵机恢复中心
    ServoControl.setPWMServoMove(1, center_x_pulse, 300)
    time.sleep(0.3)
    # 垂直方向: 增大通道2脉宽的影响
    test_pulse = min(center_y_pulse + 50, SERVO_MAX_PULSE)
    ServoControl.setPWMServoMove(2, test_pulse, 300)
    time.sleep(0.3)
    coord2 = capture_and_detect(cam, M, color='green')
    if coord2 is None:
        logging.warning("垂直方向小幅移动后光斑未检测到，假定舵机方向为默认")
        vertical_normal = True
    else:
        vertical_normal = (coord2[1] < cy)  # 脉宽增→y减小，则为正常方向
    ServoControl.setPWMServoMove(2, center_y_pulse, 300)
    time.sleep(0.3)
    logging.info(f"水平舵机方向: {'正常' if horizontal_normal else '反向'}（脉宽增大光点向{'左' if horizontal_normal else '右'}移）")
    logging.info(f"垂直舵机方向: {'正常' if vertical_normal else '反向'}（脉宽增大光点向{'下' if vertical_normal else '上'}移）")
    # 开始标定各边界
    # 水平左边界 (接近 x=0 + SAFE_MARGIN)
    logging.info(">>> 标定左边界...")
    current_pulse = center_x_pulse
    last_detect_pulse = current_pulse
    last_detect_x = cx
    target_x = SAFE_MARGIN
    # 确定朝左扫描的方向
    direction = 1 if horizontal_normal else -1
    dx = INIT_STEP
    while True:
        current_pulse += direction * dx
        if current_pulse < SERVO_MIN_PULSE or current_pulse > SERVO_MAX_PULSE:
            logging.warning(f"水平方向左扫达到舵机极限（pulse={current_pulse}），停止扫描")
            break
        ServoControl.setPWMServoMove(1, current_pulse, 300)
        time.sleep(0.3)
        coord = capture_and_detect(cam, M, color='green')
        if coord is None:
            logging.info("光斑已移出左侧视野")
            break
        last_detect_pulse = current_pulse
        last_detect_x = coord[0]
        if last_detect_x <= target_x:
            logging.info(f"达到左侧目标区域: x≈{last_detect_x:.1f} ≤ {target_x}")
            break
        # 接近目标区域时减小步进提高精度
        if last_detect_x - target_x < 2 * dx * (100/(2592-1)):
            dx = FINE_STEP
    left_pulse = last_detect_pulse
    logging.info(f"左边界脉宽={left_pulse}, 检测x≈{last_detect_x:.1f}")
    # 回到水平中心
    ServoControl.setPWMServoMove(1, center_x_pulse, 500)
    time.sleep(0.5)
    # 水平右边界 (接近 x=100 - SAFE_MARGIN)
    logging.info(">>> 标定右边界...")
    current_pulse = center_x_pulse
    last_detect_pulse = current_pulse
    last_detect_x = cx
    target_x = 100.0 - SAFE_MARGIN
    direction = -1 if horizontal_normal else 1
    dx = INIT_STEP
    while True:
        current_pulse += direction * dx
        if current_pulse < SERVO_MIN_PULSE or current_pulse > SERVO_MAX_PULSE:
            logging.warning(f"水平方向右扫达到舵机极限（pulse={current_pulse}），停止扫描")
            break
        ServoControl.setPWMServoMove(1, current_pulse, 300)
        time.sleep(0.3)
        coord = capture_and_detect(cam, M, color='green')
        if coord is None:
            logging.info("光斑已移出右侧视野")
            break
        last_detect_pulse = current_pulse
        last_detect_x = coord[0]
        if last_detect_x >= target_x:
            logging.info(f"达到右侧目标区域: x≈{last_detect_x:.1f} ≥ {target_x}")
            break
        if target_x - last_detect_x < 2 * dx * (100/(2592-1)):
            dx = FINE_STEP
    right_pulse = last_detect_pulse
    logging.info(f"右边界脉宽={right_pulse}, 检测x≈{last_detect_x:.1f}")
    ServoControl.setPWMServoMove(1, center_x_pulse, 500)
    time.sleep(0.5)
    # 垂直下边界 (接近 y=0 + SAFE_MARGIN)
    logging.info(">>> 标定下边界...")
    current_pulse = center_y_pulse
    last_detect_pulse = current_pulse
    last_detect_y = cy
    target_y = SAFE_MARGIN
    direction = 1 if vertical_normal else -1
    dx = INIT_STEP
    while True:
        current_pulse += direction * dx
        if current_pulse < SERVO_MIN_PULSE or current_pulse > SERVO_MAX_PULSE:
            logging.warning(f"垂直方向下扫达到舵机极限（pulse={current_pulse}），停止扫描")
            break
        ServoControl.setPWMServoMove(2, current_pulse, 300)
        time.sleep(0.3)
        coord = capture_and_detect(cam, M, color='green')
        if coord is None:
            logging.info("光斑已移出下边界视野")
            break
        last_detect_pulse = current_pulse
        last_detect_y = coord[1]
        if last_detect_y <= target_y:
            logging.info(f"达到下侧目标区域: y≈{last_detect_y:.1f} ≤ {target_y}")
            break
        if last_detect_y - target_y < 2 * dx * (100/(2592-1)):
            dx = FINE_STEP
    bottom_pulse = last_detect_pulse
    logging.info(f"下边界脉宽={bottom_pulse}, 检测y≈{last_detect_y:.1f}")
    ServoControl.setPWMServoMove(2, center_y_pulse, 500)
    time.sleep(0.5)
    # 垂直上边界 (接近 y=100 - SAFE_MARGIN)
    logging.info(">>> 标定上边界...")
    current_pulse = center_y_pulse
    last_detect_pulse = current_pulse
    last_detect_y = cy
    target_y = 100.0 - SAFE_MARGIN
    direction = -1 if vertical_normal else 1
    dx = INIT_STEP
    while True:
        current_pulse += direction * dx
        if current_pulse < SERVO_MIN_PULSE or current_pulse > SERVO_MAX_PULSE:
            logging.warning(f"垂直方向上扫达到舵机极限（pulse={current_pulse}），停止扫描")
            break
        ServoControl.setPWMServoMove(2, current_pulse, 300)
        time.sleep(0.3)
        coord = capture_and_detect(cam, M, color='green')
        if coord is None:
            logging.info("光斑已移出上边界视野")
            break
        last_detect_pulse = current_pulse
        last_detect_y = coord[1]
        if last_detect_y >= target_y:
            logging.info(f"达到上侧目标区域: y≈{last_detect_y:.1f} ≥ {target_y}")
            break
        if target_y - last_detect_y < 2 * dx * (100/(2592-1)):
            dx = FINE_STEP
    top_pulse = last_detect_pulse
    logging.info(f"上边界脉宽={top_pulse}, 检测y≈{last_detect_y:.1f}")
    cam.close()
    # 计算并保存新的 plus 参数
    new_p1 = int(round(right_pulse / 5))    # X轴100%
    new_p2 = int(round(left_pulse / 5))     # X轴0%
    new_p3 = int(round(top_pulse / 5))      # Y轴100%
    new_p4 = int(round(bottom_pulse / 5))   # Y轴0%
    save_plus_params(new_p1, new_p2, new_p3, new_p4)
    logging.info(f"标定完成，新 plus 参数: p1={new_p1}, p2={new_p2}, p3={new_p3}, p4={new_p4}")
    logging.info("plus.txt 已更新")
    logging.info(f"参数改动: Δp1={new_p1-old_p1}, Δp2={new_p2-old_p2}, Δp3={new_p3-old_p3}, Δp4={new_p4-old_p4}")

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    calibrate()
        # ========== 标定后验证四个测试点 ========== #

    # 标定完成后验证四个预设测试点的光斑检测

    import xy_to_plus  # 复用统一的坐标→脉宽映射

    TEST_POINTS = [(30, 30), (30, 70), (70, 30), (70, 70)]

    # 重新开启相机（标定流程已关闭 cam）
    cam = detect_laser.Picamera2()
    cam.configure(cam.create_still_configuration(
        main={"format": "RGB888", "size": (2592, 2592)}))
    cam.start()
    time.sleep(1.0)                          # 预热
    M = get_perspective_matrix(2592, 2592)   # 同一映射

    for (tx, ty) in TEST_POINTS:
        # 计算脉宽并下发
        # pulse_x, pulse_y = xy_to_plus.map_coords_to_pulses(tx, ty)
        # ServoControl.setPWMServoMove(1, pulse_x, 600)
        # ServoControl.setPWMServoMove(2, pulse_y, 600)

        # 使用ServoControl.set_xy()方法直接控制舵机
        pulse_x, pulse_y = xy_to_plus.set_xy(tx, ty, waiting_ms=1000)
        logging.info(f"指令: ({tx:.1f}, {ty:.1f}) → 脉宽: ({pulse_x}, {pulse_y})")
        time.sleep(1)                      # 等待舵机到位与光斑稳定

        # 拍照并检测
        detected = capture_and_detect(cam, M, color='green')
        if detected is None:
            logging.warning(f"指令 ({tx:.1f}, {ty:.1f}) 未检测到光斑。")
        else:
            dx, dy = detected
            logging.info(f"指令 ({tx:.1f}, {ty:.1f}) → 检测 ({dx:.1f}, {dy:.1f})  "
                         f"Δ=({dx-tx:+.1f}, {dy-ty:+.1f})")

    cam.close()
    logging.info("四点验证完成，task1.py 退出")
