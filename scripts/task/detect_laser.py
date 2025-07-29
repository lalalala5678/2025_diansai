import cv2
import numpy as np

# 定义HSV颜色阈值范围
# 绿色激光HSV范围 (H约60度，S和V要求较高)
LOWER_GREEN = np.array([40, 100, 50])   # 下界: H>=40, S>=100, V>=50
UPPER_GREEN = np.array([80, 255, 255])  # 上界: H<=80
# 红色激光HSV范围需要考虑红色环绕Hue轴两端的情况
LOWER_RED1 = np.array([0, 100, 50])     # 红色低Hue区间
UPPER_RED1 = np.array([10, 255, 255])
LOWER_RED2 = np.array([170, 100, 50])   # 红色高Hue区间
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
if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    if not ret:
        print("Camera capture failed.")
    else:
        # 需要有提前标定的M矩阵，这里假设为单位矩阵以进行测试
        M = np.eye(3)
        g_pos = detect_green(frame, M, debug=True)
        r_pos = detect_red(frame, M, debug=True)
        print("Green laser position:", g_pos)
        print("Red laser position:", r_pos)
    cap.release()
    cv2.destroyAllWindows()
