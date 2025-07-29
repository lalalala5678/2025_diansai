import cv2
import numpy as np
from picamera2 import Picamera2
import time



def detect_rectangle(frame, debug=False):
    """
    检测给定图像中的黑色胶带矩形轮廓。
    返回值:
        norm_corners (numpy.ndarray): 形状为(4,2)的数组，表示矩形四个角点的坐标（已映射到0-100坐标系）。
        M (numpy.ndarray): 3x3透视变换矩阵，将原始图像坐标映射到0-100坐标系。
        src_pts (numpy.ndarray): 形状为(4,2)的数组，表示原始图像中矩形四个角点的像素坐标。
    如果未检测到矩形，返回 (None, None, None)。
    
    当 debug=True 时，会打印调试信息。
    调试信息为检测到的角点坐标和透视变换矩阵。
    """
    # 将图像转换为灰度并进行模糊，以减轻噪声影响
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)
    # 使用大津阈值法进行二值化（阈值取反，使黑色胶带区域为白色）
    _, thresh = cv2.threshold(gray_blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    # 查找图像中的所有外轮廓
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if debug:
        print(f"[detect_rectangle] Found {len(contours)} contours")
    if not contours:
        if debug:
            print("[detect_rectangle] No contours found.")
        return None, None

    # 寻找最大面积的轮廓，假设为黑色矩形边框
    largest_contour = max(contours, key=cv2.contourArea)
    # 将该轮廓多边形逼近，简化边缘点
    peri = cv2.arcLength(largest_contour, True)
    approx = cv2.approxPolyDP(largest_contour, 0.02 * peri, True)
    if debug:
        print(f"[detect_rectangle] Approximated contour has {len(approx)} vertices")
    corners = None
    if len(approx) == 4:
        # 如果逼近得到4个顶点，直接使用
        corners = approx.reshape(4, 2)
    else:
        # 顶点不是4个的情况，尝试使用凸包获取大致的矩形框
        all_points = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            # 忽略很小的噪声轮廓
            if area < 50:
                continue
            for p in cnt:
                all_points.append(p[0])  # 提取轮廓点坐标
        all_points = np.array(all_points)
        if all_points.size == 0:
            if debug:
                print("[detect_rectangle] No significant contour points for hull.")
            return None, None
        # 计算所有点的凸包并逼近
        hull = cv2.convexHull(all_points)
        peri_hull = cv2.arcLength(hull, True)
        approx_hull = cv2.approxPolyDP(hull, 0.02 * peri_hull, True)
        if debug:
            print(f"[detect_rectangle] Convex hull has {len(approx_hull)} vertices after approximation")
        if len(approx_hull) == 4:
            corners = approx_hull.reshape(4, 2)
        else:
            # 最后手段: 使用最小包围矩形
            rect = cv2.minAreaRect(all_points)  # 获取旋转矩形
            corners = cv2.boxPoints(rect)       # 获取矩形的4个角点
            corners = np.array(corners, dtype=np.float32)
            if debug:
                print("[detect_rectangle] Used minAreaRect for corner estimation")

    if corners is None:
        if debug:
            print("[detect_rectangle] Rectangle not detected.")
        return None, None

    # 对检测到的角点进行排序: 顺序为 [top-left, top-right, bottom-right, bottom-left]
    sum_vals = corners.sum(axis=1)
    diff_vals = np.diff(corners, axis=1).reshape(-1)
    tl_index = np.argmin(sum_vals)   # 左上角: x+y 最小
    br_index = np.argmax(sum_vals)   # 右下角: x+y 最大
    tr_index = np.argmin(diff_vals)  # 右上角: y-x 最小
    bl_index = np.argmax(diff_vals)  # 左下角: y-x 最大
    ordered_idx = [tl_index, tr_index, br_index, bl_index]
    # 去重以防万一（通常矩形能得到4个独立点）
    ordered_idx = list(dict.fromkeys(ordered_idx))
    if len(ordered_idx) != 4:
        # 如果排序结果不是4个点，按照x坐标和y坐标排序作为备用方案
        pts = corners[corners[:, 0].argsort()]
        left_two = pts[:2]   # x较小的两个点
        right_two = pts[2:]  # x较大的两个点
        left_two = left_two[left_two[:, 1].argsort()]    # 左边两个按y排序（上、下）
        right_two = right_two[right_two[:, 1].argsort()] # 右边两个按y排序
        ordered = np.vstack([left_two, right_two])
    else:
        ordered = corners[ordered_idx]

    # 计算从图像坐标到0-100目标坐标的透视变换矩阵
    src_pts = np.array(ordered, dtype=np.float32)
    dst_pts = np.array([[0.0, 0.0],
                        [100.0, 0.0],
                        [100.0, 100.0],
                        [0.0, 100.0]], dtype=np.float32)
    M = cv2.getPerspectiveTransform(src_pts, dst_pts)

    # 将角点映射到0-100坐标系并取整处理
    pts = src_pts.reshape(1, 4, 2)
    norm_corners = cv2.perspectiveTransform(pts, M).reshape(4, 2)
    norm_corners = np.clip(norm_corners, 0, 100)               # 限制范围在0-100
    norm_corners = np.rint(norm_corners).astype(int)           # 四舍五入取整
    if debug:
        print("[detect_rectangle] Ordered corners (pixel):", src_pts)
        print("[detect_rectangle] Normalized corners (0-100):", norm_corners)
    return norm_corners, src_pts, M   # 多返回一个 src_pts (4,2) 像素角点


# 如果直接运行本模块，可以进行简单的摄像头测试（调试用途）

# 如果直接运行本模块，可以进行简单的摄像头测试并把检测结果保存到当前目录
if __name__ == "__main__":


    SAVE_PATH = "rect_debug.jpg"

    cam = Picamera2()
    cam.configure(cam.create_preview_configuration(
        main={"format": "RGB888", "size": (2592, 2592)}))
    cam.start(); time.sleep(0.4)
    frame_rgb = cam.capture_array(); cam.close()
    frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

    norm_corners, pix_corners, M = detect_rectangle(frame_bgr, debug=True)

    if norm_corners is not None:
        vis = frame_bgr.copy()
        pts = pix_corners.astype(int)           # 真实像素
        cv2.polylines(vis, [pts.reshape(-1,1,2)], True, (0,255,0), 2)
        for (x,y) in pts: cv2.circle(vis,(x,y),6,(0,0,255),-1)
        cv2.imwrite(SAVE_PATH, vis)
        print(f"[INFO] saved {SAVE_PATH}")
        print("Normalized:", norm_corners.tolist())
    else:
        print("No rectangle detected.")


