import cv2
import time
import numpy as np
from picamera2 import Picamera2
from typing import Tuple, List
import ServoControl
from xy_to_plus import map_coords_to_pulses

# ---------------- 图像捕获与裁剪 ---------------- #
def capture_image() -> np.ndarray:
    cam = Picamera2()
    cam.configure(cam.create_still_configuration())
    cam.start()
    time.sleep(2)
    frame = cam.capture_array()
    cam.close()
    return frame

def crop_image(img: np.ndarray, center: Tuple[int, int], width: int, height: int) -> np.ndarray:
    x, y = center
    h, w = img.shape[:2]
    w2, h2 = width // 2, height // 2
    x1, y1 = max(0, x - w2), max(0, y - h2)
    x2, y2 = min(w, x + w2), min(h, y + h2)
    return img[y1:y2, x1:x2].copy()

# ---------------- 矩形检测（引入detect_rec） ---------------- #
def find_black_rectangle_corners(debug: bool = False, save_path: str = "debug_detected.jpg") -> List[Tuple[float, float]]:
    original_img = capture_image()
    h, w = original_img.shape[:2]
    side_len = min(w, h)
    center_pt = (w // 2, h // 2)
    cropped_img = crop_image(original_img, center_pt, side_len, side_len)
    resized_img = cv2.resize(cropped_img, (480, 480))
    height, width = resized_img.shape[:2]

    gray = cv2.cvtColor(resized_img, cv2.COLOR_BGR2GRAY)
    gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(gray_blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    kernel = np.ones((5, 5), np.uint8)
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return []

    contour = max(contours, key=cv2.contourArea)
    peri = cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, 0.02 * peri, True)
    if len(approx) == 4:
        corners = [tuple(pt[0]) for pt in approx]
    else:
        rot_rect = cv2.minAreaRect(contour)
        box = cv2.boxPoints(rot_rect)
        corners = [tuple(pt) for pt in box]

    def sort_corners_clockwise(c_pts):
        c_pts = sorted(c_pts, key=lambda p: p[0] + p[1])
        tl = c_pts[0]; br = c_pts[-1]
        other = c_pts[1:-1]
        tr, bl = (other[0], other[1]) if other[0][0] > other[1][0] else (other[1], other[0])
        return [tl, tr, br, bl]

    corners = sort_corners_clockwise(corners)

    result_points = []
    for (px, py) in corners:
        X = (px / (width - 1)) * 100.0
        Y = ((height - 1 - py) / (height - 1)) * 100.0
        result_points.append((round(X, 4), round(Y, 4)))

    if debug:
        pts_int = np.array([[int(x), int(y)] for (x, y) in corners], dtype=np.int32)
        cv2.polylines(resized_img, [pts_int.reshape(-1, 1, 2)], True, (0, 255, 0), 2)
        font = cv2.FONT_HERSHEY_SIMPLEX
        for idx, (px, py) in enumerate(pts_int):
            cv2.circle(resized_img, (px, py), 5, (0, 0, 255), -1)
            norm_x, norm_y = result_points[idx]
            text = f"{idx}:({norm_x:.1f},{norm_y:.1f})"
            cv2.putText(resized_img, text, (px + 6, py + 18), font, 0.45, (255, 0, 0), 1, cv2.LINE_AA)
        cv2.imwrite(save_path, resized_img)
        print(f"[DEBUG] 检测结果已保存为 {save_path}")

    return result_points

# ---------------- 舵机循迹控制 ---------------- #
def trace_rectangle(corners: List[Tuple[float, float]], delay: float = 0.5):
    for idx, (x, y) in enumerate(corners):
        px, py = map_coords_to_pulses(x, y)
        print(f"Moving to corner {idx}: (X={x}, Y={y}) → (pulseX={px}, pulseY={py})")
        ServoControl.setPWMServoMove(1, px, 500)
        ServoControl.setPWMServoMove(2, py, 500)
        time.sleep(delay)

# ---------------- 主程序入口 ---------------- #
if __name__ == "__main__":
    corners = find_black_rectangle_corners(debug=True, save_path="debug_detected.jpg")
    if corners and len(corners) == 4:
        print("检测到的矩形坐标(0~100)：", corners)
        print("开始循迹...")
        trace_rectangle(corners, delay=1.0)  # 每个点停留1秒，可调
        print("循迹完成！")
    else:
        print("未检测到有效的矩形，无法循迹。")
