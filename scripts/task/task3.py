#!/usr/bin/env python3
from picamera2 import Picamera2
import time
import cv2

def capture_crop(center: tuple[int,int], width: int, height: int, output: str = "cropped.jpg"):
    """
    拍摄一张照片并根据 (center, width, height) 裁剪后保存。
    center: (x, y) 中心点像素坐标
    width, height: 裁剪框的宽、高（像素）
    """
    picam = Picamera2()
    picam.configure(picam.create_still_configuration())
    picam.start()
    time.sleep(2)                  # 预热
    img = picam.capture_array()
    cv2.imwrite("original.jpg", img)  # 保存原图以便调试
    picam.close()

    x, y = center
    print(img.shape)  # 输出图像尺寸 (height, width, channels)
    w2, h2 = width // 2, height // 2
    x1, x2 = max(0, x - w2), min(img.shape[1], x + w2)
    y1, y2 = max(0, y - h2), min(img.shape[0], y + h2)

    cv2.imwrite(output, img[y1:y2, x1:x2])
    print(f"Saved {output}")

if __name__ == "__main__":

    # image尺寸为(2592,4608,3)

    # center = (x_center, y_center)
    center = (4608//2, 2592//2)        # → (2304, 1296)
    size   = 2592                      # 正方形边长 = min(width, height)
    capture_crop(center, size, size, "cropped.jpg")

