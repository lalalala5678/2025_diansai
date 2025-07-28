#!/usr/bin/env python3
"""
遍历固定四点并驱动舵机。
"""
from xy_to_plus import map_coords_to_pulses   # ← 关键：复用现有映射函数
import ServoControl                           # ← 你的舵机控制库
import time

# ---------- 1) 硬编码四点边界 ----------
px1, px2 = 26, 70   # x 轴最小、最大
py1, py2 = 25, 92   # y 轴最小、最大

# 以顺时针方式组合四个顶点
POINTS = [
    (px1, py1),
    (px1, py2),
    (px2, py2),
    (px2, py1),
]

MOVE_MS = 1300      # 每次移动时长（毫秒）


def move_to_xy(x: float, y: float, duration_ms: int = MOVE_MS) -> None:
    """把 (x,y) 坐标映射为脉宽，并驱动两路舵机"""
    pulse_x, pulse_y = map_coords_to_pulses(x, y)
    ServoControl.setPWMServoMove(1, pulse_x, duration_ms)
    ServoControl.setPWMServoMove(2, pulse_y, duration_ms)
    time.sleep(duration_ms / 1000)  # 等待动作完成


def main() -> None:
    for x, y in POINTS:
        print(f"→ Move to ({x}, {y})")
        move_to_xy(x, y)


if __name__ == "__main__":
    main()
