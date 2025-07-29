#!/usr/bin/env python3
from readparams import get_p1, get_p2, get_p3, get_p4
import ServoControl  # 按需导入，避免非树莓派环境报错
import time

# 坐标范围
X_MIN, X_MAX = 0.0, 100.0
Y_MIN, Y_MAX = 0.0, 100.0

def map_coords_to_pulses(x: float, y: float) -> tuple[int, int]:
    """（保持原逻辑，不做任何修改）"""
    p1_min = get_p1() * 5
    p1_max = get_p2() * 5
    p2_min = get_p3() * 5
    p2_max = get_p4() * 5

    def _map(v, vmin, vmax, pmin, pmax):
        if vmax == vmin:
            return (pmin + pmax) // 2
        ratio = (vmax - v) / (vmax - vmin)
        return int(round(pmin + ratio * (pmax - pmin)))

    pulse_x = _map(x, X_MIN, X_MAX, p1_min, p1_max)
    pulse_y = _map(y, Y_MIN, Y_MAX, p2_min, p2_max)
    return pulse_x, pulse_y


# ------------------------------------------------------------------
# 新增的简易控制封装
# ------------------------------------------------------------------
def set_xy(x: float, y: float, waiting_ms: int = 2000) -> tuple[int, int]:
    """
    把 0-100 坐标 (x, y) 直接发送给两路舵机，
    并在指令下发后等待 `waiting_ms` 毫秒。
    返回实际下发的 (pulse_x, pulse_y) 供调试打印。
    """
    if not (X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX):
        raise ValueError(
            f"x, y 必须分别位于 [{X_MIN}, {X_MAX}] × [{Y_MIN}, {Y_MAX}]"
        )

    pulse_x, pulse_y = map_coords_to_pulses(x, y)


    ServoControl.setPWMServoMove(1, pulse_x, waiting_ms)
    ServoControl.setPWMServoMove(2, pulse_y, waiting_ms)

    # 等待舵机到位

    time.sleep(waiting_ms / 1000.0)

    return pulse_x, pulse_y


# ------------------------------------------------------------------
# 简单测试
# ------------------------------------------------------------------
if __name__ == "__main__":
    # 测试点：左下角、中心、右上角
    while True:

        test_points = [(100, 0), (100, 50), (100, 100)]

        for (x, y) in test_points:
            px, py = set_xy(x, y, waiting_ms=1500)
            print(f"(x={x:.1f}, y={y:.1f})  →  pulse1={px}, pulse2={py}")
