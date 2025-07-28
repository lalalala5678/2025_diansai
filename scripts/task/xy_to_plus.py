#!/usr/bin/env python3
from readparams import get_p1, get_p2, get_p3, get_p4

# 坐标范围硬编码
X_MIN, X_MAX = 0.0, 100.0
Y_MIN, Y_MAX = 0.0, 100.0

def map_coords_to_pulses(x: float, y: float) -> tuple[int, int]:
    """
    将 (x, y) 映射为两路 PWM 舵机脉宽：
      • 舵机1: pulse_min = get_p1()*5, pulse_max = get_p2()*5  
      • 舵机2: pulse_min = get_p3()*5, pulse_max = get_p4()*5  

    仅需传入 x, y，返回 (pulse_x, pulse_y)。
    """
    # 从 params.txt 读取并放大 5 倍
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

# — 使用示例 —
if __name__ == "__main__":
    import time, ServoControl

    # 目标坐标
    x, y = 30.0, 50.0
    px, py = map_coords_to_pulses(x, y)

    ServoControl.setPWMServoMove(1, px, 500)
    ServoControl.setPWMServoMove(2, py, 500)
    time.sleep(0.5)

    print(f"(x={x}, y={y}) → (pulse1={px}, pulse2={py})")
