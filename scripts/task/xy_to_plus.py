#!/usr/bin/env python3
"""
xy_to_plus.py – 将 0-100 逻辑坐标映射为两路 PWM 脉宽，并控制舵机。

1. 在同一目录放置  plus.txt   （逗号或换行均可）：
       p1=287,p2=370,p3=260,p4=340
2. 调用 set_xy(x, y, waiting_ms) 即可移动舵机。
"""

import os
import time
import logging
import ServoControl   # 树莓派舵机控制库

# ───────── 读取 plus.txt ──────────
_PARAM_CACHE = None        # 第一次读取后缓存

def _read_plus_file():
    global _PARAM_CACHE
    if _PARAM_CACHE is not None:
        return _PARAM_CACHE

    defaults = dict(p1=300, p2=400, p3=300, p4=400)   # 兜底
    cfg_path = os.path.join(os.path.dirname(__file__), "plus.txt")
    if not os.path.isfile(cfg_path):
        logging.warning(f"'plus.txt' not found → using defaults {defaults}")
        _PARAM_CACHE = defaults
        return _PARAM_CACHE

    params = {}
    with open(cfg_path, "r", encoding="utf-8") as f:
        text = f.read().replace("\n", ",")            # 支持换行或逗号分隔
        for token in text.split(","):
            token = token.strip()
            if not token:
                continue
            try:
                key, val = token.split("=", 1)
                params[key.strip()] = int(val)
            except ValueError:
                logging.warning(f"Ignoring malformed token '{token}' in plus.txt")

    # 填补缺失项
    for k in ("p1", "p2", "p3", "p4"):
        params.setdefault(k, defaults[k])

    _PARAM_CACHE = params
    logging.info(f"Loaded plus parameters: {_PARAM_CACHE}")
    return _PARAM_CACHE


def get_p1(): return _read_plus_file()["p1"]
def get_p2(): return _read_plus_file()["p2"]
def get_p3(): return _read_plus_file()["p3"]
def get_p4(): return _read_plus_file()["p4"]

# ───────── 坐标与舵机脉宽映射 ─────────
X_MIN, X_MAX = 0.0, 100.0
Y_MIN, Y_MAX = 0.0, 100.0

def map_coords_to_pulses(x: float, y: float) -> tuple[int, int]:
    """保持与原逻辑一致，只是脉宽端点改为来自 plus.txt ×5 倍放大。"""
    p1_min, p1_max = get_p1() * 5, get_p2() * 5
    p2_min, p2_max = get_p3() * 5, get_p4() * 5

    def _map(v, vmin, vmax, pmin, pmax):
        if vmax == vmin:
            return (pmin + pmax) // 2
        ratio = (vmax - v) / (vmax - vmin)   # 方向依题目要求不变
        return int(round(pmin + ratio * (pmax - pmin)))

    return _map(x, X_MIN, X_MAX, p1_min, p1_max), \
           _map(y, Y_MIN, Y_MAX, p2_min, p2_max)

# ───────── 对外控制接口 ─────────
def set_xy(x: float, y: float, waiting_ms: int = 2000) -> tuple[int, int]:
    """
    把 0-100 坐标 (x, y) 发送给两路舵机，等待 waiting_ms 毫秒。
    返回实际下发的 (pulse_x, pulse_y)。
    """
    if not (X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX):
        raise ValueError(f"x, y 必须在 [{X_MIN}, {X_MAX}] × [{Y_MIN}, {Y_MAX}]")

    pulse_x, pulse_y = map_coords_to_pulses(x, y)
    ServoControl.setPWMServoMove(1, pulse_x, waiting_ms)
    ServoControl.setPWMServoMove(2, pulse_y, waiting_ms)
    time.sleep(waiting_ms / 1000.0)
    return pulse_x, pulse_y

# ───────── 自测 ─────────
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    test_points = [(0, 0),(100, 0), (100, 100), (0, 100),]
    while True:
        for x, y in test_points:
            px, py = set_xy(x, y, waiting_ms=1500)
            print(f"(x={x:.1f}, y={y:.1f}) → pulse1={px}, pulse2={py}")
