#!/usr/bin/env python3
# move_by_status.py
"""
根据 task_status.txt 中的值依次移动到四个坐标：
0 → (0, 0)       → 状态置 1
1 → (0, 100)     → 状态置 2
2 → (100, 100)   → 状态置 3
3 → (100, 0)     → 状态置 0

依赖：
    - xy_to_plus.py  (同目录)
    - ServoControl.py (提供 setPWMServoMove)
    - task_status.txt (若不存在将自动创建并置 0)
"""

import os
import time
from xy_to_plus import map_coords_to_pulses
import ServoControl  # 假设提供 setPWMServoMove(channel, pulse, duration_ms)

# ------------------ 配置 ------------------ #
STATUS_FILE = os.path.join(os.path.dirname(__file__), "task_status.txt")
POINTS = [               # 依次要走的 4 个点
    (0, 0),
    (0, 100),
    (100, 100),
    (100, 0),
]
MOVE_DURATION_MS = 1500  # 舵机移动时间（毫秒）
# ------------------------------------------ #


def read_status() -> int:
    """读取当前状态（0‒3），若文件缺失/损坏则默认 0"""
    try:
        with open(STATUS_FILE, "r") as f:
            return int(f.read().strip()) % 4
    except (FileNotFoundError, ValueError):
        return 0


def write_status(val: int) -> None:
    """写入新状态"""
    with open(STATUS_FILE, "w") as f:
        f.write(str(val))


def move_to_xy(x: float, y: float, duration_ms: int = MOVE_DURATION_MS) -> None:
    """调用 xy_to_plus → ServoControl，把两路舵机移动到指定坐标"""
    pulse_x, pulse_y = map_coords_to_pulses(x, y)
    ServoControl.setPWMServoMove(1, pulse_x, duration_ms)
    ServoControl.setPWMServoMove(2, pulse_y, duration_ms)
    time.sleep(duration_ms / 1000)  # 等待动作完成


def main() -> None:
    status = read_status()               # 0‒3
    target_x, target_y = POINTS[status]  # 取本轮目标点

    print(f"Current status: {status} → move to ({target_x}, {target_y})")
    move_to_xy(target_x, target_y)

    next_status = (status + 1) % 4       # 环状更新
    write_status(next_status)
    print(f"New status written: {next_status}")


if __name__ == "__main__":
    main()
