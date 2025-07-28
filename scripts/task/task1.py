#!/usr/bin/env python3
"""
task1.py ─ 读取 p1 并通过 UART0 (GPIO14/15 → /dev/serial0) @ 9600 baud 输出
依赖：pip install pyserial
"""

import ServoControl  # ← 引入 ServoControl 模块
from readparams import get_p1   # ← 直接复用公共模块
import time



def main() -> None:
    # 示例：控制舵机1转到1000脉宽，持续时间为2000毫秒
    ServoControl.setBusServoMove(1, 1000, 2000)
    ServoControl.setBusServoMove(2, 1000, 2000)
    time.sleep(2)  # 等待2秒
    ServoControl.setPWMServoMove(1, 1500, 2000)  # 控制PWM舵机1转到1500脉宽，持续时间为2000毫秒
    ServoControl.setPWMServoMove(2, 1500, 2000)  # 控制PWM舵机2转到1500脉宽，持续时间为2000毫秒
    time.sleep(2)  # 等待2秒

    print("Servo commands executed successfully.")

if __name__ == "__main__":
    main()
