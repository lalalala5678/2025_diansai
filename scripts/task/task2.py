#!/usr/bin/env python3
"""
task2.py ─ 读取 p1 并通过 UART0 (GPIO14/15 → /dev/serial0) @ 9600 baud 输出
依赖：pip install pyserial
"""

import ServoControl  # ← 引入 ServoControl 模块
from readparams import get_p1   # ← 直接复用公共模块
import time



def main() -> None:


    ServoControl.setPWMServoMove(1, 1710, 2000)  
    ServoControl.setPWMServoMove(2, 1325, 2000)  
    time.sleep(2)  # 等待2秒

    ServoControl.setPWMServoMove(1, 1440, 2000)  
    ServoControl.setPWMServoMove(2, 1337, 2000)  
    time.sleep(2)  # 等待2秒

    ServoControl.setPWMServoMove(1, 1430, 2000)  
    ServoControl.setPWMServoMove(2, 1490, 2000)  
    time.sleep(2)  # 等待2秒

    ServoControl.setPWMServoMove(1, 1730, 2000)  
    ServoControl.setPWMServoMove(2, 1495, 2000)  
    time.sleep(2)  # 等待2秒


    print("first servo commands executed successfully.")


    print("Servo commands executed successfully.")

if __name__ == "__main__":
    main()
