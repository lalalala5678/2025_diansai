#!/usr/bin/env python3
"""
task1.py ─ 读取 p1 并通过 UART0 (GPIO14/15 → /dev/serial0) @ 9600 baud 输出
依赖：pip install pyserial
"""

import serial
from pathlib import Path
from readparams import get_p1   # ← 直接复用公共模块

PORT  = "/dev/serial0"   # GPIO14=TX, GPIO15=RX
BAUD  = 9600

def main() -> None:
    p1 = get_p1()                       # 取参数
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        ser.write(f"{p1}\n".encode())   # 发送
    print(f"Sent p1={p1} → {PORT}@{BAUD}")

if __name__ == "__main__":
    main()
