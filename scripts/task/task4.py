#!/usr/bin/env python3
"""task4.py ─ 输出 p4 到 UART0 (GPIO14/15) @ 9600 baud"""
import serial
from readparams import get_p4

PORT = "/dev/serial0"
BAUD = 9600

def main() -> None:
    val = get_p4()
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        ser.write(f"{val}\n".encode())
    print(f"Sent p4={val} → {PORT}@{BAUD}")

if __name__ == "__main__":
    main()
