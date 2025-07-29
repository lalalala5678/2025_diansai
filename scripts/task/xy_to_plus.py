# xy_to_plus.py – 将 0-100 逻辑坐标映射为两路 PWM 脉宽，并控制舵机.

"""
xy_to_plus.py – 将 0-100 逻辑坐标映射为两路 PWM 脉宽，并控制舵机。

说明:
- 通过读取同目录下的 plus.txt 文件获取四个标定参数 p1,p2,p3,p4，
  这些参数对应摄像头识别坐标系(0-100)的边界与舵机PWM脉宽的关系:
    * p1: X轴逻辑坐标100对应的舵机1脉宽（值为实际脉宽的1/5）
    * p2: X轴逻辑坐标 0对应的舵机1脉宽（值为实际脉宽的1/5）
    * p3: Y轴逻辑坐标100对应的舵机2脉宽（值为实际脉宽的1/5）
    * p4: Y轴逻辑坐标 0对应的舵机2脉宽（值为实际脉宽的1/5）
  注意: 逻辑坐标系假定 (0,0) 在摄像头画面一角（例如左下角），(100,100) 在对角（如右上角）。
  plus.txt 文件可用逗号或换行分隔键值对，例如:
      p1=287, p2=370, p3=260, p4=340

- 提供 set_xy(x, y, waiting_ms) 接口，将 0-100 范围内的逻辑坐标映射为舵机脉宽并移动舵机至该点。

- 映射采用线性插值，已考虑舵机旋转方向（通过标定得到的 p1..p4 值决定）。

- 在进行标定时，由于激光指示器固定在舵机上旋转，激光点位置与舵机脉宽可能存在非线性关系，
  此处按线性近似处理。建议通过多点标定或小范围校正以提高精度。
"""

import os
import time
import logging
import ServoControl   # 树莓派舵机控制库

# 读取 plus.txt 参数的缓存（首次读取后缓存以避免重复IO）
_PARAM_CACHE = None

def _read_plus_file():
    """读取 plus.txt 文件中的标定参数。如果文件不存在则使用默认值。"""
    global _PARAM_CACHE
    if _PARAM_CACHE is not None:
        return _PARAM_CACHE
    defaults = {"p1": 300, "p2": 400, "p3": 300, "p4": 400}  # 默认值兜底
    cfg_path = os.path.join(os.path.dirname(__file__), "plus.txt")
    if not os.path.isfile(cfg_path):
        logging.warning(f"'plus.txt' not found → 使用默认参数 {defaults}")
        _PARAM_CACHE = defaults
        return _PARAM_CACHE
    params = {}
    with open(cfg_path, "r", encoding="utf-8") as f:
        text = f.read().replace("\n", ",")  # 支持换行或逗号分隔
        for token in text.split(","):
            token = token.strip()
            if not token:
                continue
            try:
                key, val = token.split("=", 1)
                params[key.strip()] = int(val)
            except ValueError:
                logging.warning(f"忽略无法解析的配置项: '{token}'")
    # 填补缺失项
    for k, v in defaults.items():
        params.setdefault(k, v)
    _PARAM_CACHE = params
    logging.info(f"加载 plus 参数: {_PARAM_CACHE}")
    return _PARAM_CACHE

# 提供获取单个参数的接口
def get_p1(): return _read_plus_file()["p1"]
def get_p2(): return _read_plus_file()["p2"]
def get_p3(): return _read_plus_file()["p3"]
def get_p4(): return _read_plus_file()["p4"]

# 坐标范围常量
X_MIN, X_MAX = 0.0, 100.0
Y_MIN, Y_MAX = 0.0, 100.0

def map_coords_to_pulses(x: float, y: float) -> tuple[int, int]:
    """
    将0-100的逻辑坐标 (x, y) 映射为舵机1和舵机2的PWM脉宽值（整数）。
    脉宽端点从 plus.txt 参数读取（p1..p4，并乘以5得到实际脉宽），
    然后对输入坐标进行线性插值得到对应脉宽。

    返回: (pulse_x, pulse_y)
    """
    params = _read_plus_file()
    p1_val, p2_val = params["p1"], params["p2"]
    p3_val, p4_val = params["p3"], params["p4"]
    # 计算脉宽端点值（plus参数乘以5倍）
    pulse_x_min = p1_val * 5  # 对应 x = X_MAX (100)
    pulse_x_max = p2_val * 5  # 对应 x = X_MIN (0)
    pulse_y_min = p3_val * 5  # 对应 y = Y_MAX (100)
    pulse_y_max = p4_val * 5  # 对应 y = Y_MIN (0)
    # 内部线性映射函数
    def _map(val, vmin, vmax, pmin, pmax):
        if vmax == vmin:
            return (pmin + pmax) // 2
        # 线性插值计算
        ratio = (vmax - val) / (vmax - vmin)
        return int(round(pmin + ratio * (pmax - pmin)))
    pulse_x = _map(x, X_MIN, X_MAX, pulse_x_min, pulse_x_max)
    pulse_y = _map(y, Y_MIN, Y_MAX, pulse_y_min, pulse_y_max)
    return pulse_x, pulse_y

def set_xy(x: float, y: float, waiting_ms: int = 2000) -> tuple[int, int]:
    """
    控制舵机将逻辑坐标 (x, y) 移动到对应位置。

    参数:
      x, y: 目标逻辑坐标 (0-100)
      waiting_ms: 等待舵机运动的时间（毫秒）

    返回:
      实际下发的 (pulse_x, pulse_y) 脉宽值元组。
    """
    if not (X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX):
        raise ValueError(f"x, y 必须在 [{X_MIN}, {X_MAX}] × [{Y_MIN}, {Y_MAX}] 范围内")
    pulse_x, pulse_y = map_coords_to_pulses(x, y)
    # 发送PWM控制信号
    ServoControl.setPWMServoMove(1, pulse_x, waiting_ms)
    ServoControl.setPWMServoMove(2, pulse_y, waiting_ms)
    # 等待舵机运动完成
    time.sleep(waiting_ms / 1000.0)
    return pulse_x, pulse_y

# 自测代码
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    # 输出当前加载的plus参数（若无则使用默认值）
    print("Current plus parameters:", _read_plus_file())
    # 测试将舵机移动到四个角落 (可能为视野极限位置)，打印对应脉宽
    test_points = [(0, 0), (100, 0), (100, 100), (0, 100)]
    try:
        while True:
            for (x, y) in test_points:
                px, py = set_xy(x, y, waiting_ms=1500)
                print(f"(x={x:.1f}, y={y:.1f}) -> pulse1={px}, pulse2={py}")
    except KeyboardInterrupt:
        print("\nStopped.")
