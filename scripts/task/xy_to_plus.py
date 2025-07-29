#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
xy_to_plus.py - 将逻辑坐标 (x, y) 映射为舵机PWM脉宽 (pulse1, pulse2)，考虑激光偏心和投影几何。
配置参数从 plus.txt 加载，包括:
    ox, oy: 逻辑坐标原点偏移 (浮点数，可正可负)
    angle_h, angle_v: 水平/垂直最大扫描角度 (度数，浮点数)
    pulse1_min, pulse1_max, pulse2_min, pulse2_max: 两个舵机通道对应的最小/最大脉宽值 (整数或浮点)
提供函数:
    map_coords_to_pulses(x, y) -> (pulse1, pulse2)
    set_xy(x, y) -> None (移动舵机到指定逻辑坐标)
"""

import math
import logging

# 配置日志输出到文件
logging.basicConfig(filename='xy_to_plus.log', level=logging.INFO,
                    format='%(asctime)s [xy_to_plus] %(levelname)s: %(message)s')

# 从配置文件读取参数
config_file = "plus.txt"
params = {}
try:
    with open(config_file, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            # 支持 "key=value" 格式
            if '=' in line:
                key, val = line.split('=', 1)
                params[key.strip()] = val.strip()
except FileNotFoundError:
    logging.error(f"配置文件 {config_file} 未找到！请先运行标定脚本生成该文件。")
    raise

# 将参数转换为相应类型
try:
    ox = float(params.get("ox", 0.0))
    oy = float(params.get("oy", 0.0))
    angle_h = float(params.get("angle_h", 0.0))    # 水平总扫描角度（度）
    angle_v = float(params.get("angle_v", 0.0))    # 垂直总扫描角度（度）
    pulse1_min = float(params.get("pulse1_min", 0))  # 允许浮点但通常为整数
    pulse1_max = float(params.get("pulse1_max", 0))
    pulse2_min = float(params.get("pulse2_min", 0))
    pulse2_max = float(params.get("pulse2_max", 0))
except Exception as e:
    logging.error("配置参数解析错误: %s", e)
    raise

# 为方便，计算一些中间值
# 舵机脉宽中心值 (假设线性近似下为min和max的中点)
pulse1_center = (pulse1_min + pulse1_max) / 2.0
pulse2_center = (pulse2_min + pulse2_max) / 2.0

# 将角度范围转换为弧度在计算中使用
half_angle_h_rad = math.radians(angle_h / 2.0)
half_angle_v_rad = math.radians(angle_v / 2.0)

def map_coords_to_pulses(x, y):
    """
    将逻辑坐标 (x, y) 转换为两个舵机的PWM脉宽 (pulse1, pulse2)。
    输入x, y取值0~100的浮点数。返回pulse1, pulse2为float（可取整后发送）。
    该映射考虑激光非居中和透视投影引起的非线性:
      - 使用配置的原点偏移ox, oy校正激光指向中心的偏差。
      - 使用反正切(arctan)关系将逻辑坐标线性映射到舵机旋转角度，再换算为脉宽。
    """
    # 正常化逻辑坐标到 -1.0 ~ 1.0 范围，考虑原点偏移
    # 原点偏移ox, oy用于校准舵机中心与逻辑坐标中心(50)的不一致
    x_norm = (x - 50.0 - ox) / 50.0  # 水平归一化坐标
    y_norm = (y - 50.0 - oy) / 50.0  # 垂直归一化坐标

    # 基于几何模型计算对应舵机角度（相对于中心的偏移角度）
    # 利用反正切和切线，以处理投影面的非线性: angle = arctan(x_norm * tan(FOV/2))
    # 其中FOV/2为最大扫描角的一半。
    angle_x_rad = math.atan(x_norm * math.tan(half_angle_h_rad))
    angle_y_rad = math.atan(y_norm * math.tan(half_angle_v_rad))
    # 转回角度值（度）
    angle_x_deg = math.degrees(angle_x_rad)
    angle_y_deg = math.degrees(angle_y_rad)

    # 将舵机角度映射为脉宽：假设舵机脉宽与物理角度近似线性相关:contentReference[oaicite:0]{index=0}。
    # 当angle_x_deg = -angle_h/2时对应pulse1_min， angle_x_deg = +angle_h/2对应pulse1_max
    # 线性内插计算pulse1, pulse2
    fraction_x = (angle_x_deg + (angle_h / 2.0)) / angle_h  # 将[-angle_h/2, +angle_h/2]映射到[0,1]
    fraction_y = (angle_y_deg + (angle_v / 2.0)) / angle_v  # 将[-angle_v/2, +angle_v/2]映射到[0,1]

    pulse1 = pulse1_min + fraction_x * (pulse1_max - pulse1_min)
    pulse2 = pulse2_min + fraction_y * (pulse2_max - pulse2_min)

    # 限制脉宽在[min, max]范围内，避免因计算误差超出范围
    if pulse1 < pulse1_min: pulse1 = pulse1_min
    if pulse1 > pulse1_max: pulse1 = pulse1_max
    if pulse2 < pulse2_min: pulse2 = pulse2_min
    if pulse2 > pulse2_max: pulse2 = pulse2_max

    logging.debug(f"map_coords_to_pulses: input=({x:.2f},{y:.2f}), norm=({x_norm:.3f},{y_norm:.3f}), "
                  f"angle=({angle_x_deg:.2f}°, {angle_y_deg:.2f}°), pulses=({pulse1:.1f}, {pulse2:.1f})")
    return pulse1, pulse2

def set_xy(x, y):
    """
    移动激光指向逻辑坐标(x, y)。会计算相应舵机脉宽并控制舵机。
    """
    p1, p2 = map_coords_to_pulses(x, y)
    # 调用具体的舵机控制接口
    try:
        # 假定ServoControl.setPWMServoMove(channel, pulse)为已有接口，示例调用两路舵机
        ServoControl.setPWMServoMove(1, int(p1))
        ServoControl.setPWMServoMove(2, int(p2))
        logging.info(f"set_xy: Moved to ({x:.1f}, {y:.1f}) -> pulses ({p1:.1f}, {p2:.1f})")
    except Exception as e:
        logging.error(f"set_xy: 控制舵机移动失败: {e}")
        # 根据实际需要，可以在此进行错误处理，例如重试或安全位置复位
        raise

# 如果直接运行脚本，做一个简单的测试或演示 (可选)
if __name__ == "__main__":
    # 示例: 移动到中心和四角
    for test in [(50, 50), (0, 0), (100, 0), (0, 100), (100, 100)]:
        try:
            set_xy(test[0], test[1])
        except NameError:
            # 如果没有实际ServoControl库，用日志模拟输出
            p1, p2 = map_coords_to_pulses(test[0], test[1])
            logging.info(f"[Demo] Target {test} -> pulses ({p1:.1f}, {p2:.1f})")
