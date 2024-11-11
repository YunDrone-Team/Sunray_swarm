#!/usr/bin/env python3
# coding:utf-8
from djitellopy import Tello
import time

# 创建 Tello 对象   
t = Tello()
t.connect()
t.takeoff()
# 打印电池电量
print(f'Battery: {t.get_battery()}%')
time.sleep(3)

# 绕圆飞行，使用 curve 命令。每段弧都需要起始点和终点，以及中间点。
# 假设绕圆的速度为 30 cm/s
try:
    # # 每个 curve 命令需要两个控制点和一个终点，且飞行半径需在 0.5m 到 5m 之间
    t.curve_xyz_speed(100, 0, 0, 0, 100, 0, 30)
    t.curve_xyz_speed(-100, 0, 0, 0, -100, 0, 30)
    time.sleep(3.0)
finally:
    t.land()

    