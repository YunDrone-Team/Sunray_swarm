#!/usr/bin/env python3
# coding:utf-8
from djitellopy import Tello
import time

t = Tello()
t.connect()
t.takeoff()
time.sleep(3)

try:
    # 方形走线，四次移动和旋转
    for _ in range(4):
        # 前进100厘米
        t.move_forward(100) 
        # 休眠3秒 
        time.sleep(3)
        # 顺时针旋转90度
        t.rotate_clockwise(90)  
        # 休眠1秒
        time.sleep(1)
finally:
    # 降落
    t.land()