#!/usr/bin/env python3
# coding:utf-8
from djitellopy import Tello
import time

# 定义一个对象
t=Tello()
# 连接tello
t.connect()
# 起飞
t.takeoff()
# 等待3秒
time.sleep(3)
# 获取电池电量
response = t.get_battery()
# 打印
print(response)
# 等待3秒
time.sleep(3)
# 前进100cm
t.move_forward(100)
# 降落
t.land()



