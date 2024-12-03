#!/usr/bin/env python3
# coding:utf-8
from djitellopy import Tello  # 导入Tello控制库
import time  # 导入time库，用于时间控制
import math  # 导入数学库，用于计算圆形坐标

# 初始化无人机
tello = Tello()
tello.connect()  # 连接无人机
print(f"Battery level: {tello.get_battery()}%")  # 输出电池电量

# 启用任务卡功能
tello.enable_mission_pads()  # 启用任务卡检测
tello.set_mission_pad_detection_direction(0)  # 设置任务卡检测方向为下视（1: 下视）

# 起飞
tello.takeoff()
time.sleep(2)



# try:
#     for _ in range(5):
#         pad_id = tello.get_mission_pad_id()  # 获取当前检测到的任务卡ID
#         if pad_id == -1:  # 检测到任务卡
#             print("未检测到任务卡")
#         else:  # 检测到任务卡
#             print(f"检测到任务卡{pad_id}")
# finally:
#     tello.land()
#     tello.disable_mission_pads()
#     tello.end()


# # 设置圆形飞行参数
radius = 50  # 圆形半径（单位：cm）
speed = 20   # 飞行速度（单位：cm/s）
center_mid = 12  # 假设地图中心点的任务卡ID为1
steps = 2  # 将圆分为36段（即每10度飞行一次）

# 确保任务卡检测到中心点
if tello.get_mission_pad_id() == center_mid:
    print(f"已检测到中心点任务卡ID: {center_mid}")

    # 计算圆形路径并逐步飞行
    for i in range(steps):
        angle = (2 * math.pi / steps) * i  # 当前角度（弧度）
        x = int(radius * math.cos(angle))  # X坐标
        y = int(radius * math.sin(angle))  # Y坐标
        z = 100  # 保持高度不变

        # 发送飞行指令到(x, y, z)
        print(f"飞往点: x={x}, y={y}, z={z}")
        tello.go_xyz_speed_mid(x, y, z, speed, center_mid)
        time.sleep(1)  # 等待飞行完成

else:
    print("未检测到中心点任务卡，请调整无人机位置或地图设置")

# 降落
tello.land()

# 关闭任务卡功能
tello.disable_mission_pads()
tello.end()