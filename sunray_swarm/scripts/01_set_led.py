#!/usr/bin/env python3
# coding:utf-8

# 识别挑战卡
from djitellopy import Tello  # 导入Tello控制库
import time  # 导入time库，用于时间控制

# 初始化无人机
tello = Tello()
tello.connect()  # 连接无人机
print(f"Battery level: {tello.get_battery()}%")  # 输出电池电量

# 进入SDK模式
tello.enable_mission_pads()  # 启用任务卡识别功能
tello.set_mission_pad_detection_direction(0)  # 设置任务卡检测方向为前视（0: 关闭, 1: 下视, 2: 前视）

print("已启用飞行地图任务卡功能")

# 起飞
tello.takeoff()  # 无人机起飞
time.sleep(3)  # 等待3秒

# 检测任务卡并读取地图信息
try:
    for _ in range(5):  # 尝试多次检测任务卡
        pad_id = tello.get_mission_pad_id()  # 获取当前检测到的任务卡ID
        if pad_id != -1:  # 检测到任务卡
            print(f"检测到任务卡ID: {pad_id}")
            # 获取任务卡的相对坐标
            x = tello.get_mission_pad_distance_x()  # 获取X方向距离
            y = tello.get_mission_pad_distance_y()  # 获取Y方向距离
            z = tello.get_mission_pad_distance_z()  # 获取Z方向距离
            print(f"任务卡坐标: x={x}cm, y={y}cm, z={z}cm")

            # 控制无人机移动到新位置（例如：向前飞30厘米）
            tello.move_forward(30)
            time.sleep(2)

        else:
            print("未检测到任务卡，请检查地图是否放置正确或调整无人机位置")
            time.sleep(1)
except Exception as e:
    print(f"发生错误: {e}")

# 降落并退出
tello.land()
tello.disable_mission_pads()  # 关闭任务卡检测功能
tello.end()
	




