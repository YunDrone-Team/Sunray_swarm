#!/usr/bin/env python3
# coding:utf-8
# import pygame
# from djitellopy import Tello
# import sys

# # 初始化 pygame
# pygame.init()

# # 设置窗口
# window = pygame.display.set_mode((400, 400))

# # 初始化 Tello
# t = Tello()
# t.connect()
# print(f"Battery level: {t.get_battery()}%")

# # 无人机控制函数
# def handle_input(key):
#     if key == pygame.K_UP:  # 向前
#         t.move_forward(30)
#     elif key == pygame.K_DOWN:  # 向后
#         t.move_back(30)
#     elif key == pygame.K_LEFT:  # 向左
#         t.move_left(30)
#     elif key == pygame.K_RIGHT:  # 向右
#         t.move_right(30)
#     elif key == pygame.K_w:  # 向上
#         t.move_up(30)
#     elif key == pygame.K_x:  # 向下
#         t.move_down(30)
#     elif key == pygame.K_SPACE:  # 起飞
#         t.takeoff()
#     elif key == pygame.K_l:  # 降落
#         t.land()

# # 主循环
# running = True
# while running: 
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False
#         elif event.type == pygame.KEYDOWN:
#             handle_input(event.key)

# # 清理并退出
# t.land()  # 确保无人机降落
# t.end()
# pygame.quit() 
# sys.exit()


import pygame  # 导入pygame库，用于捕获键盘输入
from djitellopy import Tello  # 从djitellopy库中导入Tello类，用于控制Tello无人机
import sys  # 导入sys库，用于系统退出

# 初始化 pygame
pygame.init()  # 初始化pygame模块

# 设置窗口
window = pygame.display.set_mode((400, 400))  # 创建一个400x400像素的窗口，用于显示界面

# 初始化 Tello
t = Tello()  # 创建Tello对象，实例化无人机
t.connect()  # 连接无人机
print(f"Battery level: {t.get_battery()}%")  # 输出电池电量

# 无人机控制函数
def handle_input(key):  # 定义控制无人机的函数，根据按下的键触发不同的动作
    if key == pygame.K_UP:  # 如果按下“向上箭头”键
        t.move_forward(30)  # 无人机向前移动30厘米
    elif key == pygame.K_DOWN:  # 如果按下“向下箭头”键
        t.move_back(30)  # 无人机向后移动30厘米
    elif key == pygame.K_LEFT:  # 如果按下“向左箭头”键
        t.move_left(30)  # 无人机向左移动30厘米
    elif key == pygame.K_RIGHT:  # 如果按下“向右箭头”键
        t.move_right(30)  # 无人机向右移动30厘米
    elif key == pygame.K_w:  # 如果按下“W”键
        t.move_up(30)  # 无人机向上移动30厘米
    elif key == pygame.K_x:  # 如果按下“X”键
        t.move_down(30)  # 无人机向下移动30厘米
    elif key == pygame.K_SPACE:  # 如果按下“空格”键
        t.takeoff()  # 无人机起飞
    elif key == pygame.K_l:  # 如果按下“L”键
        t.land()  # 无人机降落

# 主循环
running = True  # 设置程序运行标志
while running:  # 主循环，持续检测事件
    for event in pygame.event.get():  # 遍历所有pygame事件
        if event.type == pygame.QUIT:  # 如果窗口关闭事件触发
            running = False  # 结束主循环
        elif event.type == pygame.KEYDOWN:  # 如果按下键盘上的任意键
            handle_input(event.key)  # 调用handle_input函数并传入按键事件

# 清理并退出
t.land()  # 确保无人机降落
t.end()  # 断开无人机连接
pygame.quit()  # 退出pygame
sys.exit()  # 退出程序

    