#!/usr/bin/env python3
# coding:utf-8

import pygame  # 用于检测键盘输入
from djitellopy import Tello

# 初始化无人机
tello = Tello()
tello.connect()
print(f"电池电量: {tello.get_battery()}%")

# 初始化pygame
pygame.init()
screen = pygame.display.set_mode((400, 300))  # 创建一个窗口以捕获键盘事件
pygame.display.set_caption("按键触发MLED图案显示")

# 定义5种图案的点阵字符串
patterns = {
    "smile_face": "000000000rr00rr00rr00rr0000000000r0000r000rrrr0000000",
    "sad_face": "000000000bb00bb00bb00bb00000000000bbbb000b0000b0000000",
    "heart": "000000000pp00pp0pppppppppppppppp0pppppp000pppp00000pp000000000",
    "arrow": "000bb00000bbbb000bbbbbb000bbbb00000bb000000bb000000bb00000000",
    "star": "0r0000r000r00r00ppppppppp000000pp0p00p0pp000000ppppppppp00000000"
}

def display_pattern(pattern_name, pattern):
    """
    显示MLED图案
    :param pattern_name: 图案名称
    :param pattern: 点阵字符串
    """
    print(f"正在显示图案: {pattern_name}")
    tello.send_command_with_return(f"EXT mled g {pattern}")

# 主程序循环
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:  # 如果点击关闭窗口
            running = False
        
        if event.type == pygame.KEYDOWN:  # 检测键盘按键
            if event.key == pygame.K_a:  # 按下 "a" 键，显示笑脸
                display_pattern("smile_face", patterns["star"])
            elif event.key == pygame.K_b:  # 按下 "b" 键，显示哭脸
                display_pattern("sad_face", patterns["sad_face"])
            elif event.key == pygame.K_c:  # 按下 "c" 键，显示心形
                display_pattern("heart", patterns["heart"])
            elif event.key == pygame.K_d:  # 按下 "d" 键，显示箭头
                display_pattern("arrow", patterns["arrow"])
            elif event.key == pygame.K_e:  # 按下 "e" 键，显示星形
                display_pattern("star", patterns["smile_face"])

# 关闭pygame和无人机连接
pygame.quit()
tello.end()