#!/usr/bin/env python3
# coding:utf-8


import time
from robomaster import robot
from robomaster import led


if __name__ == '__main__':
    robot_sn_list = ["0TQZM47CNT046P"]
    robot.config.DEFAULT_PROTO_TYPE = '8889'
    robot.config.LOCAL_IP_STR = "192.168.25.59"
    robot.config.ROBOT_DEFAULT_WIFI_ADDR = ('192.168.25.73', 8889)
    robot.config.DEFAULT_CONN_PROTO = "udp"
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_led = ep_robot.led

    # 设置灯效为常亮，亮度递增
    bright = 1
    for i in range(0, 8):
        ep_led.set_led(comp=led.COMP_ALL, r=bright << i, g=bright << i, b=bright << i, effect=led.EFFECT_ON)
        time.sleep(1)
        print("brightness: {0}".format(bright << i))

    ep_robot.close()


