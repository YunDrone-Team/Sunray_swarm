#!/usr/bin/env python3
# coding:utf-8
# import rospy

# if __name__ == '__main__':
#     # 创建节点
#     rospy.init_node("pyhello")
#     print("hello Irvingao! this is ur first ros python code! Good luck for u!")

import time
import robomaster
from robomaster import robot
import rospy


if __name__ == '__main__':

    rospy.init_node('mled')
    # pub = rospy.Publisher('/sunray_swarm/rmtt_1/mesh', queue_size=5)

    mled_smile1 = '000000000r0000r0r0r00r0r000000000000000000r00r00000rr00000000000'
    mled_smile2 = '00rrrr000r0000r0r0r00r0rr000000rr0r00r0rr00rr00r0r0000r000rrrr00'

    tl_drone = robot.Drone()
    tl_drone.initialize()

    # 显示自定义图案
    tl_drone.led.set_mled_graph(mled_smile1)
    time.sleep(3)
    tl_drone.led.set_mled_graph(mled_smile2)
    time.sleep(3)

    # 显示数字
    for num in range(10):
        tl_drone.led.set_mled_char('r', num)
        time.sleep(0.5)

    # 显示字符A, B, C
    tl_drone.led.set_mled_char(color='b', display_char='A')
    time.sleep(3)
    tl_drone.led.set_mled_char(color='b', display_char='B')
    time.sleep(3)
    tl_drone.led.set_mled_char(color='b', display_char='C')
    time.sleep(3)

    # 清屏
    tl_drone.led.set_mled_char('0')

    tl_drone.close()




