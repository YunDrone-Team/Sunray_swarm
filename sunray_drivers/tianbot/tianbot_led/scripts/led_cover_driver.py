#! /usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray
import serial

# 设置串口
ser = serial.Serial('/dev/ttyUSB0', 115200)  # 请根据实际情况修改串口名称和波特率
#topic_name=rospy.resolve_name('leds')
def led_callback(data):
    led_values = data.data
    for i in range(16):
        index = i
        r = led_values[i * 3]
        g = led_values[i * 3 + 1]
        b = led_values[i * 3 + 2]
        message = f"l {index} {r} {g} {b} 0\n"
        ser.write(message.encode('utf-8'))
        # rospy.loginfo(f"Sent: {message}")
    message = f"l {index} {r} {g} {b} 1\n"
    ser.write(message.encode('utf-8'))

def listener():
    rospy.init_node('leds_controller', anonymous=True)
    rospy.Subscriber('leds', Int32MultiArray, led_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    finally:
        ser.close()