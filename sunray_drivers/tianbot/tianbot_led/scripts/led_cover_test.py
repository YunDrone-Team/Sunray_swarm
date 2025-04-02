#! /usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray
import math

def led_white_breath_publisher():
    rospy.init_node('leds_white_breath_publisher', anonymous=True)
    pub = rospy.Publisher('leds', Int32MultiArray, queue_size=10)
    rospy.loginfo("LEDs white breath effect started.")
    rate = rospy.Rate(30)  # 30 Hz for a smooth effect

    num_leds = 16  # Number of LEDs
    max_brightness = 255  # Maximum brightness value
    breath_period = 5.0  # Time in seconds for a full breath cycle

    # Function to calculate the brightness based on the sine wave
    def calculate_brightness(t):
        return int((math.sin(t * 2 * math.pi) + 1) / 2 * max_brightness)

    # Initialize LED values
    led_values = Int32MultiArray()
    led_values.data = [0] * num_leds * 3  # 3 values per LED (R, G, B)

    # Start time
    start_time = rospy.get_time()

    while not rospy.is_shutdown():
        # Calculate the current phase in the breath cycle
        t = (rospy.get_time() - start_time) % breath_period / breath_period
        current_brightness = calculate_brightness(t)

        # Update all LEDs to the current brightness
        for i in range(num_leds):
            led_values.data[i * 3:(i + 1) * 3] = [current_brightness] * 3

        rospy.loginfo(f"Publishing: {led_values.data}")
        pub.publish(led_values)
        rate.sleep()

if __name__ == '__main__':
    try:
        led_white_breath_publisher()
    except rospy.ROSInterruptException:
        pass

