#!/bin/bash

# 编译rmtt_msgs模块
catkin_make --source sunray_swarm_msgs --build build/sunray_swarm_msgs
# 编译oradar_ros模块
catkin_make --source sunray_drivers/oradar_ros --build build/oradar_ros
# 编译rmtt_driver模块
catkin_make --source sunray_drivers/rmtt/rmtt_driver --build build/rmtt_driver
# 编译rmtt_description模块
catkin_make --source sunray_drivers/rmtt/rmtt_description --build build/rmtt_description
# 编译rmtt_teleop模块
catkin_make --source sunray_drivers/rmtt/rmtt_teleop --build build/rmtt_teleop
# 编译turn_on_wheeltec_robot模块
catkin_make --source sunray_drivers/wheeltec/turn_on_wheeltec_robot --build build/turn_on_wheeltec_robot
# 编译tianbot_core模块
catkin_make --source sunray_drivers/tianbot/tianbot_core --build build/tianbot_core
# 编译tianbot_led模块
catkin_make --source sunray_drivers/tianbot/tianbot_led --build build/tianbot_led
# 编译vrpn_client_ros模块
catkin_make --source vrpn_client_ros --build build/vrpn_client_ros
# 编译sunray_swarm模块
catkin_make --source sunray_swarm --build build/sunray_swarm
# 编译sunray_swarm_sim模块
catkin_make --source sunray_swarm_sim --build build/sunray_swarm_sim
# 编译turn_on_wheeltec_robot模块
catkin_make --source sunray_drivers/web_cam --build build/web_cam