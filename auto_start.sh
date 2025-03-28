#! /bin/bash
source /opt/ros/noetic/setup.bash
source ~/Sunray_swarm/devel/setup.bash

gnome-terminal --window --title="ROS Core" -- bash -c "roscore; exec bash"
sleep 10  # 等待 roscore 初始化
gnome-terminal --window --title="ugv_control_node" -- bash -c "roslaunch sunray_swarm ugv_control_node.launch; exec bash"
