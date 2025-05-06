#!/bin/bash

## 脚本：一键启动无人机实验（所有无人机）
LAUNCH_FILE="sample.launch"

is_launch_running() {
    pgrep -f "$LAUNCH_FILE" > /dev/null
    return $?
}

# 检查sample.launch是否正在运行
if is_launch_running; then
    echo "$LAUNCH_FILE 已经在运行，无需再次启动。"
else
    echo "启动 $LAUNCH_FILE ..."

    gnome-terminal --window -e 'bash -c "roslaunch vrpn_client_ros sample.launch ; exec bash"'
fi

sleep 2
gnome-terminal --window -e 'bash -c "roslaunch sunray_swarm rmtt_all_drone.launch; exec bash"' \
--tab -e 'bash -c "sleep 3.0; roslaunch sunray_swarm orca_rmtt.launch agent_num:=6; exec bash"' \