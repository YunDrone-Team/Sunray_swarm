#!/bin/bash

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

sleep 3
gnome-terminal --window -e 'bash -c "roslaunch sunray_swarm orca_ugv.launch agent_num:=6; exec bash"' \