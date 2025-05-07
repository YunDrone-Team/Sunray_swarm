#!/bin/bash

LAUNCH_FILE="sample.launch"

is_launch_running() {
    pgrep -f "$LAUNCH_FILE" > /dev/null
    return $?
}

if is_launch_running; then
    echo "vrpn 已经在运行，无需再次启动。"
else
    echo "启动 vrpn..."
    gnome-terminal --window -e 'bash -c "roslaunch vrpn_client_ros sample.launch ; exec bash"'
fi

read -p "请输入ugv的数量: " AGENT_NUM

sleep 3
gnome-terminal --window -e "bash -c \"roslaunch sunray_swarm orca_ugv.launch agent_num:=$AGENT_NUM; exec bash\""