#!/bin/bash

## 脚本：一键启动无人机实验（所有无人机）
LAUNCH_FILE="sample.launch"

is_launch_running() {
    pgrep -f "$LAUNCH_FILE" > /dev/null
    return $?
}

if is_launch_running; then
    echo "vrpn 已经在运行，无需再次启动。"
else
    echo "启动 $LAUNCH_FILE ..."
    gnome-terminal --window -e 'bash -c "roslaunch vrpn_client_ros sample.launch ; exec bash"'
fi

sleep 1

read -p "请输入需要启动的rmtt数量: " num

gnome-terminal --window -e "bash -c \"cd ~/Sunray_swarm/sunray_swarm/launch_rmtt/&& python3 generate_launch.py -n $num; exec bash\""

read -p "等待rmtt_all_drone.launch 生成完毕，按任意键继续..."

gnome-terminal --window -e 'bash -c "roslaunch sunray_swarm rmtt_all_drone.launch; exec bash"' \
--tab -e "bash -c \"sleep 2.0; roslaunch sunray_swarm orca_rmtt.launch agent_num:=$num; exec bash\""