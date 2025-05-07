#!/bin/bash

## 脚本：一键启动无人机实验（所有无人机）
LAUNCH_FILE="sample.launch"

is_launch_running() {
    pgrep -f "$LAUNCH_FILE" > /dev/null
    return $?
}

# 检查sample.launch是否正在运行
if is_launch_running; then
    echo "vrpn已经在运行，无需再次启动。"
else
    echo "启动 $LAUNCH_FILE ..."
    gnome-terminal --window -e 'bash -c "roslaunch vrpn_client_ros sample.launch ; exec bash"'
fi

sleep 1

# 提示用户输入一个数字
read -p "请输入需要启动的rmtt数量: " num

# 运行generate_launch.py脚本并等待其结束
gnome-terminal --window -e "bash -c \"python3 ~/Sunray_swarm/sunray_swarm/launch_rmtt/generate_launch.py -n $num; read -p \\\"rmtt_all_drone.launch生成完毕，按任意键继续...\\\"; exec bash\""

read -p "等待rmtt_all_drone.launch生成完毕，按任意键继续..."

gnome-terminal --window -e 'bash -c "roslaunch sunray_swarm rmtt_all_drone.launch; exec bash"' \
--tab -e "bash -c \"sleep 2.0; roslaunch sunray_swarm orca_rmtt.launch agent_num:=$num; exec bash\""