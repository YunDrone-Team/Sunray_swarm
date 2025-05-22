#! /bin/bash

## 虽然环境变量里面已经做了如下设置，但是设置为开机自启动脚本的时候，系统还没有完全启动，所以需要手动source
source /opt/ros/noetic/setup.bash
source ~/Sunray_swarm/devel/setup.bash
export ROS_MASTER_URI=http://192.168.25.91:11311
export ROS_HOSTNAME=192.168.25.161
export ROS_IP=192.168.25.161

# 等待开发主机中的roscore就绪
echo "等待roscore启动..."

max_wait=1000
timeout_flag=true  # 添加超时标志

for ((i=1; i<=$max_wait; i++)); do
    if rosnode list 2>/dev/null | grep -q '/rosout'; then
        echo "roscore 已就绪！"
        timeout_flag=false
        break
    fi
    echo "等待中... ($i/$max_wait)"
    sleep 1
done

# 超时后退出脚本
if $timeout_flag; then
    echo "错误：等待 roscore 超时！"
    exit 1  # 非零退出码表示异常退出
fi

## 不同的无人车注意修改下面参数中的ugv_id
## 启动wheeltec_driver
sleep 5
gnome-terminal --window --title="wheeltec_driver" -- bash -c "roslaunch sunray_swarm wheeltec_driver.launch ugv_id:=1; exec bash"
sleep 8
## 启动tianbot_driver
##gnome-terminal --window --title="tianbot_driver" -- bash -c "roslaunch sunray_swarm tianbot_driver.launch ugv_id:=1; exec bash"
## 启动ugv_control_node
gnome-terminal --window --title="ugv_control_node" -- bash -c "roslaunch sunray_swarm ugv_control_node.launch ugv_id:=1; exec bash"
## 启动ugv_control_node_diff
##gnome-terminal --window --title="ugv_control_node" -- bash -c "roslaunch sunray_swarm ugv_control_node_diff.launch ugv_id:=1; exec bash"
##启动ugv_cam
sleep 5
gnome-terminal --window --title="ugv_cam" -- bash -c "roslaunch sunray_swarm ugv_camera.launch ugv_id:=1; exec bash"

