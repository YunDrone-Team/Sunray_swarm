#! /bin/bash
## 脚本：一键启动Gazebo仿真
## 加载gazebo world
## 加载6台无人车模型（默认为麦轮）
## 加载6台无人机模型
## 启动6台无人车的控制节点（默认为麦轮）
## 启动6台无人机的控制节点
## 启动UGV ORCA
## 启动RMT ORCA
gnome-terminal --window -e 'bash -c "roslaunch sunray_swarm_sim gazebo_world.launch; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_swarm_sim gazebo_multi_ugv_mac.launch; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_swarm_sim gazebo_multi_rmtt.launch; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_swarm_sim multi_ugv_control_node.launch ugv_type:=0; exec bash"' \
--tab -e 'bash -c "sleep 1.0; roslaunch sunray_swarm_sim multi_rmtt_control_node.launch pose_source:=3; exec bash"' \
--tab -e 'bash -c "sleep 1.0; roslaunch sunray_swarm_sim multi_ugv_orca_sim.launch; exec bash"' \
--tab -e 'bash -c "sleep 1.0; roslaunch sunray_swarm_sim multi_rmtt_orca_sim.launch; exec bash"' \