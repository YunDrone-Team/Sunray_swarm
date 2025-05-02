#! /bin/bash
## 脚本：一键启动RVIZ仿真
## 启动6台RMTT无人机的控制节点
## 启动6台RMTT无人机的仿真节点
## 启动RMTT ORCA
## 启动6台无人车的控制节点
## 启动6台无人车的仿真节点
## 启动UGV ORCA
gnome-terminal --window -e 'bash -c "roslaunch sunray_swarm_sim multi_rmtt_control_node.launch pose_source:=1; exec bash"' \
--tab -e 'bash -c "sleep 1.0; roslaunch sunray_swarm_sim multi_rmtt_sim_node.launch; exec bash"' \
--tab -e 'bash -c "sleep 1.0; roslaunch sunray_swarm_sim multi_rmtt_orca_sim.launch; exec bash"' \
--tab -e 'bash -c "sleep 1.0; roslaunch sunray_swarm_sim multi_ugv_control_node.launch pose_source:=1; exec bash"' \
--tab -e 'bash -c "sleep 1.0; roslaunch sunray_swarm_sim multi_ugv_sim_node.launch; exec bash"' \
--tab -e 'bash -c "sleep 1.0; roslaunch sunray_swarm_sim multi_ugv_orca_sim.launch; exec bash"' \
