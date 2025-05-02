#! /bin/bash
## 脚本：测试脚本（差速轮）
gnome-terminal --window -e 'bash -c "roslaunch sunray_swarm_sim gazebo_world.launch; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_swarm_sim gazebo_multi_ugv_diff.launch; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_swarm_sim multi_ugv_control_node.launch ugv_type:=1; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_swarm_sim multi_ugv_orca_sim.launch; exec bash"' \