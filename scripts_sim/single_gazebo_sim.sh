#! /bin/bash
## 脚本：一键启动Gazebo仿真(1无人机+1无人车)
gnome-terminal --window -e 'bash -c "roslaunch sunray_swarm_sim gazebo_world.launch; exec bash"' \
--tab -e 'bash -c "sleep 5.0; roslaunch sunray_swarm_sim gazebo_single_model.launch; exec bash"' \