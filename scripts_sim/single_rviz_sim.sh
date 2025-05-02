#! /bin/bash
## 脚本：一键启动RVIZ仿真（1无人机+1无人车）
gnome-terminal --window -e 'bash -c "roslaunch sunray_swarm_sim single_rmtt_sim.launch; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_swarm_sim single_ugv_sim.launch; exec bash"' \