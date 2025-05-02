#! /bin/bash
## 脚本：一键启动？？？？？
gnome-terminal --window -e 'bash -c "roslaunch sunray_swarm ugv_control_node_diff.launch ugv_id:=1; exec bash"' \
