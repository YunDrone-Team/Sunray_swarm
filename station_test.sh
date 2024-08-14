source devel/setup.bash
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch sunray_swarm ugv_sim_node.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch sunray_swarm ugv_control_sim_mode.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch sunray_swarm rmtt_sim_node.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch sunray_swarm rmtt_control_sim_mode.launch rviz_enable:=false; exec bash"' \
