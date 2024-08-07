source devel/setup.bash
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch sunray_swarm ugv_sim_node.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch sunray_swarm ugv_sim_control.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch sunray_swarm ugv_orca.launch; exec bash"' \
--window -e 'bash -c "sleep 1; roslaunch sunray_swarm ugv_station.launch; exec bash"' \
