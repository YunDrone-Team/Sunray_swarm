source devel/setup.bash
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch sunray_swarm_sim multi_ugv_sim_1.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch sunray_swarm_sim multi_ugv_sim_2.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch sunray_swarm_sim multi_ugv_orca_sim.launch; exec bash"' \
--window -e 'bash -c "sleep 1; roslaunch sunray_swarm_sim agent_terminal_station.launch agent_type:=1 agent_num:=6; exec bash"' \
