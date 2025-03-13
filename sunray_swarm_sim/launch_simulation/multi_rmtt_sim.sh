gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch sunray_swarm_sim multi_rmtt_sim_1.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch sunray_swarm_sim multi_rmtt_sim_2.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch sunray_swarm_sim multi_rmtt_orca_sim.launch; exec bash"' \
--window -e 'bash -c "sleep 1; roslaunch sunray_swarm_sim agent_terminal_station.launch agent_type:=0 agent_num:=6; exec bash"' \
