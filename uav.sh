gnome-terminal --window -e 'bash -c "roslaunch sunray_swarm_sim multi_rmtt_sim_1.launch; exec bash"' \
--tab -e 'bash -c "sleep 1.0; roslaunch sunray_swarm_sim multi_rmtt_sim_2.launch; exec bash"' \