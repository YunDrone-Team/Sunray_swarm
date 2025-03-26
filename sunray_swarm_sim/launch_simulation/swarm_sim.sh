gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 0.5; roslaunch sunray_swarm_sim multi_rmtt_sim_1.launch; exec bash"' \
--tab -e 'bash -c "sleep 0.5; roslaunch sunray_swarm_sim multi_rmtt_sim_2.launch; exec bash"' \
--tab -e 'bash -c "sleep 0.5; roslaunch sunray_swarm_sim multi_rmtt_orca_sim.launch; exec bash"' \
--tab -e 'bash -c "sleep 2.5; roslaunch sunray_swarm_sim multi_ugv_sim_1.launch; exec bash"' \
--tab -e 'bash -c "sleep 1.5; roslaunch sunray_swarm_sim multi_ugv_sim_2.launch; exec bash"' \
--tab -e 'bash -c "sleep 0.5; roslaunch sunray_swarm_sim multi_ugv_orca_sim.launch; exec bash"' \
