source devel/setup.bash

gnome-terminal --window -e 'bash -c "roslaunch sunray_swarm orca_sim.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch sunray_swarm rmtt_sim_node.launch"' \
--tab -e 'bash -c "sleep 3; roslaunch sunray_swarm rmtt_control_sim_mode.launch"' \
--tab -e 'bash -c "sleep 3; roslaunch sunray_swarm rmtt_show.launch"' \
