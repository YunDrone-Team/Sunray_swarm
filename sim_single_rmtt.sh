source devel/setup.bash
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch sunray_swarm single_rmtt_sim.launch; exec bash"' \
--window -e 'bash -c "sleep 1; roslaunch sunray_swarm agent_station.launch; exec bash"' \
