source devel/setup.bash

gnome-terminal --window -e 'bash -c "roslaunch sunray_rmtt rmtt_sim_node_nokov.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch sunray_rmtt rmtt_station.launch; exec bash"' \
