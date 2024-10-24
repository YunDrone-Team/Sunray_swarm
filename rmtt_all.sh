source devel/setup.bash

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch vrpn_client_ros sample.launch"' \
--tab -e 'bash -c "sleep 3; roslaunch sunray_swarm orca.launch"' \
--tab -e 'bash -c "sleep 3; roslaunch sunray_swarm rmtt_all_drone.launch"' \
--tab -e 'bash -c "sleep 3; roslaunch sunray_swarm rmtt_show.launch"' \
