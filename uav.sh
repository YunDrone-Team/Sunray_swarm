gnome-terminal --window -e 'bash -c "roslaunch vrpn_client_ros sample.launch; exec bash"' \
--tab -e 'bash -c "sleep 2.0; roslaunch sunray_swarm rmtt_all_drone.launch; exec bash"' \