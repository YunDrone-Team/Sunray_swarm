source devel/setup.bash
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch vrpn_client_ros sample.launch; exec bash"' \
--tab -e 'bash -c "sleep 15; roslaunch sunray_swarm ugv_control_mode.launch; exec bash"' \

