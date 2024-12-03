source devel/setup.bash
gnome-terminal --window -e 'bash -c "roslaunch sunray_swarm orca_ugv.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch sunray_swarm ugv_control_mode.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch sunray_swarm swarm_circle_ugv.launch; exec bash"' \


