source devel/setup.bash
gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch vrpn_client_ros sample.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch sunray_swarm orca.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; cd sunray_swarm/launch ; sleep 2; ./generate_launch.py -n 5; sleep 15; roslaunch sunray_swarm rmtt_all_drone.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch sunray_swarm swarm_circle.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch sunray_swarm orca_ugv.launch; exec bash"' \
--tab -e 'bash -c "sleep 15; roslaunch sunray_swarm ugv_control_mode.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch sunray_swarm swarm_circle_ugv.launch; exec bash"' \

