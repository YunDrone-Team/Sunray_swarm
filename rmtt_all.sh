source devel/setup.bash

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch vrpn_client_ros sample.launch"' \
--tab -e 'bash -c "sleep 3; roslaunch sunray_swarm orca.launch"' \
--tab -e 'bash -c "sleep 3; cd sunray_swarm/launch_agent/; sleep 3; ./generate_launch.py -n 1; sleep 5;roslaunch sunray_swarm rmtt_all_drone.launch"' \
--tab -e 'bash -c "sleep 3; roslaunch sunray_swarm swarm_nokov.launch"' \
--tab -e 'bash -c "sleep 3; cd ; sleep 1; cd swarmV1.1.0/; sleep 1; ./startSwarm.sh"' \
