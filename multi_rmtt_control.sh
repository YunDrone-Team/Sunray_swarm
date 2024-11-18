source devel/setup.bash \
--tab -e 'bash -c "sleep 2; roslaunch sunray_swarm orca.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch sunray_swarm rmtt_all_drone.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch sunray_swarm swarm_lead_follower.launch; exec bash"' \

