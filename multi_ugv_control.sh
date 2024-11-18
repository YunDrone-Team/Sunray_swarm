source devel/setup.bash \
--tab -e 'bash -c "sleep 2; roslaunch sunray_swarm orca_ugv.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch sunray_swarm ugv_control_mode.launch; exec bash"' \
--tab -e 'bash -c "sleep 2; roslaunch sunray_swarm swarm_lead_follower_ugv.launch; exec bash"' \

