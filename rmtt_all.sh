source devel/setup.bash

gnome-terminal --window -e 'bash -c "roscore; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch vrpn_client_ros sample.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch sunray_rmtt rmtt_all_drone.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; roslaunch sunray_rmtt rmtt_orca.launch; exec bash"' \

# --tab -e 'bash -c "sleep 3; roslaunch sunray_rmtt rmtt_nokov.launch"' \
# gnome-terminal --window -e 'bash -c "sleep 3; cd ../neuR; source startNEU.sh;exec bash"' \





