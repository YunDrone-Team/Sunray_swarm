# 定义等待时间
SLEEP_TIME=5

# 启动 roscore
gnome-terminal --window -- bash -c "roscore; exec bash"

# 等待 roscore 启动
sleep $SLEEP_TIME

# 启动多个 roslaunch 命令
gnome-terminal --tab -- bash -c "roslaunch sunray_swarm_sim multi_rmtt_sim_1.launch; exec bash"
gnome-terminal --tab -- bash -c "roslaunch sunray_swarm_sim multi_rmtt_sim_2.launch; exec bash"
gnome-terminal --tab -- bash -c "roslaunch sunray_swarm_sim multi_rmtt_orca_sim.launch; exec bash"
sleep $SLEEP_TIME
gnome-terminal --tab -- bash -c "roslaunch sunray_swarm_sim multi_ugv_sim_1.launch; exec bash"
gnome-terminal --tab -- bash -c "roslaunch sunray_swarm_sim multi_ugv_sim_2.launch; exec bash"
gnome-terminal --tab -- bash -c "roslaunch sunray_swarm_sim multi_ugv_orca_sim.launch; exec bash"