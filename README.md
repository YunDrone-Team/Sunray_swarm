# Sunray_swarm

思锐多智能体协同控制与规划开发平台

## 安装

```
## 安装vrpn依赖
sudo apt-get install ros-melodic-vrpn
## 安装RMTT依赖
pip install robomaster==0.1.1.63
## 下载代码
git clone https://gitee.com/yundrone_sunray2023/Sunray_swarm
## 编译
./build.sh
```

## 仿真

```
## 单个无人车仿真测试
cd Sunray_swarm
./sim_single_ugv.sh
## 需要在终端发出对应指令，可以测试：INIT、HOLD、POS_CONTROL、VEL_CONTROL_BODY、VEL_CONTROL_ENU


## 单个RMTT仿真测试
cd Sunray_swarm
./sim_single_rmtt.sh
## 需要在终端发出对应指令，可以测试：INIT、HOLD、POS_CONTROL、VEL_CONTROL_BODY、VEL_CONTROL_ENU、TAKEOFF、LAND
```

```
## 多智能体ORCA仿真测试
cd Sunray_swarm
./sim_multi_ugv_orca.sh
## 需要在终端发出对应指令无人车才会开始避障
```

```
## 多智能体绕圈
cd Sunray_swarm
./sim_multi_ugv_circle.sh
## 可以在这个程序里修改对应轨迹参数(以及数量)
roslaunch sunray_swarm circle_trajectory.launch
## 需要输入
```

```
## 多智能体ORCA仿真测试 - NOKOV阵型
cd Sunray_swarm
./sim_multi_ugv_orca.sh
## 启动阵型任务节点
roslaunch sunray_swarm formation_nokov.launch
## 新建终端发布如下指令开始阵型
rostopic pub /sunray_swarm/formation_nokov std_msgs/Bool "data: false" 
```

```
## 多智能体ORCA仿真测试 - 固定障碍点
cd Sunray_swarm
./sim_multi_ugv_orca.sh
## 启动阵型任务节点
roslaunch sunray_swarm swarm_with_obstacles.launch
## 新建终端发布如下指令开始阵型
rostopic pub /sunray_swarm/swarm_with_obstacles std_msgs/Bool "data: false" 
```

## 真机测试

```
## 追踪
roslaunch sunray_swarm track_mission.launch
```

```
## 多智能体跟随
cd Sunray_swarm
./sim_multi_ugv_circle.sh
## 可以在这个程序里修改对应轨迹参数(以及数量)
roslaunch sunray_swarm multi_lead_follower.launch
## 新建终端发布如下指令开始阵型
rostopic pub /sunray_swarm/leader_follower std_msgs/Bool "data: true"
```
```
## 多智能体队形变换
cd Sunray_swarm
./sim_multi_ugv_circle.sh
## 可以在这个程序里修改对应轨迹参数(以及数量)
roslaunch sunray_swarm multi_formation_triangle.launch
## 新建终端发布如下指令开始阵型
rostopic pub /sunray_swarm/formation_control std_msgs/Bool "data: true"
```
## TODO

单机测试

单车测试
```
## 单智能体绕圈
cd Sunray_swarm
./sim_multi_ugv_circle.sh
## 可以在这个程序里修改对应轨迹参数(以及数量)
roslaunch sunray_swarm single_circle.launch
## 新建终端发布如下指令开始阵型
rostopic pub /sunray_swarm/single_circle std_msgs/Bool "data: true" 
```
多机测试

多车测试