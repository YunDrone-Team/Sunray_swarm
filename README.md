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
## 无人车仿真测试 - ORCA
cd Sunray_swarm
./ugv_sim.sh
## 需要在终端发出对应指令无人车才会开始避障


## RMTT仿真测试 - ORCA
cd Sunray_swarm
./rmtt_sim.sh
## 需要在终端发出对应指令RMTT才会开始避障
```

```
## 多智能体绕圈
cd Sunray_swarm
./ugv_sim.sh
## 可以在这个程序里修改对应轨迹参数
rosrun sunray_swarm circle_trajectory 
```

## TODO

单机测试

单车测试

多机测试

多车测试