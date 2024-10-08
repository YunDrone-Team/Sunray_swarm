# Sunray_swarm
## neu_display(二期)

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

## 单个RMTT仿真测试
cd Sunray_swarm
./sim_single_rmtt.sh
## 需要在终端发出对应指令，可以测试：INIT、HOLD、POS_CONTROL、VEL_CONTROL_BODY、VEL_CONTROL_ENU、TAKEOFF、LAND
```

