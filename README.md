RoboMaster Tello Talent ROS Driver. Developed by Tianbot


# Sunray_RMTT
RoboMaster TT (also know as DJI Tello Talent or Ryze Tello Talent) features improved hardware and an LED light. RoboMaster TT suports Wi-Fi 5G channel and a flying map, which can be used for low-cost drone swarm. 

## 安装
Only support authorized devices and the environment is pre-configured in ROS2GO Noetic version.

```
##
pip install robomaster==0.1.1.63
## 下载代码
git clone https://gitee.com/yundrone_sunray2023/Sunray_RMTT
## 编译
./build.sh
```

## 仿真

```
cd Sunray_RMTT
./rmtt_sim_step1.sh
./rmtt_sim_step2.sh

## start show
roslaunch sunray_rmtt rmtt_show.launch
```

```
cd Sunray_RMTT
./rmtt_nokov_step1.sh
./rmtt_nokov_step2.sh

## start show
roslaunch sunray_rmtt rmtt_nokov.launch
```

## TODO

单机测试

单车测试

多机测试

多车测试