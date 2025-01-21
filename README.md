  # Sunray_swarm
  思锐多智能体协同控制与规划开发平台
  配套文档链接：https://wiki.yundrone.cn/docs/rmtt_doc
  ### 1. 安装依赖

  **检查安装vrpn**：运行`sudo apt-get install ros-<your-ros-distro>-vrpn`来安装`vrpn_client_ros`包。

```Bash
sudo apt-get install ros-noetic-vrpn

```


  **检查安装Tello无人机环境依赖(使用清华源进行安装)**：运行`sudo apt-get install ros-<your-ros-distro>-vrpn`来安装`vrpn_client_ros`包。

```Bash
pip3 install robomaster==0.1.1.63 -i https://pypi.tuna.tsinghua.edu.cn/simple
```



###  2: 如何安装配置Sunray_swarm代码及编译

  #### 步骤 1: 下载源码

  打开终端（`ctrl+alt+t`），输入如下指令下载代码

```Bash
# 从gitee服务器clone代码，默认放置在home目录下
git clone https://gitee.com/yundrone_sunray2023/Sunray_swarm.git

```



  #### 步骤 2: 编译源码

  如果你还没有编译你的工作空间，请使用以下命令进行编译。

```Bash
cd ~/Sunray_swarm
# 通过脚本编译代码
./build.sh

```


  

  #### 步骤 3: 更新 ROS 工作空间（当前终端）

  确保你的 ROS 工作空间已经被正确设置，并且代码已经被编译（只在当前终端生效）。

```Bash
source /opt/ros/noetic/setup.bash
source ~/Sunray_swarm/devel/setup.bash

```



  **注意：**上述代码只在当前终端生效，写入配置文件bashrc中即可在所有终端生效。



  进入配置文件后使用键盘输入i,光标移动到末尾，输入代码，键盘输入`:wq!` 即可保存。

```Bash
source /opt/ros/noetic/setup.bash
source ~/Sunray_swarm/devel/setup.bash
```

###  3: 如何安装地面站（无需编译）

  #### 步骤 1: 下载程序

  

  #### 步骤 2: 启动地面站程序

  测试地面站程序是否能够正常启动：

```Bash
cd swarmV1.0.0/
# 通过脚本启动地面站
./startSwarm.sh

```



###  4: 使用地面站控制一台RMTT进行起飞操控

  #### 步骤 1: 设置RMTT无人机路由模式(如已经设置过，可跳过该步骤)

  首先进行特洛无人机进行连接路由器设置，连接每台Tello无人机的热点，使用python脚本 `set.sta.py` 设置无人机连接的路由器。以下是使用 `set.sta.py` 的方式：

  其中yundrone_NOKOV为wifi名称，12345678为密码

```Bash
cd Sunray_swarm/sunray_drivers/rmtt/rmtt_driver/scripts/
./set_sta.py yundrone_NOKOV 12345678


```


  #### 步骤 2: 启动 ROS Master

  启动 ROS Master（`roscore`），这是所有 ROS 节点通信的核心。

  

```Bash
# 启动ROS主节点
roscore

```


  #### 步骤 3: 启动 `vrpn_client` 节点


```text
roslaunch vrpn_client_ros sample.launch
```

  
  #### 步骤 4: 启动 `rmtt_control、rmtt_node` 节点

  在启动无人机节点前需要扫设置无人机的IP，在launch文件夹下使用脚本进行获取无人机SN以及IP

```Bash
cd Sunray_swarm/sunray_swarm/launch/
./generate_launch.py -n 6
```

  generate_launch.py脚本直接创建了一个 `rmtt_all_drone.launch` 文件，参数`-n 6 `表示扫描有6架无人机：其中`local_ip`为本机IP



```Bash
roslaunch sunray_swarm rmtt_all_drone.launch
```



  #### 步骤 5: 启动地面站节点(需下载)


```XML
cd swarmV1.0.0/
./startSwarm.sh

```


  #### 步骤 6: 操控无人机起飞、发送目标点

  完成上述操作后即可在地面站看到上线设备以及设备的信息


  点击`单机起飞`按钮即可起飞无人机，等待3秒无人机起飞后在位置控制区域输入`x、y、yaw`，点击发送控制无人机即可移动到目标点,无人机到达目标点后等待3秒点击`单机降落`即可降落RMTT
![alt text](image.png)


### 5.1单机示例程序

#### 5.1.1悬停示例（单机）

无人机目标点按钮按下后无人机一键起飞，并在指定位置悬停:

```Bash
"输入下述指令后，点击地面站无人机目标点悬停即可控制无人机移动到指定目标点"
roscore

roslaunch vrpn_client_ros sample.launch

roslaunch sunray_swarm rmtt_all_drone.launch

roslaunch sunray_swarm rmtt_hover.launch

rostopic pub /sunray_swarm/demo/rmtt_hover std_msgs/Bool "data: true"

```


#### 5.1.2画圆示例（单机）

无人机画圆按钮按下后无人机一键起飞，并按照指定的圆形轨迹移动:

```Bash
"输入下述指令后，点击地面站无人机画圆即可控制无人机起飞移动到点后以圆点为中心、半径1米、持续20秒中的圆形轨迹"
roscore

roslaunch vrpn_client_ros sample.launch

roslaunch sunray_swarm rmtt_all_drone.launch

roslaunch sunray_swarm rmtt_circle.launch

rostopic pub /sunray_swarm/demo/rmtt_hover std_msgs/Bool "data: true"

```


#### 5.1.3航点示例（单机）

无人机航点按钮按下后无人机一键起飞，无人机起飞依次到设定的目标点:

```Bash
"输入下述指令后，点击地面站无人机航点即可控制无人机起飞依次到设定的目标点"
roscore

roslaunch vrpn_client_ros sample.launch

roslaunch sunray_swarm rmtt_all_drone.launch

roslaunch sunray_swarm rmtt_waypoint.launch

rostopic pub /sunray_swarm/demo/rmtt_circle std_msgs/Bool "data: true" 

```


#### 5.1.1路径规划示例（单机）

无人机路径规划按钮按下后无人机一键起飞，无人机起飞依次到设定的目标点并安全到达:

```Bash
"输入下述指令后，点击地面站无人机路径规划即可控制无人机起飞依次到设定的目标点并安全到达"
roscore

roslaunch vrpn_client_ros sample.launch

roslaunch sunray_swarm rmtt_all_drone.launch

roslaunch sunray_swarm rmtt_pathplanning.launch

rostopic pub /sunray_swarm/demo/rmtt_pathPlanning std_msgs/Bool "data: true"

```


### 5.2多机示例程序

#### 5.1.1阵型示例（多机）

无人机NOKOV阵型按钮按下后无人机一键起飞，无人机起飞依次到设定的目标点并安全到达:

```Bash
"输入下述指令后，点击地面站无人机阵型即可控制无人机起飞依次到设定的目标点，摆出N、O、K、O、V阵型，让后返回起始点"
roscore

roslaunch vrpn_client_ros sample.launch

roslaunch sunray_swarm orca.launch

roslaunch sunray_swarm rmtt_all_drone.launch

roslaunch sunray_swarm swarm_nokov.launch

rostopic pub /sunray_swarm/demo/swarm_formation_nokov std_msgs/Bool "data: false"
```


#### 5.1.1固定障碍物示例

无人机固定障碍物按钮按下后无人机一键起飞，无人机起飞依次到设定的目标点展示阵型避开障碍物并安全到达:

```Bash
"输入下述指令后，点击地面站无人机阵型即可控制无人机起飞依次到设定的目标点，摆出N、O、K、O、V阵型，让后返回起始点，同时避开程序设置障碍物"
roscore

roslaunch vrpn_client_ros sample.launch

roslaunch sunray_swarm rmtt_all_drone.launch

roslaunch sunray_swarm orca.launch

roslaunch sunray_swarm swarm_with_obstacles.launch

rostopic pub /sunray_swarm/demo/swarm_with_obstacles std_msgs/Bool "data: false" 

```


