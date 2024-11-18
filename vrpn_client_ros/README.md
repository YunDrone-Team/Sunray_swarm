# vrpn_client_ros
在ROS中使用[VRPN](http://wiki.ros.org/vrpn_client_ros)获取动作捕捉系统数据

### 已测试环境
- Vicon Tracker 3.4, Ubuntu 18.04, ROS Melodic
- Vicon Tracker 3.4, Ubuntu 20.04, ROS Noetic


### 使用方法
- 安装依赖项，在Ubuntu 18.04中
  ```cmd 
  sudo apt-get install ros-melodic-vrpn
  ```
  或者在Ubuntu 20.04中
  ```cmd
  sudo apt-get install ros-noetic-vrpn 
  ```

- 安装编译
  ```cmd
  cd ~/catkin_ws/src 
  git clone https://gitee.com/ASSIL/vrpn_client_ros.git
  cd vrpn_client_ros 
  git checkout assil 
  cd ../..
  catkin_make 
  source devel/setup.bash
  ```

- 获取动捕数据
  ```cmd 
  roslaunch vrpn_client_ros assil_vicon.launch
  ```

### 更多资料
- [Vicon动捕系统使用教程](manual/vicon/vicon_manual.md)
