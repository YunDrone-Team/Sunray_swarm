# Vicon动捕系统使用教程
本教程简要介绍了Vicon室内动作捕捉系统的使用方法。


## 系统组成
- 系统架构
  ![](img/系统架构与网络设置.png)

- 主要设备
  - Vicon相机（Vantage V5）
  - PoE交换机，通过网线与Vicon相机连接
  - 标定杆
  - 标记点（球）
  - Vicon主机：Windows 10系统，安装有Vicon Tracker 3.x软件，通过网线与PoE交换机连接
  - 路由器：建立局域网，连接Vicon主机、地面站、远端无人机/车
  - ROS地面站主机：Ubuntu 18.04（安装ROS melodic）或Ubuntu 20.04（安装 ROS noetic）系统，通过网线直接与路由器连接
  - 无人机/车：搭载机载计算机，Ubuntu 18.04（安装ROS melodic），通过WiFi与路由器连接

  **`注意`**：
  - `关闭Windows防火墙`
  - 在Windows 10系统下运行Tracker 3.4软件，需要设置`以兼容模式运行这个程序`

  ![](img/以兼容模式运行.png)


## Vicon主机网络设置
设置完成后不需要更改，设置方法为：
- 用网线将PoE交换机和Vicon主机连接
- 打开`控制面板`-`网络和Internet`-`网络和共享中心`，可以看到`查看活动网络`面板 
- 在该面板中，点击连接PoE交换机的以太网，然后点击`属性`
- 在`属性`面板中，选择`Inernet协议版本4 (TCP/IPv4)`，并继续点击`属性`按钮

  ![](img/以太网属性.png)

- 在弹出的TCP/IPv4属性面板中，选择`使用下面的IP地址`来设置如下静态IP
    - IP地址：192.168.10.1
    - 子网掩码：255.255.255.0

  ![](img/Vicon主机连接相机IP设置.png)

- 点击`确定`，完成网络设置 


## 基本设置与相机参数调节
- 新建系统配置：打开Tracker软件，在`Resources`-`System`面板下，点击下拉菜单中`New`新建系统配置，然后保存并命名，保存时可选择`Shared`，配置文件将保存到`C:\Users\Public\Documents\Vicon\Tracker3.x\Configurations\Systems`文件夹中

  ![](img/新建System配置.png) 

- 系统基本设置：在`System`面板中，点击选择`Local Vicon System`节点，在其下方相应的`Properties`面板中，选择`Show Advanced`，可以对系统的基本属性进行设置

  ![](img/System_Local_Vicon_System_Properties.png)

  大多数属性均可采用默认值，下面是一些常用的配置属性：

  - `Requested Frame Rate`：设置系统帧率
  - `Low Jitter`：抖动消除，若开启该选项，所有相机的`Grayscale Mode`参数自动设为`Only`
  - `UDP Object Stream`：开启UDP数据流，可以不打开
  - `VRPN Stream`：在VRPN数据流中增加Filtered Tracker，可以不打开

- 相机参数设置：在`System`面板中，点击选择`Vicon Cameras`节点，在其下方的`Properties`面板中可对所有相机的参数同时进行设置。也可以展开该节点，对每个相机的参数分别进行设定 

  ![](img/相机参数.png)

  其中常用的设置参数包括：
  - `Strobe Intensity`：调节相机发射光线的强度，0-1之间，数值越大相机发射的光线越强，标记点越容易被检测到（同时也越容易出现噪点）。可结合相机的`Threshold`参数一起调节，来消除噪点影响，但通常建议设为1
  - `Threshold`：调节标记点被检测到的亮度阈值，0-1之间，数值越大阈值越高，标记点越难被检测到，一般设在0.2-0.5之间，用来调节消除噪点影响
  - `Focal length`：相机焦距


## 噪点消除
在系统标定前，需要对动捕空间中的噪点进行消除，一般有以下两种方法，推荐在第一种方法无法消除全部噪点的情况下再使用第二种方法。
- 调节相机参数：在场地中央放置一个带有标记点的物体，选择`System`-`Vicon Cameras`下的某一相机，同时在`View`面板的下拉菜单中选择`Camera`模式，可以看到该相机当前视野中的检测数据

  ![](img/camera_view.png)

  可以通过鼠标右键放大/缩小当前窗口。若不存在噪点，则此时相机应仅能清晰看到所放置物体上的标记点。若存在噪点，或者放置的标记点显示不稳定，可通过调节相机的`Strobe Intensity`和`Threshold`参数，使噪点消除，且放置的标记点清晰稳定可见。在相机光圈和焦距正常的情况下，大多数时候通过调节`Threshold`就能消除噪点
  
  然后，对其他所有相机重复以上操作，通常在同一时间和相似的光照条件下，所有相机的参数（`Threshold`）应该几乎一致 

- 创建遮挡区域：对于难以通过调节相机参数消除的噪点，可以通过创建遮挡来消除其影响。遮挡区域内的标记点将不被相机识别，因此建议尽量不要创建过大的遮挡区域
  - 手动创建：接前述，对每一个相机，在其`View`面板的`Camera`模式下，按`F7`键弹出`Options`面板，勾选`General View Options`下的`Threshold`和`Threshold Map`选项

  ![](img/Options.png)

  然后通过该窗口上方的![](img/manual_mask.png)等按钮手动创建、消除遮挡区域，使噪点不可见

  ![](img/manual_mask_area.png)

  - 自动创建：在`系统标定`过程中完成（见下述内容）


## 系统标定
每天做实验前均应先对系统进行标定，标定和实验进行时注意拉上实验室窗帘、使用室内照明，避免强烈的太阳光造成干扰。标定前需要将动捕空间中的所有标记点移除。
- 点击`Resources`面板中的`Calibrate`选项卡进入标定面板 

  ![](img/标定面板.png) 

- 创建相机遮挡：将动捕空间中的所有标记点移除，按`F7`确定已勾选`Threshold Map`选项。然后进入标定面板中的`Create Camera Masks`模块，选择`All Cameras`

  ![](img/标定-创建遮挡.png)

  点击`Start`，软件将自动为每个相机创建遮挡区域，结束后点击`Stop`完成遮挡区域创建 

  ![](img/完成自动创建遮挡.png)

- 相机标定：进入标定面板中的`Calibrate Cameras`模块
  - 在`Wand`中选择所使用的标定杆类型，这里使用`Active Wand v2` 

  ![](img/相机标定面板.png)

  - 设置其他参数，可采用默认值，其中若勾选`Auto Stop`则系统将在收集到足够数据后自动停止并进行标定计算
  - 点击`Start`，打开标定杆电源，持标定杆进入动捕空间漫步挥舞，对着每一个摄像头逐个挥动，使标定杆的运行轨迹尽量覆盖相机的视野。相机的状态由慢闪变为快闪表示其接收到的帧数越来越多，直到其收集到足够多的数据，两侧的指示灯变为绿色，然后换另外一个摄像头
  
  ![](img/Wand数据.png)

  - 所有相机的指示灯都变为绿色后，点击`Stop`，软件将自动解算相机标定结果，该结果为所有相机之间的相对位姿。解算完成后查看image error是否小于0.3，若image error为0.3~0.5则重新挥动标定杆收集数据，若为0.5以上则需要调整相机参数重新开始标定
  - 完成后将标定杆移除动捕空间。切回3D Perspective视角，可以发现此时的相机所在平面与显示的地面（X-Y平面）并不平行，因此接下来需要标定坐标系
- 设置动捕空间坐标系原点和坐标轴指向：进入标定面板中的`Set Volume Origin`模块 

  ![](img/标定-坐标系设置.png) 

  - 在`L-Frame`选项中，选择使用的标定杆，这里使用的是`Active Wand v2`
  - 打开标定杆电源，标定杆放入场地中央，调节标定杆底座螺丝使标定杆处于水平状态，标定杆所在平面为地面，交叉点气泡附近标记点为坐标系原点，长轴为正Y轴方向，短轴为X轴方向，Z轴可根据右手法则定出 
  - 点击`Start`按钮，并继续点击`Set Origin`按钮，完成坐标系设置 

- 保存标定结果：点击`Save`保存标定结果 
  
  ![](img/保存标定结果.png)


## 刚体建立
标定完成后可以在动捕空间中建立刚体并获取动捕数据。
- 放置标记点：将标记点小球粘贴在想要进行运动捕捉的物体（无人机、无人车）上，注意每个物体上应粘贴不少于3个标记点（推荐4个以上）；多个标记点位置应避免对称，且其几何中心应尽量与物体中心保持一致；标记点应粘贴牢固，防止物体在运动过程中振动、移位甚至脱落；不同物体上的标记点配置应尽量不同，使得动捕系统能够清晰区分
- 建立刚体：将带有标记点的物体放置在动捕空间中，由于系统默认在建立刚体时物体的当前旋转角度（roll, pitch, yaw）均为零，因此尽可能将物体平放在地面上（对应roll和pitch为零），且其头部应朝向X轴正向（对应yaw为零），然后在Tracker软件中进行下面操作：
  - 在`Resource`-`Object`面板中，点击右侧暂定按钮进入`PAUSED`模式
  
  ![](img/Object暂停.png)

  - 在`View`窗口中，切到`3D Perspective`模式，可以看到动捕空间中的标记点，按住`Alt`键并按住拖动鼠标左键选择物体上配置的多个标记点 
  - 在`Object`面板中，输入所建立刚体的名称，然后点击`Create` 
  
  ![](img/刚体名称.png)

  `注意`：如果在动捕空间中看不见放置的标记点，可以先点击上图中的`TRACK`按钮，并切换至`LIVE`模式，就可以看见标记点了；然后切换至`PAUSED`模式建立刚体

  - 重新点击`Object`面板右侧的暂停按钮，进入`LIVE`模式，系统将事实捕捉所建立刚体的位姿信息

  ![](img/建立的刚体.png)

  - 按照上述方法，可以建立多个不同的刚体；不同刚体在建立时需要输入不同的名称

  ![](img/多个不同刚体.png)


## ROS获取动捕数据
在Tracker中建立刚体并切到`LIVE`模式后，软件将自动通过VRPN向外发送数据。在另一台Ubuntu计算机中可以通过VRPN获取动捕数据
- 通过路由器建立局域网，分别连接Vicon主机和ROS开发机，并将Vicon主机的IP设置为固定IP，如`192.168.1.2`
- 确保ROS开发机与Vicon主机能够相互`ping`通
- 使用[`vrpn_client_ros`](https://gitee.com/ASSIL/vrpn_client_ros.git)在ROS中获取动捕数据，以在Ubuntu 20.04中为例：
  - 安装编译
  ```cmd
  sudo apt-get install ros-noetic-vrpn 
  cd ~/catkin_ws/src 
  git clone https://gitee.com/ASSIL/vrpn_client_ros.git
  cd vrpn_client_ros 
  git checkout assil 
  cd ../..
  catkin_make 
  source devel/setup.bash
  ```
  - 使用
  ```cmd
  roslaunch vrpn_client_ros assil_vicon.launch
  ```
  - 通过`rostopic list`查看，可以看到获取到的动捕数据，包括刚体的位姿、速度和加速度信息
  
  ![](img/vrpn_client_ros_topic.png)
  


