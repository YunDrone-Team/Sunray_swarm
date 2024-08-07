std_msgs/Header header

## 基本状态
uint8 uav_id                ## 无人机编号
string uav_ip               ## 无人机IP
bool connected             ## 是否连接上rmtt驱动
bool odom_valid             ## 是否收到动捕数据

## 位置、速度、姿态
float32[3] pos                 ## [m]
float32[3] vel                 ## [m/s]
float32[3] att                 ## [rad]
geometry_msgs/Quaternion attitude_q ## 四元数

## 电池状态
float32 battery                ## [0-1]




    
