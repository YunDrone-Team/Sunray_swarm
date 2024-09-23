std_msgs/Header header

## 基本状态
uint8 agent_type              ## 智能体类型
uint8 agent_id                ## 智能体编号
string agent_ip               ## 智能体IP
bool connected                ## 是否连接上智能体驱动
bool odom_valid               ## 是否收到动捕数据

## 智能体位置、速度、姿态
float32[3] pos                 ## [m]
float32[3] vel                 ## [m/s]
float32[3] att                 ## [rad]
geometry_msgs/Quaternion attitude_q ## 四元数

## 智能体电池状态
float32 battery                ## [0-1]

## 智能体当前执行的控制状态
uint8 control_state
                          
## agent_type枚举
uint8 RMTT = 0
uint8 UGV = 1
uint8 SIKONG = 2