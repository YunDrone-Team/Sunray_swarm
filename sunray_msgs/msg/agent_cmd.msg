## 自定义消息：智能体控制命令

## 消息header
std_msgs/Header header

## 智能体ID：只有指定ID的智能体才会响应该命令(注：当agent_id赋值为99时，所有智能体均响应)
uint8 agent_id  

## 控制指令来源
string cmd_source

## 控制指令
uint8 control_state   

## control_state枚举
## 对于无人车来说，TAKEOFF和LAND为无效指令
uint8 INIT = 0
uint8 HOLD = 1
uint8 POS_CONTROL = 2
uint8 VEL_CONTROL_BODY = 3
uint8 VEL_CONTROL_ENU = 4
uint8 TAKEOFF = 11
uint8 LAND = 12

## 期望位置、偏航角 -> 对应POS_CONTROL模式
geometry_msgs/Point desired_pos    ## [m]
float32 desired_yaw                 ## [rad]

## 期望速度 -> 对应VEL_CONTROL_BODY和VEL_CONTROL_ENU模式
geometry_msgs/Twist desired_vel




    
