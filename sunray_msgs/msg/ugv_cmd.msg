std_msgs/Header header

## 当指定ID的时候，只有指定的ID响应
uint8 agent_id  

## 状态机指令
uint8 control_state   

## 期望位置、偏航角 -> 对应POS_CONTROL模式
geometry_msgs/Point desired_pos    ## [m]
float32 desired_yaw                 ## [rad]

## 期望速度 -> 对应VEL_CONTROL模式
geometry_msgs/Twist desired_vel

## control_state枚举
uint8 INIT = 0
uint8 HOLD = 1
uint8 POS_CONTROL = 2
uint8 VEL_CONTROL = 3
uint8 ORCA_MODE = 4
uint8 TRACK_MODE = 5
uint8 RETURN_HOME = 6


    
