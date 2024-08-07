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






    
