## 自定义消息：ORCA算法状态

## 消息header
std_msgs/Header header
## ORCA指令
uint8 orca_cmd
## ORCA算法中智能体数量
uint8 agent_num               
## 当前智能体ID
uint8 agent_id               
## 当前智能体是否到达目标点
bool arrived_goal
## 全部智能体是否到达目标点
bool arrived_all_goal
## 当前智能体的目标位置和航向角
float32[2] goal                 ## [m]
float32 yaw                     ## [deg]
## 当前智能体的ORCA期望速度（ORCA算法计算得到）
float32[2] vel_orca                 ## [m/s]
## 当前智能体的Home点
float32[2] home_pos                 ## [m]
float32 home_yaw                    ## [deg]




    
