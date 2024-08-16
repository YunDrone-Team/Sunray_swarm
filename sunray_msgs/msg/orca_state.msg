std_msgs/Header header

## 智能体数量
uint8 agent_num               
## 智能体ID
uint8 agent_id               
## 当前智能体是否到达目标点
bool arrived_goal
## 全部智能体是否到达目标点
bool arrived_all_goal
## 目标位置
float32[2] goal                 ## [m]
## ORCA期望速度
float32[2] vel_orca                 ## [m/s]





    
