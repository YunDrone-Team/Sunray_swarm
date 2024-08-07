std_msgs/Header header

## 基本状态
uint8 mission_state
## 基本状态
uint8 uav_id                ## 无人机编号
## 是否到达目标点
bool arrived_goal
## 是否到达目标点
bool arrived_all_goal
## 目标位置
float32[2] goal                 ## [m]
## ORCA期望速度
float32[2] vel_orca                 ## [m/s]





    
