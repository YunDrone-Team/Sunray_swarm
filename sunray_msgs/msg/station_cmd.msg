std_msgs/Header header

## 对于起飞、降落、悬停、ORCA_RUN指令：当指定ID的时候，只有指定的ID响应
## 对于其他指令，uav_id设置为99，所有飞机响应该状态  
uint8 uav_id  

## 状态机指令
uint8 mission_state   
uint8 scenario_id           
 





    
