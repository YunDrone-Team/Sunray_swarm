## 自定义消息：智能体控制命令

## 消息header
std_msgs/Header header

## 智能体ID：只有指定ID的智能体才会响应该命令(注：当agent_id赋值为99时，所有智能体均响应)
uint8 agent_id  

## 控制指令来源
string cmd_source

## 智能体控制指令
uint8 control_state   

## control_state控制指令枚举
uint8 INIT = 0                      ## 初始化：不执行任何操作
uint8 HOLD = 1                      ## 悬停：无人机悬停在当前点，无人车停止移动
uint8 POS_CONTROL = 2               ## 位置控制：惯性系中，移动到指定点和指定偏航角，需要配合desired_pos和desired_yaw
uint8 VEL_CONTROL_BODY = 3          ## 机体系速度控制：机体系中，按照指定的速度移动，需要配合desired_vel（包括线速度和角速度）
uint8 VEL_CONTROL_ENU = 4           ## 位置控制：惯性系中，按照指定的速度和指定的偏航角移动，需要配合desired_vel和desired_yaw
uint8 TAKEOFF = 11                  ## 起飞：仅针对无人机
uint8 LAND = 12                     ## 降落：仅针对无人机

## 期望位置、偏航角 -> 对应POS_CONTROL模式
geometry_msgs/Point desired_pos    ## [m]
float32 desired_yaw                 ## [rad]

## 期望速度 -> 对应VEL_CONTROL_BODY和VEL_CONTROL_ENU模式
geometry_msgs/Twist desired_vel




    
