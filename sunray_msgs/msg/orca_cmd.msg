std_msgs/Header header

## 控制指令来源
string cmd_source

## ORCA算法指令
uint8 orca_cmd   

## 障碍物顶点，4个点围成一个凸体,针对SETUP_OBS模式
geometry_msgs/Point[] obs_point    ## [m]

## orca_cmd枚举
uint8 SET_HOME = 0            ##必须要先发送SET_HOME指令才能开始ORCA算法运行
uint8 RETURN_HOME = 1         ##返航至初始点
uint8 SETUP_OBS = 2           ##设置障碍物
uint8 ORCA_SCENARIO_1 = 11    ##ORCA程序预设目标点场景1
uint8 ORCA_SCENARIO_2 = 12    ##ORCA程序预设目标点场景2
uint8 ORCA_SCENARIO_3 = 12    ##ORCA程序预设目标点场景3
uint8 ORCA_SCENARIO_4 = 12    ##ORCA程序预设目标点场景4
uint8 ORCA_SCENARIO_5 = 12    ##ORCA程序预设目标点场景5
uint8 ORCA_RUN = 99    ##ORCA_RUN






    
