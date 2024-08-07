#include "rmtt_orca.h"

void RMTT_ORCA::init(ros::NodeHandle& nh)
{
    // 【参数】无人机数量
    nh.param<int>("rmtt_num", rmtt_num, 8);
    // 【参数】无人机固定飞行高度
    nh.param<float>("rmtt_height", rmtt_height, 1.0);
    // 【参数】无人机之间的假想感知距离
    nh.param<float>("rmtt_orca_param/neighborDist", rmtt_orca_param.neighborDist, 1.5);
    rmtt_orca_param.maxNeighbors = rmtt_num;
    // 【参数】数字越大，无人机响应相邻无人机的碰撞越快，但速度可选的自由度越小
    nh.param<float>("rmtt_orca_param/timeHorizon", rmtt_orca_param.timeHorizon, 2.0);
    // 【参数】数字越大，无人机响应障碍物的碰撞越快，但速度可选的自由度越小
    nh.param<float>("rmtt_orca_param/timeHorizonObst", rmtt_orca_param.timeHorizonObst, 2.0);
    // 【参数】无人机体积半径
    nh.param<float>("rmtt_orca_param/radius", rmtt_orca_param.radius, 0.3);
    // 【参数】无人机最大速度
    nh.param<float>("rmtt_orca_param/maxSpeed", rmtt_orca_param.maxSpeed, 0.5);
    // 【参数】时间步长 不确定有啥用
    nh.param<float>("rmtt_orca_param/time_step", rmtt_orca_param.time_step, 0.1);

    // 初始化订阅器和发布器
    // 【订阅】地面站指令
    station_cmd_sub = nh.subscribe<sunray_msgs::station_cmd>("/sunray_rmtt/station_cmd", 1, &RMTT_ORCA::station_cmd_cb, this);
    // 【发布】文字提示消息（回传至地面站显示）
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_rmtt/text_info", 1);
    // 初始化 goal_reached_printed
    goal_reached_printed.resize(rmtt_num, false);

    string agent_name;
    for(int i = 0; i < rmtt_num; i++) 
    {
        agent_name = "/rmtt_" + std::to_string(i+1);
        // 【订阅】无人机状态数据
        rmtt_state_sub[i] = nh.subscribe<sunray_msgs::rmtt_state>("/sunray_rmtt" + agent_name + "/rmtt_state", 1, boost::bind(&RMTT_ORCA::rmtt_state_cb,this ,_1,i));
        // 【订阅】无人机的目标点
        rmtt_goal_sub[i] = nh.subscribe<geometry_msgs::Point>("/sunray_rmtt" + agent_name + "/goal_point", 1, boost::bind(&RMTT_ORCA::rmtt_goal_cb,this ,_1,i));
        // 【发布】无人机ORCA控制指令（ENU系，单位：米/秒，Rad/秒）
		orca_cmd_pub[i] = nh.advertise<geometry_msgs::Twist>("/sunray_rmtt" + agent_name + "/orca_cmd_vel", 1); 
        // 【发布】无人机orca状态 回传地面站
		rmtt_orca_state_pub[i] = nh.advertise<sunray_msgs::rmtt_orca>("/sunray_rmtt" + agent_name + "/rmtt_orca_state", 1); 
    }

    // 打印定时器
    debug_timer = nh.createTimer(ros::Duration(100.0), &RMTT_ORCA::debugCb, this);

    for(int i = 0; i < rmtt_num; i++) 
    {
        arrived_goal[i] = false;
    }
    arrived_all_goal = false;     

    // 【函数】打印参数
    printf_param();

    node_name = ros::this_node::getName();
    cout << BLUE << node_name << ": rmtt_orca init! " << TAIL << endl;
    text_info.data = node_name + ": rmtt_orca init!";
    text_info_pub.publish(text_info);

    // ORCA算法初始化 - 添加智能体
    setup_agents();
    // ORCA算法初始化 - 添加障碍物
    // setup_obstacles();

}

bool RMTT_ORCA::orca_run()
{
    if(!start_flag)
    {
        return false;
    }

    arrived_all_goal = true;

    // 判断是否达到目标点附近
	for(int i = 0; i < rmtt_num; ++i) 
	{	
		if(!arrived_goal[i])
		{
			arrived_goal[i] = reachedGoal(i);
		}
    }

	// 更新RVO中的位置和速度
	for(int i = 0; i < rmtt_num; ++i) 
	{	
		RVO::Vector2 pos = RVO::Vector2(rmtt_state[i].pos[0],rmtt_state[i].pos[1]);
		sim->setAgentPosition(i, pos);//更新RVO仿真中的位置
	}

	// 计算每一个智能体的期望速度
	sim->computeVel();

	for(int i = 0; i < rmtt_num; ++i) 
	{	
        // 如果达到目标点附近，则使用直接控制直接移动到目标点
		if(arrived_goal[i])
		{
            cmd_vel[i].linear.x = 0.0;
            cmd_vel[i].linear.y = 0.0;
            cmd_vel[i].linear.z = 0.0;
            cmd_vel[i].angular.x = 0.0;
            cmd_vel[i].angular.y = 0.0;
            cmd_vel[i].angular.z = 0.0; 
            orca_cmd_pub[i].publish(cmd_vel[i]);
            if (!goal_reached_printed[i])
            {
                cout << BLUE << node_name << ": RMTT " << i+1 << " Arrived." << TAIL << endl;
                text_info.data = "RMTT " + std::to_string(i+1) + " Arrived.";
                text_info_pub.publish(text_info);
                goal_reached_printed[i] = true;
            }
		}
        // 如果没有达到目标点附近，则使用ORCA算法计算期望速度
		else
		{
            // 获得期望速度 注：这个速度是ENU坐标系的，并将其转换为机体系速度指令
            RVO::Vector2 vel = sim->getAgentVelCMD(i);           
            cmd_vel[i].linear.x = vel.x();
            cmd_vel[i].linear.y = vel.y();
            cmd_vel[i].linear.z = 0.0;
            cmd_vel[i].angular.x = 0.0;
            cmd_vel[i].angular.y = 0.0;
            cmd_vel[i].angular.z = 0.0; 
            orca_cmd_pub[i].publish(cmd_vel[i]);
            // 只要有一个无人机未到达目标点，整体状态就是未完成
            arrived_all_goal = false;                   
        }
	}

    // 循环遍历每个无人机，更新并发布ORCA状态信息
    for(int i = 0; i < rmtt_num; i++) 
    {
        rmtt_orca_state[i].mission_state = 0;
        rmtt_orca_state[i].uav_id = i+1;
        rmtt_orca_state[i].arrived_goal = arrived_goal[i];
        rmtt_orca_state[i].arrived_all_goal = arrived_all_goal;
        RVO::Vector2 rvo_goal = sim->getAgentGoal(i);
        rmtt_orca_state[i].goal[0] = rvo_goal.x();
        rmtt_orca_state[i].goal[1] = rvo_goal.y();
        RVO::Vector2 vel = sim->getAgentVelCMD(i);
        rmtt_orca_state[i].vel_orca[0] = cmd_vel[i].linear.x;
        rmtt_orca_state[i].vel_orca[1] = cmd_vel[i].linear.y;
        rmtt_orca_state_pub[i].publish(rmtt_orca_state[i]);
    }

    return arrived_all_goal;
}

void RMTT_ORCA::setup_agents()
{
    // 设置算法参数
	// sim->setAgentDefaults(5.0f, 10, 2.0f, 2.0f, 0.5f, 0.5f);
    sim->setAgentDefaults(rmtt_orca_param.neighborDist, rmtt_orca_param.maxNeighbors, rmtt_orca_param.timeHorizon, 
                    rmtt_orca_param.timeHorizonObst, rmtt_orca_param.radius, rmtt_orca_param.maxSpeed);
	// 设置时间间隔（这个似乎没有用？）
	sim->setTimeStep(rmtt_orca_param.time_step);
	// 添加智能体
	for (int i = 0; i < rmtt_num; i++) 
	{	
		RVO::Vector2 pos = RVO::Vector2(rmtt_state[i].pos[0],rmtt_state[i].pos[1]);
		sim->addAgent(pos);	
        cout << BLUE << node_name << ": ORCA add agents_" << i+1 << " at [" << rmtt_state[i].pos[0] << "," << rmtt_state[i].pos[1] << "]"<< TAIL << endl;
	}

    cout << BLUE << node_name << ": Set agents success!" << TAIL << endl;
}

void RMTT_ORCA::setup_obstacles()
{
	// 声明障碍物（凸多边形），障碍物建立规则：逆时针依次添加多边形的顶点
	std::vector<RVO::Vector2> obstacle1, obstacle2, obstacle3, obstacle4;

	// 障碍物示例：中心在原点，边长为1的正方体
	// obstacle1.push_back(RVO::Vector2(0.5f, 0.5f));
	// obstacle1.push_back(RVO::Vector2(-0.5f, 0.5f));
	// obstacle1.push_back(RVO::Vector2(-0.5f, 0.5f));
	// obstacle1.push_back(RVO::Vector2(0.5f, -0.5f));

	obstacle2.push_back(RVO::Vector2(10.0f, 40.0f));
	obstacle2.push_back(RVO::Vector2(10.0f, 10.0f));
	obstacle2.push_back(RVO::Vector2(40.0f, 10.0f));
	obstacle2.push_back(RVO::Vector2(40.0f, 40.0f));

	// obstacle3.push_back(RVO::Vector2(10.0f, -40.0f));
	// obstacle3.push_back(RVO::Vector2(40.0f, -40.0f));
	// obstacle3.push_back(RVO::Vector2(40.0f, -10.0f));
	// obstacle3.push_back(RVO::Vector2(10.0f, -10.0f));

	// obstacle4.push_back(RVO::Vector2(-10.0f, -40.0f));
	// obstacle4.push_back(RVO::Vector2(-10.0f, -10.0f));
	// obstacle4.push_back(RVO::Vector2(-40.0f, -10.0f));
	// obstacle4.push_back(RVO::Vector2(-40.0f, -40.0f));
    
    // 在算法中添加障碍物
	// sim->addObstacle(obstacle1);
	sim->addObstacle(obstacle2);
	// sim->addObstacle(obstacle3);
	// sim->addObstacle(obstacle4);

	// 在算法中处理障碍物信息
	sim->processObstacles();

    cout << BLUE << node_name << ":  Set obstacles success!" << TAIL << endl;
}

void RMTT_ORCA::debugCb(const ros::TimerEvent &e)
{
    //固定的浮点显示
    cout.setf(ios::fixed);
    // setprecision(n) 设显示小数精度为n位
    cout << setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);

    cout << BLUE << ">>>>>>>>>>>>>>>>>>>>>>>> RMTT_ORCA Node  <<<<<<<<<<<<<<<<<<<<<<<<" << TAIL << endl;
    for(int i = 0; i < rmtt_num; i++) 
    {
        cout << GREEN << "RMTT_ID: " << i+1 << TAIL << endl;

        if(arrived_goal[i])
        {
            cout << GREEN << "Arrived_goal : [ true ]" << TAIL << endl;
        }else
        {
            cout << RED << "Arrived_goal : [ false ]" << TAIL << endl;
        }
        // 打印
        RVO::Vector2 rvo_goal = sim->getAgentGoal(i);
        cout << GREEN << "GOAL [X Y Z]  : " << rvo_goal.x() << " [ m ] " << rvo_goal.y()<< " [ m ] " << TAIL << endl;
        cout << GREEN << "CMD  [X Y Z]  : " << cmd_vel[i].linear.x << " [m/s] " << cmd_vel[i].linear.y << " [m/s] " << cmd_vel[i].linear.z << " [m/s] " << TAIL << endl;
        cout << GREEN << "CMD  [ Yaw ]  : " << cmd_vel[i].angular.z * 180 / M_PI<< " [deg/s] " << TAIL << endl;
    }
}

bool RMTT_ORCA::reachedGoal(int i)
{
    bool xy_arrived{false}, z_arrived{false}, yaw_arrived{false};
    RVO::Vector2 rvo_goal = sim->getAgentGoal(i);
    float xy_distance = (rmtt_state[i].pos[0] - rvo_goal.x())*(rmtt_state[i].pos[0] - rvo_goal.x()) + (rmtt_state[i].pos[1] - rvo_goal.y())*(rmtt_state[i].pos[1] - rvo_goal.y()); 
	if (xy_distance < 0.15f * 0.15f) 
	{
        xy_arrived = true;
	}

    if(abs(rmtt_state[i].pos[2] - rmtt_height) < 0.08f)
    {
        z_arrived = true;
    }

    if(abs(rmtt_state[i].att[2] - 0.0) /M_PI *180< 3.0f)
    {
        yaw_arrived = true;
    }

    if(xy_arrived && z_arrived && yaw_arrived)
    {
	    return true;
    }
    return false;
}

void RMTT_ORCA::rmtt_state_cb(const sunray_msgs::rmtt_state::ConstPtr& msg, int i)
{
    rmtt_state[i] = *msg;
}

void RMTT_ORCA::rmtt_goal_cb(const geometry_msgs::Point::ConstPtr& msg, int i)
{
    arrived_goal[i] = false;
    arrived_all_goal = false;    
    rmtt_goal[i] = *msg;
    // 高度固定
    rmtt_goal[i].z = rmtt_height;

    goal_reached_printed[i] = false;

    sim->setAgentGoal(i, RVO::Vector2(rmtt_goal[i].x, rmtt_goal[i].y));
    cout << BLUE << node_name << ": Set agents_" << i+1 << " goal at [" << rmtt_goal[i].x << "," << rmtt_goal[i].y << "]"<< TAIL << endl;
    text_info.data = "[RMTT_ORCA] Set agents_"+std::to_string(i+1)+" goal";
    text_info_pub.publish(text_info);
}

void RMTT_ORCA::station_cmd_cb(const sunray_msgs::station_cmd::ConstPtr& msg)
{
    current_station_cmd = *msg; // 更新 current_station_cmd

    // 一键起飞的时候，才会设置home点
    if(msg->mission_state == MISSION_STATE::TAKEOFF && msg->uav_id == 99 && !start_flag)
    {
        start_flag = true;
        // 记录home点
        for(int i = 0; i < rmtt_num; i++) 
        {
            home_point[i].x = rmtt_state[i].pos[0];
            home_point[i].y = rmtt_state[i].pos[1];
            home_point[i].z = rmtt_height;
        }
        // ORCA算法初始化 - 添加当前为目标点（意味着起飞后无人机已经抵达对应目标点）
        setup_init_goals();
        cout << BLUE << node_name << ":  ORCA start" << TAIL << endl;
        text_info.data = "ORCA start"; 
        text_info_pub.publish(text_info);
    }

    if(msg->mission_state == MISSION_STATE::RETURN_HOME && msg->uav_id == 99)
    {
        // 初始化arrived_goal
        for(int i = 0; i < rmtt_num; i++) 
        {
            arrived_goal[i] = false;
        }
        arrived_all_goal = false;

        // 将home点设置为目标点
        goals.clear();
        for (int i = 0; i < rmtt_num; i++)  
        {  
            goals.push_back(RVO::Vector2(home_point[i].x, home_point[i].y));
        }

        for (int i = 0; i < rmtt_num; i++)  
        {   
            if (i < goals.size()) 
            {
                sim->setAgentGoal(i, goals[i]);
                cout << BLUE << node_name << ":  Set agents_" << i+1 << " goal at [" << goals[i].x() << "," << goals[i].y() << "]"<< TAIL << endl;
            }
            goal_reached_printed[i] = false;
        }
        cout << BLUE << node_name << ":  Get station_cmd: RETURN_HOME" << TAIL << endl;
        text_info.data = "Get station_cmd: RETURN_HOME"; 
        text_info_pub.publish(text_info);
    }

    // 一次性设置所有目标点
    if(msg->mission_state == MISSION_STATE::ORCA_SETUP && msg->uav_id == 99)
    {
        // 初始化arrived_goal
        for(int i = 0; i < rmtt_num; i++) 
        {
            arrived_goal[i] = false;
        }
        arrived_all_goal = false;

        if(msg->scenario_id == 1)
        {
            // 场景1的目标点设置
            setup_scenario_1();
            cout << BLUE << node_name << ":  Get station_cmd: ORCA_SETUP, ID: " << msg->scenario_id << TAIL << endl;
            text_info.data = "Received station_cmd: ORCA_SETUP, ID: 1"; 
        }
        else if(msg->scenario_id == 2)
        {
            // 场景2的目标点设置
            setup_scenario_2();
            cout << BLUE << node_name << ":  Get station_cmd: ORCA_SETUP, ID: " << msg->scenario_id << TAIL << endl;
            text_info.data = "Received station_cmd: ORCA_SETUP, ID: 2"; 
        }
        else if(msg->scenario_id == 3)
        {
            // 场景3的目标点设置
            setup_scenario_3();
            cout << BLUE << node_name << ":  Get station_cmd: ORCA_SETUP, ID: " << msg->scenario_id << TAIL << endl;
            text_info.data = "Received station_cmd: ORCA_SETUP, ID: 3"; 
        }
        else if(msg->scenario_id == 4)
        {
            // 场景4的目标点设置
            setup_scenario_4();
            cout << BLUE << node_name << ":  Get station_cmd: ORCA_SETUP, ID: " << msg->scenario_id << TAIL << endl;
            text_info.data = "Received station_cmd: ORCA_SETUP, ID: 4"; 
        }
        else if(msg->scenario_id == 5)
        {
            // 场景5的目标点设置
            setup_scenario_5();
            cout << BLUE << node_name << ":  Get station_cmd: ORCA_SETUP, ID: " << msg->scenario_id << TAIL << endl;
            text_info.data = "Received station_cmd: ORCA_SETUP, ID: 5"; 
        }else if(msg->scenario_id == 99)
        {
            // 自定义，来自goal话题
            goals.clear();
            for (int i = 0; i < rmtt_num; i++)  
            {  
                goals.push_back(RVO::Vector2(rmtt_goal[i].x, rmtt_goal[i].y));
            }

            for (int i = 0; i < rmtt_num; i++)  
            {   
                if (i < goals.size()) 
                {
                    sim->setAgentGoal(i, goals[i]);
                    cout << BLUE << node_name << ":  Set agents_" << i+1 << " goal at [" << goals[i].x() << "," << goals[i].y() << "]"<< TAIL << endl;
                }
                goal_reached_printed[i] = false;
            }
            cout << BLUE << node_name << ":  Get station_cmd: ORCA_SETUP, ID: " << msg->scenario_id << TAIL << endl;
            text_info.data = "Received station_cmd: ORCA_SETUP, ID: 99"; 
        }
        else
        {
            cout << RED << node_name << ": Invalid scenario_id: " << msg->scenario_id << TAIL << endl;
            text_info.data = "Received station_cmd: ORCA_SETUP, Invalid ID"; 
        }
        text_info_pub.publish(text_info);
    }
}

void RMTT_ORCA::printf_param()
{
    cout << GREEN << ">>>>>>>>>>>>>>>>>>> RMTT_ORCA Parameters <<<<<<<<<<<<<<<<" << TAIL << endl;
    // 无人机数量和飞行高度
    cout << GREEN << "rmtt_num : " << rmtt_num << TAIL << endl;
    // ORCA算法参数
    cout << GREEN << "neighborDist : " << rmtt_orca_param.neighborDist << " [m]" << TAIL << endl;
    cout << GREEN << "maxNeighbors : " << rmtt_orca_param.maxNeighbors << TAIL << endl;
    cout << GREEN << "timeHorizon : " << rmtt_orca_param.timeHorizon << " [s]" << TAIL << endl;
    cout << GREEN << "timeHorizonObst : " << rmtt_orca_param.timeHorizonObst << " [s]" << TAIL << endl;
    cout << GREEN << "radius : " << rmtt_orca_param.radius << " [m]" << TAIL << endl;
    cout << GREEN << "maxSpeed : " << rmtt_orca_param.maxSpeed << " [m/s]" << TAIL << endl;
    cout << GREEN << "time_step : " << rmtt_orca_param.time_step << " [s]" << TAIL << endl;
}

void RMTT_ORCA::setup_init_goals()
{
    // 容器清零
    goals.clear();
    RVO::Vector2 rmtt_set_goal;

    for(int i = 0; i < rmtt_num; i++) 
    {
        rmtt_goal[i].x = rmtt_state[i].pos[0];
        rmtt_goal[i].y = rmtt_state[i].pos[1];
        rmtt_goal[i].z = rmtt_height;
        rmtt_set_goal = RVO::Vector2(rmtt_goal[i].x, rmtt_goal[i].y);
        goals.push_back(rmtt_set_goal);
    }

    for (int i = 0; i < rmtt_num; i++)  
    {   
        if (i < goals.size()) {
            sim->setAgentGoal(i, goals[i]);
            cout << BLUE << node_name << ":  Set agents_" << i+1 << " init goal at [" << goals[i].x() << "," << goals[i].y() << "]"<< TAIL << endl;
        }
        goal_reached_printed[i] = false;
    }

    cout << BLUE << node_name << ":  set init goals success!" << TAIL << endl;
}

void RMTT_ORCA::setup_scenario_1()
{
    // 容器清零
    goals.clear();
    RVO::Vector2 rmtt_set_goal;

    // 设置预设目标点 rmtt_1 -> rmtt_2
    rmtt_goal[0].x = 1.5;
    rmtt_goal[0].y = 1.5;
    rmtt_goal[0].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[0].x, rmtt_goal[0].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[1].x = 1.5;
    rmtt_goal[1].y = 0.75;
    rmtt_goal[1].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[1].x, rmtt_goal[1].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[2].x = 1.5;
    rmtt_goal[2].y = -0.75;
    rmtt_goal[2].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[2].x, rmtt_goal[2].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[3].x = 1.5;
    rmtt_goal[3].y = -1.5;
    rmtt_goal[3].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[3].x, rmtt_goal[3].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[4].x = -1.5;
    rmtt_goal[4].y = 1.5;
    rmtt_goal[4].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[4].x, rmtt_goal[4].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[5].x = -1.5;
    rmtt_goal[5].y = 0.75;
    rmtt_goal[5].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[5].x, rmtt_goal[5].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[6].x = -1.5;
    rmtt_goal[6].y = -0.75;
    rmtt_goal[6].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[6].x, rmtt_goal[6].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[7].x = -1.5;
    rmtt_goal[7].y = -1.5;
    rmtt_goal[7].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[7].x, rmtt_goal[7].y);
    goals.push_back(rmtt_set_goal);

    for (int i = 0; i < rmtt_num; i++)  
    {   
        if (i < goals.size()) {
            sim->setAgentGoal(i, goals[i]);
            cout << BLUE << node_name << ":  Set agents_" << i+1 << " goal at [" << goals[i].x() << "," << goals[i].y() << "]"<< TAIL << endl;
        }
        goal_reached_printed[i] = false;
    }

    cout << BLUE << node_name << ":  setup_scenario_1 goals success!" << TAIL << endl;
}

void RMTT_ORCA::setup_scenario_2()
{
    // 容器清零
    goals.clear();
    RVO::Vector2 rmtt_set_goal;

    // 设置预设目标点 rmtt_1 -> rmtt_2
    rmtt_goal[0].x = 1.5;
    rmtt_goal[0].y = 1.5;
    rmtt_goal[0].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[0].x, rmtt_goal[0].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[1].x = 1.5;
    rmtt_goal[1].y = 0.75;
    rmtt_goal[1].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[1].x, rmtt_goal[1].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[2].x = 1.5;
    rmtt_goal[2].y = -0.75;
    rmtt_goal[2].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[2].x, rmtt_goal[2].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[3].x = 1.5;
    rmtt_goal[3].y = -1.5;
    rmtt_goal[3].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[3].x, rmtt_goal[3].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[4].x = -1.5;
    rmtt_goal[4].y = -1.5;
    rmtt_goal[4].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[4].x, rmtt_goal[4].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[5].x = -1.5;
    rmtt_goal[5].y = 0.75;
    rmtt_goal[5].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[5].x, rmtt_goal[5].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[6].x = -1.5;
    rmtt_goal[6].y = -0.75;
    rmtt_goal[6].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[6].x, rmtt_goal[6].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[7].x = -1.5;
    rmtt_goal[7].y = 1.5;
    rmtt_goal[7].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[7].x, rmtt_goal[7].y);
    goals.push_back(rmtt_set_goal);


    for (int i = 0; i < rmtt_num; i++)  
    {   
        if (i < goals.size()) {
            sim->setAgentGoal(i, goals[i]);
            cout << BLUE << node_name << ":  Set agents_" << i+1 << " goal at [" << goals[i].x() << "," << goals[i].y() << "]"<< TAIL << endl;
        }
        goal_reached_printed[i] = false;
    }

    cout << BLUE << node_name << ":  setup_scenario_2 goals success!" << TAIL << endl;
}

void RMTT_ORCA::setup_scenario_3()
{
    // 容器清零
    goals.clear();
    RVO::Vector2 rmtt_set_goal;

    // 设置预设目标点 rmtt_1 -> rmtt_8
    rmtt_goal[0].x = -1.5;
    rmtt_goal[0].y = 1.5;
    rmtt_goal[0].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[0].x, rmtt_goal[0].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[1].x = 1.5;
    rmtt_goal[1].y = 1.5;
    rmtt_goal[1].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[1].x, rmtt_goal[1].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[2].x = 1.5;
    rmtt_goal[2].y = -1.5;
    rmtt_goal[2].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[2].x, rmtt_goal[2].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[3].x = -1.5;
    rmtt_goal[3].y = -1.5;
    rmtt_goal[3].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[3].x, rmtt_goal[3].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[4].x = -1.5;
    rmtt_goal[4].y = 0.0;
    rmtt_goal[4].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[4].x, rmtt_goal[4].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[5].x = 0.0;
    rmtt_goal[5].y = 1.5;
    rmtt_goal[5].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[5].x, rmtt_goal[5].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[6].x = 1.5;
    rmtt_goal[6].y = 0.0;
    rmtt_goal[6].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[6].x, rmtt_goal[6].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[7].x = 0.0;
    rmtt_goal[7].y = -1.5;
    rmtt_goal[7].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[7].x, rmtt_goal[7].y);
    goals.push_back(rmtt_set_goal);

    for (int i = 0; i < rmtt_num; i++)  
    {   
        if (i < goals.size()) {
            sim->setAgentGoal(i, goals[i]);
            cout << BLUE << node_name << ":  Set agents_" << i+1 << " goal at [" << goals[i].x() << "," << goals[i].y() << "]"<< TAIL << endl;
        }
        goal_reached_printed[i] = false;
    }

    cout << BLUE << node_name << ":  setup_scenario_3 goals success!" << TAIL << endl;
}

void RMTT_ORCA::setup_scenario_4()
{
    // 容器清零
    goals.clear();
    RVO::Vector2 rmtt_set_goal;

    // 设置预设目标点 rmtt_1 -> rmtt_2
    rmtt_goal[0].x = 1.5;
    rmtt_goal[0].y = -1.5;
    rmtt_goal[0].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[0].x, rmtt_goal[0].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[1].x = 1.5;
    rmtt_goal[1].y = 1.5;
    rmtt_goal[1].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[1].x, rmtt_goal[1].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[2].x = 1.5;
    rmtt_goal[2].y = 0.75;
    rmtt_goal[2].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[2].x, rmtt_goal[2].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[3].x = 1.5;
    rmtt_goal[3].y = -0.75;
    rmtt_goal[3].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[3].x, rmtt_goal[3].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[4].x = -1.5;
    rmtt_goal[4].y = 0.75;
    rmtt_goal[4].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[4].x, rmtt_goal[4].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[5].x = -1.5;
    rmtt_goal[5].y = -0.75;
    rmtt_goal[5].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[5].x, rmtt_goal[5].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[6].x = -1.5;
    rmtt_goal[6].y = -1.5;
    rmtt_goal[6].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[6].x, rmtt_goal[6].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[7].x = -1.5;
    rmtt_goal[7].y = 1.5;
    rmtt_goal[7].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[7].x, rmtt_goal[7].y);
    goals.push_back(rmtt_set_goal);

    for (int i = 0; i < rmtt_num; i++)  
    {   
        if (i < goals.size()) {
            sim->setAgentGoal(i, goals[i]);
            cout << BLUE << node_name << ":  Set agents_" << i+1 << " goal at [" << goals[i].x() << "," << goals[i].y() << "]"<< TAIL << endl;
        }
        goal_reached_printed[i] = false;
    }

    cout << BLUE << node_name << ":  setup_scenario_4 goals success!" << TAIL << endl;
}

void RMTT_ORCA::setup_scenario_5()
{
    // 容器清零
    goals.clear();
    RVO::Vector2 rmtt_set_goal;


    // 设置预设目标点 rmtt_1 -> rmtt_2
    rmtt_goal[0].x = 1.5;
    rmtt_goal[0].y = -1.5;
    rmtt_goal[0].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[0].x, rmtt_goal[0].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[1].x = 1.5;
    rmtt_goal[1].y = -0.75;
    rmtt_goal[1].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[1].x, rmtt_goal[1].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[2].x = 1.5;
    rmtt_goal[2].y = 0.75;
    rmtt_goal[2].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[2].x, rmtt_goal[2].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[3].x = 1.5;
    rmtt_goal[3].y = 1.5;
    rmtt_goal[3].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[3].x, rmtt_goal[3].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[4].x = 0;
    rmtt_goal[4].y = -1.5;
    rmtt_goal[4].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[4].x, rmtt_goal[4].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[5].x = -1.5;
    rmtt_goal[5].y = -0.75;
    rmtt_goal[5].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[5].x, rmtt_goal[5].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[6].x = -1.5;
    rmtt_goal[6].y = 0.75;
    rmtt_goal[6].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[6].x, rmtt_goal[6].y);
    goals.push_back(rmtt_set_goal);

    rmtt_goal[7].x = -1.5;
    rmtt_goal[7].y = 1.5;
    rmtt_goal[7].z = rmtt_height;
    rmtt_set_goal = RVO::Vector2(rmtt_goal[7].x, rmtt_goal[7].y);
    goals.push_back(rmtt_set_goal);
    for (int i = 0; i < rmtt_num; i++)  
    {   
        if (i < goals.size()) {
            sim->setAgentGoal(i, goals[i]);
            cout << BLUE << node_name << ":  Set agents_" << i+1 << " goal at [" << goals[i].x() << "," << goals[i].y() << "]"<< TAIL << endl;
        }
        goal_reached_printed[i] = false;
    }

    cout << BLUE << node_name << ":  setup_scenario_5 goals success!" << TAIL << endl;
}
