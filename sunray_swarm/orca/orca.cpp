#include "orca.h"

void ORCA::init(ros::NodeHandle& nh)
{
    // 【参数】智能体类型
    nh.param<int>("agent_type", agent_type, 1);
    // 【参数】智能体数量
    nh.param<int>("agent_num", agent_num, 8);
    // 【参数】智能体高度
    nh.param<float>("agent_height", agent_height, 1.0);
    // 【参数】无人机之间的假想感知距离
    nh.param<float>("orca_params/neighborDist", orca_params.neighborDist, 1.5);
    orca_params.maxNeighbors = agent_num;
    // 【参数】数字越大，无人机响应相邻无人机的碰撞越快，但速度可选的自由度越小
    nh.param<float>("orca_params/timeHorizon", orca_params.timeHorizon, 2.0);
    // 【参数】数字越大，无人机响应障碍物的碰撞越快，但速度可选的自由度越小
    nh.param<float>("orca_params/timeHorizonObst", orca_params.timeHorizonObst, 2.0);
    // 【参数】无人机体积半径
    nh.param<float>("orca_params/radius", orca_params.radius, 0.3);
    // 【参数】无人机最大速度
    nh.param<float>("orca_params/maxSpeed", orca_params.maxSpeed, 0.5);
    // 【参数】时间步长 不确定有啥用
    nh.param<float>("orca_params/time_step", orca_params.time_step, 0.1);

    // 初始化订阅器和发布器
    // 【发布】文字提示消息（回传至地面站显示）
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);
    // 初始化 goal_reached_printed
    goal_reached_printed.resize(agent_num, false);  

    string agent_name;
    if(agent_type == sunray_msgs::agent_state::RMTT)
    {
        agent_prefix = "/rmtt_";
    }else if(agent_type == sunray_msgs::agent_state::TIANBOT)
    {
        agent_prefix = "/tianbot_";
    }else if(agent_type == sunray_msgs::agent_state::WHEELTEC)
    {
        agent_prefix = "/wheeltec_";
    }else if(agent_type == sunray_msgs::agent_state::SIKONG)
    {
        agent_prefix = "/sikong_";
    }else
    {
        agent_prefix = "/unkonown_";
    }

    // 【订阅】ORCA指令 外部 -> 本节点
    orca_cmd_sub = nh.subscribe<sunray_msgs::orca_cmd>("/sunray_swarm" + agent_prefix + "/orca_cmd", 10, &ORCA::orca_cmd_cb, this);

    for(int i = 0; i < agent_num; i++) 
    {
        agent_name = agent_prefix + std::to_string(i+1);
        // 【订阅】无人机状态数据
        agent_state_sub[i] = nh.subscribe<sunray_msgs::agent_state>("/sunray_swarm" + agent_name + "/agent_state", 10, boost::bind(&ORCA::agent_state_cb,this ,_1,i));
        // 【订阅】无人机的目标点
        agent_goal_sub[i] = nh.subscribe<geometry_msgs::Point>("/sunray_swarm" + agent_name + "/goal_point", 10, boost::bind(&ORCA::agent_goal_cb,this ,_1,i));
        // 【发布】智能体控制指令
		agent_cmd_pub[i] = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 1); 
        // 【发布】无人机orca状态 回传地面站
		agent_orca_state_pub[i] = nh.advertise<sunray_msgs::orca_state>("/sunray_swarm" + agent_name + "/agent_orca_state", 1);
        // 【发布】目标点marker 本节点 -> RVIZ
        goal_point_pub[i] = nh.advertise<visualization_msgs::Marker>("/sunray_swarm" + agent_name + "/goal_point_rviz", 1);   
    }

    // 打印定时器
    debug_timer = nh.createTimer(ros::Duration(10.0), &ORCA::debugCb, this);

    for(int i = 0; i < agent_num; i++) 
    {
        arrived_goal[i] = false;
    }
    arrived_all_goal = false;     

    // 【函数】打印参数
    printf_param();

    // ORCA算法初始化 - 添加智能体
    setup_agents();
    // ORCA算法初始化 - 添加障碍物
    // setup_obstacles();

    setup_color();

    node_name = ros::this_node::getName();
    text_info.data = node_name + ": ORCA init!";
    text_info_pub.publish(text_info);
    cout << BLUE << text_info.data << TAIL << endl;
}

bool ORCA::orca_run()
{
    if(!start_flag)
    {
        return false;
    }

    arrived_all_goal = true;

    // 判断是否达到目标点附近
	for(int i = 0; i < agent_num; ++i) 
	{	
		if(!arrived_goal[i])
		{
			arrived_goal[i] = reachedGoal(i);
		}
    }

	// 更新RVO中的位置和速度
	for(int i = 0; i < agent_num; ++i) 
	{	
		RVO::Vector2 pos = RVO::Vector2(agent_state[i].pos[0],agent_state[i].pos[1]);
		sim->setAgentPosition(i, pos);//更新RVO仿真中的位置
	}

	// 计算每一个智能体的期望速度
	sim->computeVel();

	for(int i = 0; i < agent_num; ++i) 
	{	
        // 如果达到目标点附近，则使用直接控制直接移动到目标点
		if(arrived_goal[i])
		{
            agent_cmd[i].agent_id = i+1;
            agent_cmd[i].control_state = sunray_msgs::agent_cmd::POS_CONTROL;
            agent_cmd[i].cmd_source = "ORCA";
            agent_cmd[i].desired_pos.x = sim->getAgentGoal(i).x();
            agent_cmd[i].desired_pos.y = sim->getAgentGoal(i).y();
            agent_cmd[i].desired_pos.z = agent_height; 
            agent_cmd[i].desired_yaw = 0.0;
            agent_cmd_pub[i].publish(agent_cmd[i]);
            if (!goal_reached_printed[i])
            {
                cout << BLUE << node_name << agent_prefix << i+1 << " Arrived." << TAIL << endl;
                text_info.data = agent_prefix + std::to_string(i+1) + " Arrived.";
                text_info_pub.publish(text_info);
                goal_reached_printed[i] = true;
            }
		}
        // 如果没有达到目标点附近，则使用ORCA算法计算期望速度
		else
		{
            // 获得期望速度 注：这个速度是ENU坐标系的，并将其转换为机体系速度指令
            RVO::Vector2 vel = sim->getAgentVelCMD(i);   
            agent_cmd[i].agent_id = i+1;
            agent_cmd[i].control_state = sunray_msgs::agent_cmd::VEL_CONTROL_ENU;
            agent_cmd[i].cmd_source = "ORCA";
            agent_cmd[i].desired_vel.linear.x = vel.x();
            agent_cmd[i].desired_vel.linear.y = vel.y();
            agent_cmd[i].desired_vel.linear.z = 0.0;
            agent_cmd[i].desired_vel.angular.x = 0.0;
            agent_cmd[i].desired_vel.angular.y = 0.0;
            agent_cmd[i].desired_vel.angular.z = 0.0; 
            agent_cmd[i].desired_yaw = 0.0;
            agent_cmd_pub[i].publish(agent_cmd[i]);
            // 只要有一个智能体未到达目标点，整体状态就是未完成
            arrived_all_goal = false;                   
        }
	}

    // 循环遍历每个智能体，更新并发布ORCA状态信息
    for(int i = 0; i < agent_num; i++) 
    {
        agent_orca_state[i].agent_num = agent_num;
        agent_orca_state[i].agent_id = i+1;
        agent_orca_state[i].arrived_goal = arrived_goal[i];
        agent_orca_state[i].arrived_all_goal = arrived_all_goal;
        RVO::Vector2 rvo_goal = sim->getAgentGoal(i);
        agent_orca_state[i].goal[0] = rvo_goal.x();
        agent_orca_state[i].goal[1] = rvo_goal.y();
        RVO::Vector2 vel = sim->getAgentVelCMD(i);
        agent_orca_state[i].vel_orca[0] = agent_cmd[i].desired_vel.linear.x;
        agent_orca_state[i].vel_orca[1] = agent_cmd[i].desired_vel.linear.y;
        agent_orca_state_pub[i].publish(agent_orca_state[i]);

        // 发布目标点mesh
        visualization_msgs::Marker goal_marker;
        goal_marker.header.frame_id = "world";
        goal_marker.header.stamp = ros::Time::now();
        goal_marker.ns = "goal";
        goal_marker.id = i+1;
        goal_marker.type = visualization_msgs::Marker::SPHERE;
        goal_marker.action = visualization_msgs::Marker::ADD;
        goal_marker.pose.position.x = agent_orca_state[i].goal[0];
        goal_marker.pose.position.y = agent_orca_state[i].goal[1];
        goal_marker.pose.position.z = agent_height;
        goal_marker.pose.orientation.x = 0.0;
        goal_marker.pose.orientation.y = 0.0;
        goal_marker.pose.orientation.z = 0.0;
        goal_marker.pose.orientation.w = 1.0;
        goal_marker.scale.x = 0.2;
        goal_marker.scale.y = 0.2;
        goal_marker.scale.z = 0.2;
        goal_marker.color = led_color[i];
        goal_marker.mesh_use_embedded_materials = false;
        goal_point_pub[i].publish(goal_marker);
    }

    return arrived_all_goal;
}

void ORCA::setup_agents()
{
    // 设置算法参数
	// sim->setAgentDefaults(1.5f, 10, 2.0f, 2.0f, 0.5f, 0.5f);
    // sim->setAgentDefaults(15.0f, 10, 5.0f, 5.0f, 2.0f, 2.0f);
    sim->setAgentDefaults(orca_params.neighborDist, orca_params.maxNeighbors, orca_params.timeHorizon, 
                    orca_params.timeHorizonObst, orca_params.radius, orca_params.maxSpeed);
	// 设置时间间隔（这个似乎没有用？）
	sim->setTimeStep(orca_params.time_step);
	// 添加智能体
	for (int i = 0; i < agent_num; i++) 
	{	
		RVO::Vector2 pos = RVO::Vector2(agent_state[i].pos[0],agent_state[i].pos[1]);
		sim->addAgent(pos);	
        cout << BLUE << node_name << ": ORCA add agents_" << i+1 << " at [" << agent_state[i].pos[0] << "," << agent_state[i].pos[1] << "]"<< TAIL << endl;
	}

    cout << BLUE << node_name << ": Set agents success!" << TAIL << endl;
}

void ORCA::setup_obstacles()
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

void ORCA::debugCb(const ros::TimerEvent &e)
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

    cout << BLUE << ">>>>>>>>>>>>>>>>>>>>>>>> ORCA Node  <<<<<<<<<<<<<<<<<<<<<<<<" << TAIL << endl;
    for(int i = 0; i < agent_num; i++) 
    {
        cout << GREEN << ">>>>>>>>>>>>>>>>>>>>>>>> ORCA Agent" << i+1 << "<<<<<<<<<<<<<<<<<<<<" << TAIL << endl;

        if(agent_orca_state[i].arrived_goal)
        {
            cout << GREEN << "Arrived_goal : [ true ]" << TAIL << endl;
        }else
        {
            cout << RED   << "Arrived_goal : [ false ]" << TAIL << endl;
        }
        // 打印
        RVO::Vector2 rvo_goal = sim->getAgentGoal(i);
        cout << GREEN << "GOAL [X Y]  : " << agent_orca_state[i].goal[0] << " [ m ] " << agent_orca_state[i].goal[1] << " [ m ] " << TAIL << endl;
        cout << GREEN << "CMD  [X Y]  : " << agent_orca_state[i].vel_orca[0] << " [m/s] " << agent_orca_state[i].vel_orca[1] << " [m/s] " << TAIL << endl;
    }
}

bool ORCA::reachedGoal(int i)
{
    bool xy_arrived{false}, z_arrived{false}, yaw_arrived{false};
    RVO::Vector2 rvo_goal = sim->getAgentGoal(i);
    float xy_distance = (agent_state[i].pos[0] - rvo_goal.x())*(agent_state[i].pos[0] - rvo_goal.x()) + (agent_state[i].pos[1] - rvo_goal.y())*(agent_state[i].pos[1] - rvo_goal.y()); 
	if (xy_distance < 0.15f * 0.15f) 
	{
        xy_arrived = true;
	}

    if(abs(agent_state[i].pos[2] - agent_height) < 0.08f)
    {
        z_arrived = true;
    }

    if(abs(agent_state[i].att[2] - 0.0) /M_PI *180< 3.0f)
    {
        yaw_arrived = true;
    }

    if(xy_arrived && z_arrived && yaw_arrived)
    {
	    return true;
    }
    return false;
}

void ORCA::agent_state_cb(const sunray_msgs::agent_state::ConstPtr& msg, int i)
{
    agent_state[i] = *msg;
}

void ORCA::agent_goal_cb(const geometry_msgs::Point::ConstPtr& msg, int i)
{
    arrived_goal[i] = false;
    arrived_all_goal = false;    
    agent_goal[i] = *msg;
    // 高度固定
    agent_goal[i].z = agent_height;

    goal_reached_printed[i] = false;

    sim->setAgentGoal(i, RVO::Vector2(agent_goal[i].x, agent_goal[i].y));
    cout << BLUE << node_name << ": Set agents_" << i+1 << " goal at [" << agent_goal[i].x << "," << agent_goal[i].y << "]"<< TAIL << endl;
    text_info.data = "[ORCA] Set agents_"+std::to_string(i+1)+" goal";
    text_info_pub.publish(text_info);
}

void ORCA::orca_cmd_cb(const sunray_msgs::orca_cmd::ConstPtr& msg)
{
    // 设置home点
    if(msg->orca_cmd == sunray_msgs::orca_cmd::SET_HOME)
    {
        start_flag = true;
        // 记录home点
        for(int i = 0; i < agent_num; i++) 
        {
            home_point[i].x = agent_state[i].pos[0];
            home_point[i].y = agent_state[i].pos[1];
            home_point[i].z = agent_height;
        }
        // ORCA算法初始化 - 添加当前为目标点（意味着起飞后无人机已经抵达对应目标点）
        setup_init_goals();
        text_info.data = "Get orca_cmd: SET_HOME, ORCA start!"; 
        cout << BLUE << node_name << text_info.data << TAIL << endl;
    }

    if(msg->orca_cmd == sunray_msgs::orca_cmd::RETURN_HOME)
    {
        // 初始化arrived_goal
        for(int i = 0; i < agent_num; i++) 
        {
            arrived_goal[i] = false;
        }
        arrived_all_goal = false;

        // 将home点设置为目标点
        goals.clear();
        for (int i = 0; i < agent_num; i++)  
        {  
            goals.push_back(RVO::Vector2(home_point[i].x, home_point[i].y));
        }

        for (int i = 0; i < agent_num; i++)  
        {   
            if (i < goals.size()) 
            {
                sim->setAgentGoal(i, goals[i]);
                cout << BLUE << node_name << ":  Set agents_" << i+1 << " goal at [" << goals[i].x() << "," << goals[i].y() << "]"<< TAIL << endl;
            }
            goal_reached_printed[i] = false;
        }
        text_info.data = "Get orca_cmd: RETURN_HOME"; 
        cout << BLUE << node_name << text_info.data << TAIL << endl;
    }

    if(msg->orca_cmd == sunray_msgs::orca_cmd::SETUP_OBS)
    {
	    std::vector<RVO::Vector2> obstacle;

        for(int i = 0; i < msg->obs_point.size(); i++) 
        {
            obstacle.push_back(RVO::Vector2(msg->obs_point[i].x, msg->obs_point[i].y));
            cout << BLUE << node_name << "Add obstacle point_"<< i << " at ["<< msg->obs_point[i].x << "," << msg->obs_point[i].y << "]"<< TAIL << endl;
        }
        // 在算法中添加障碍物
        sim->addObstacle(obstacle);
        // 在算法中处理障碍物信息
        sim->processObstacles();

        text_info.data = "Received orca_cmd: SETUP_OBS"; 
        cout << BLUE << node_name << text_info.data << TAIL << endl;
    }

    if(msg->orca_cmd == sunray_msgs::orca_cmd::ORCA_SCENARIO_1)
    {
        for(int i = 0; i < agent_num; i++) 
        {
            arrived_goal[i] = false;
        }
        arrived_all_goal = false;
        setup_scenario_1();
        text_info.data = "Received orca_cmd: ORCA_SCENARIO_1"; 
        cout << BLUE << node_name << text_info.data << TAIL << endl;
    }

    if(msg->orca_cmd == sunray_msgs::orca_cmd::ORCA_SCENARIO_2)
    {
        for(int i = 0; i < agent_num; i++) 
        {
            arrived_goal[i] = false;
        }
        arrived_all_goal = false;
        setup_scenario_2();
        text_info.data = "Received orca_cmd: ORCA_SCENARIO_2"; 
        cout << BLUE << node_name << text_info.data << TAIL << endl;
    }

    if(msg->orca_cmd == sunray_msgs::orca_cmd::ORCA_SCENARIO_3)
    {
        for(int i = 0; i < agent_num; i++) 
        {
            arrived_goal[i] = false;
        }
        arrived_all_goal = false;
        setup_scenario_3();
        text_info.data = "Received orca_cmd: ORCA_SCENARIO_3"; 
        cout << BLUE << node_name << text_info.data << TAIL << endl;
    }

    if(msg->orca_cmd == sunray_msgs::orca_cmd::ORCA_SCENARIO_4)
    {
        for(int i = 0; i < agent_num; i++) 
        {
            arrived_goal[i] = false;
        }
        arrived_all_goal = false;
        setup_scenario_4();
        text_info.data = "Received orca_cmd: ORCA_SCENARIO_4"; 
        cout << BLUE << node_name << text_info.data << TAIL << endl;
    }

    if(msg->orca_cmd == sunray_msgs::orca_cmd::ORCA_SCENARIO_5)
    {
        for(int i = 0; i < agent_num; i++) 
        {
            arrived_goal[i] = false;
        }
        arrived_all_goal = false;
        setup_scenario_5();
        text_info.data = "Received orca_cmd: ORCA_SCENARIO_5"; 
        cout << BLUE << node_name << text_info.data << TAIL << endl;
    }

    text_info_pub.publish(text_info); 
}

void ORCA::printf_param()
{
    cout << GREEN << ">>>>>>>>>>>>>>>>>>> ORCA Parameters <<<<<<<<<<<<<<<<" << TAIL << endl;
    cout << GREEN << "agent_num    : " << agent_num << TAIL << endl;
    cout << GREEN << "agent_height : " << agent_height << TAIL << endl;
    if(agent_type == sunray_msgs::agent_state::RMTT)
    {
        cout << GREEN << "agent_type : RMTT" << TAIL << endl;
    }else if(agent_type == sunray_msgs::agent_state::TIANBOT)
    {
        cout << GREEN << "agent_type : TIANBOT" << TAIL << endl;
    }else if(agent_type == sunray_msgs::agent_state::WHEELTEC)
    {
        cout << GREEN << "agent_type : WHEELTEC" << TAIL << endl;
    }else if(agent_type == sunray_msgs::agent_state::SIKONG)
    {
        cout << GREEN << "agent_type : SIKONG" << TAIL << endl;
    }else
    {
        cout << GREEN << "agent_type : UNKONWN" << TAIL << endl;
    }

    // ORCA算法参数
    cout << GREEN << "neighborDist : " << orca_params.neighborDist << " [m]" << TAIL << endl;
    cout << GREEN << "maxNeighbors : " << orca_params.maxNeighbors << TAIL << endl;
    cout << GREEN << "timeHorizon : " << orca_params.timeHorizon << " [s]" << TAIL << endl;
    cout << GREEN << "timeHorizonObst : " << orca_params.timeHorizonObst << " [s]" << TAIL << endl;
    cout << GREEN << "radius : " << orca_params.radius << " [m]" << TAIL << endl;
    cout << GREEN << "maxSpeed : " << orca_params.maxSpeed << " [m/s]" << TAIL << endl;
    cout << GREEN << "time_step : " << orca_params.time_step << " [s]" << TAIL << endl;
}

void ORCA::setup_init_goals()
{
    // 容器清零
    goals.clear();
    RVO::Vector2 agent_set_goal;

    for(int i = 0; i < agent_num; i++) 
    {
        agent_goal[i].x = agent_state[i].pos[0];
        agent_goal[i].y = agent_state[i].pos[1];
        agent_goal[i].z = agent_height;
        agent_set_goal = RVO::Vector2(agent_goal[i].x, agent_goal[i].y);
        goals.push_back(agent_set_goal);
    }

    for (int i = 0; i < agent_num; i++)  
    {   
        if (i < goals.size()) {
            sim->setAgentGoal(i, goals[i]);
            cout << BLUE << node_name << ":  Set agents_" << i+1 << " init goal at [" << goals[i].x() << "," << goals[i].y() << "]"<< TAIL << endl;
        }
        goal_reached_printed[i] = false;
    }

    cout << BLUE << node_name << ":  set init goals success!" << TAIL << endl;
}

void ORCA::setup_color()
{
    led_color[0].a = 1.0;
    led_color[0].r = 1.0;
    led_color[0].g = 0.0;
    led_color[0].b = 0.0;
    led_color[1].a = 1.0;
    led_color[1].r = 0.0;
    led_color[1].g = 1.0;
    led_color[1].b = 0.0;
    led_color[2].a = 1.0;
    led_color[2].r = 0.0;
    led_color[2].g = 0.0;
    led_color[2].b = 1.0;
    led_color[3].a = 1.0;
    led_color[3].r = 1.0;
    led_color[3].g = 1.0;
    led_color[3].b = 0.0;
    led_color[4].a = 1.0;
    led_color[4].r = 1.0;
    led_color[4].g = 0.0;
    led_color[4].b = 1.0;
    led_color[5].a = 1.0;
    led_color[5].r = 0.0;
    led_color[5].g = 1.0;
    led_color[5].b = 1.0;
    led_color[6].a = 1.0;
    led_color[6].r = 0.5;
    led_color[6].g = 0.5;
    led_color[6].b = 0.5;
    led_color[7].a = 1.0;
    led_color[7].r = 0.3;
    led_color[7].g = 0.7;
    led_color[7].b = 0.2;
}

void ORCA::setup_scenario_1()
{
    // 容器清零
    goals.clear();
    RVO::Vector2 agent_set_goal;

    // 设置预设目标点 agent_1 -> agent_2
    agent_goal[0].x = -0.85;
    agent_goal[0].y = 1.35;
    agent_goal[0].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[0].x, agent_goal[0].y);
    goals.push_back(agent_set_goal);

    agent_goal[1].x = 1.05;
    agent_goal[1].y = 1.35;
    agent_goal[1].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[1].x, agent_goal[1].y);
    goals.push_back(agent_set_goal);

    agent_goal[2].x = 0.1;
    agent_goal[2].y = -0.15;
    agent_goal[2].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[2].x, agent_goal[2].y);
    goals.push_back(agent_set_goal);

    agent_goal[3].x = 1.05;
    agent_goal[3].y = -1.25;
    agent_goal[3].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[3].x, agent_goal[3].y);
    goals.push_back(agent_set_goal);

    agent_goal[4].x = -1.0;
    agent_goal[4].y = -1.3;
    agent_goal[4].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[4].x, agent_goal[4].y);
    goals.push_back(agent_set_goal);

    agent_goal[5].x = -1.5;
    agent_goal[5].y = 0.75;
    agent_goal[5].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[5].x, agent_goal[5].y);
    goals.push_back(agent_set_goal);

    agent_goal[6].x = -1.5;
    agent_goal[6].y = -0.75;
    agent_goal[6].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[6].x, agent_goal[6].y);
    goals.push_back(agent_set_goal);

    agent_goal[7].x = -1.5;
    agent_goal[7].y = -1.5;
    agent_goal[7].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[7].x, agent_goal[7].y);
    goals.push_back(agent_set_goal);

    for (int i = 0; i < agent_num; i++)  
    {   
        if (i < goals.size()) {
            sim->setAgentGoal(i, goals[i]);
            cout << BLUE << node_name << ":  Set agents_" << i+1 << " goal at [" << goals[i].x() << "," << goals[i].y() << "]"<< TAIL << endl;
        }
        goal_reached_printed[i] = false;
    }

    cout << BLUE << node_name << ":  setup_scenario_1 goals success!" << TAIL << endl;
}

void ORCA::setup_scenario_2()
{
    // 容器清零
    goals.clear();
    RVO::Vector2 agent_set_goal;

    // 设置预设目标点 agent_1 -> agent_2
    agent_goal[0].x = 0.1;
    agent_goal[0].y = 1.35;
    agent_goal[0].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[0].x, agent_goal[0].y);
    goals.push_back(agent_set_goal);

    agent_goal[1].x = 1.4;
    agent_goal[1].y = 0.5;
    agent_goal[1].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[1].x, agent_goal[1].y);
    goals.push_back(agent_set_goal);

    agent_goal[2].x = -0.8;
    agent_goal[2].y = -0.0;
    agent_goal[2].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[2].x, agent_goal[2].y);
    goals.push_back(agent_set_goal);

    agent_goal[3].x = 1.4;
    agent_goal[3].y = -0.5;
    agent_goal[3].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[3].x, agent_goal[3].y);
    goals.push_back(agent_set_goal);

    agent_goal[4].x = 0.1;
    agent_goal[4].y = -1.35;
    agent_goal[4].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[4].x, agent_goal[4].y);
    goals.push_back(agent_set_goal);

    agent_goal[5].x = -1.5;
    agent_goal[5].y = 0.75;
    agent_goal[5].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[5].x, agent_goal[5].y);
    goals.push_back(agent_set_goal);

    agent_goal[6].x = -1.5;
    agent_goal[6].y = -0.75;
    agent_goal[6].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[6].x, agent_goal[6].y);
    goals.push_back(agent_set_goal);

    agent_goal[7].x = -1.5;
    agent_goal[7].y = 1.5;
    agent_goal[7].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[7].x, agent_goal[7].y);
    goals.push_back(agent_set_goal);


    for (int i = 0; i < agent_num; i++)  
    {   
        if (i < goals.size()) {
            sim->setAgentGoal(i, goals[i]);
            cout << BLUE << node_name << ":  Set agents_" << i+1 << " goal at [" << goals[i].x() << "," << goals[i].y() << "]"<< TAIL << endl;
        }
        goal_reached_printed[i] = false;
    }

    cout << BLUE << node_name << ":  setup_scenario_2 goals success!" << TAIL << endl;
}

void ORCA::setup_scenario_3()
{
    // 容器清零
    goals.clear();
    RVO::Vector2 agent_set_goal;

    // 设置预设目标点 agent_1 -> agent_8
    agent_goal[0].x = 0.1;
    agent_goal[0].y = 0.9;
    agent_goal[0].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[0].x, agent_goal[0].y);
    goals.push_back(agent_set_goal);

    agent_goal[1].x = 1.2;
    agent_goal[1].y = 0.9;
    agent_goal[1].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[1].x, agent_goal[1].y);
    goals.push_back(agent_set_goal);

    agent_goal[2].x = -0.8;
    agent_goal[2].y = 0.9;
    agent_goal[2].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[2].x, agent_goal[2].y);
    goals.push_back(agent_set_goal);

    agent_goal[3].x = 1.2;
    agent_goal[3].y = -0.75;
    agent_goal[3].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[3].x, agent_goal[3].y);
    goals.push_back(agent_set_goal);

    agent_goal[4].x = -0.8;
    agent_goal[4].y = -0.75;
    agent_goal[4].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[4].x, agent_goal[4].y);
    goals.push_back(agent_set_goal);

    agent_goal[5].x = 0.0;
    agent_goal[5].y = 1.5;
    agent_goal[5].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[5].x, agent_goal[5].y);
    goals.push_back(agent_set_goal);

    agent_goal[6].x = 1.5;
    agent_goal[6].y = 0.0;
    agent_goal[6].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[6].x, agent_goal[6].y);
    goals.push_back(agent_set_goal);

    agent_goal[7].x = 0.0;
    agent_goal[7].y = -1.5;
    agent_goal[7].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[7].x, agent_goal[7].y);
    goals.push_back(agent_set_goal);

    for (int i = 0; i < agent_num; i++)  
    {   
        if (i < goals.size()) {
            sim->setAgentGoal(i, goals[i]);
            cout << BLUE << node_name << ":  Set agents_" << i+1 << " goal at [" << goals[i].x() << "," << goals[i].y() << "]"<< TAIL << endl;
        }
        goal_reached_printed[i] = false;
    }

    cout << BLUE << node_name << ":  setup_scenario_3 goals success!" << TAIL << endl;
}

void ORCA::setup_scenario_4()
{
    // 容器清零
    goals.clear();
    RVO::Vector2 agent_set_goal;

    // 设置预设目标点 agent_1 -> agent_2
    agent_goal[0].x = 0.1;
    agent_goal[0].y = 1.35;
    agent_goal[0].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[0].x, agent_goal[0].y);
    goals.push_back(agent_set_goal);

    agent_goal[1].x = 1.4;
    agent_goal[1].y = 0.5;
    agent_goal[1].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[1].x, agent_goal[1].y);
    goals.push_back(agent_set_goal);

    agent_goal[2].x = -0.8;
    agent_goal[2].y = 0.0;
    agent_goal[2].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[2].x, agent_goal[2].y);
    goals.push_back(agent_set_goal);

    agent_goal[3].x = 1.4;
    agent_goal[3].y = -0.5;
    agent_goal[3].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[3].x, agent_goal[3].y);
    goals.push_back(agent_set_goal);

    agent_goal[4].x = 0.1;
    agent_goal[4].y = -1.35;
    agent_goal[4].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[4].x, agent_goal[4].y);
    goals.push_back(agent_set_goal);

    agent_goal[5].x = -1.5;
    agent_goal[5].y = -0.75;
    agent_goal[5].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[5].x, agent_goal[5].y);
    goals.push_back(agent_set_goal);

    agent_goal[6].x = -1.5;
    agent_goal[6].y = -1.5;
    agent_goal[6].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[6].x, agent_goal[6].y);
    goals.push_back(agent_set_goal);

    agent_goal[7].x = -1.5;
    agent_goal[7].y = 1.5;
    agent_goal[7].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[7].x, agent_goal[7].y);
    goals.push_back(agent_set_goal);

    for (int i = 0; i < agent_num; i++)  
    {   
        if (i < goals.size()) {
            sim->setAgentGoal(i, goals[i]);
            cout << BLUE << node_name << ":  Set agents_" << i+1 << " goal at [" << goals[i].x() << "," << goals[i].y() << "]"<< TAIL << endl;
        }
        goal_reached_printed[i] = false;
    }

    cout << BLUE << node_name << ":  setup_scenario_4 goals success!" << TAIL << endl;
}

void ORCA::setup_scenario_5()
{
    // 容器清零
    goals.clear();
    RVO::Vector2 agent_set_goal;

// O——1
    // 设置预设目标点 agent_1 -> agent_2
    agent_goal[0].x = 0.1;
    agent_goal[0].y = 0.6;
    agent_goal[0].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[0].x, agent_goal[0].y);
    goals.push_back(agent_set_goal);

// N——2
    agent_goal[1].x = 1.1;
    agent_goal[1].y = 1.1;
    agent_goal[1].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[1].x, agent_goal[1].y);
    goals.push_back(agent_set_goal);
// K——3
    agent_goal[2].x = -0.7;
    agent_goal[2].y = 0.0;
    agent_goal[2].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[2].x, agent_goal[2].y);
    goals.push_back(agent_set_goal);
// V——4
    agent_goal[3].x = 1.1;
    agent_goal[3].y = -1.1;
    agent_goal[3].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[3].x, agent_goal[3].y);
    goals.push_back(agent_set_goal);
// O——5
    agent_goal[4].x = 0.1;
    agent_goal[4].y = -0.6;
    agent_goal[4].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[4].x, agent_goal[4].y);
    goals.push_back(agent_set_goal);

    agent_goal[5].x = -1.5;
    agent_goal[5].y = -0.75;
    agent_goal[5].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[5].x, agent_goal[5].y);
    goals.push_back(agent_set_goal);

    agent_goal[6].x = -1.5;
    agent_goal[6].y = 0.75;
    agent_goal[6].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[6].x, agent_goal[6].y);
    goals.push_back(agent_set_goal);

    agent_goal[7].x = -1.5;
    agent_goal[7].y = 1.5;
    agent_goal[7].z = agent_height;
    agent_set_goal = RVO::Vector2(agent_goal[7].x, agent_goal[7].y);
    goals.push_back(agent_set_goal);
    for (int i = 0; i < agent_num; i++)  
    {   
        if (i < goals.size()) {
            sim->setAgentGoal(i, goals[i]);
            cout << BLUE << node_name << ":  Set agents_" << i+1 << " goal at [" << goals[i].x() << "," << goals[i].y() << "]"<< TAIL << endl;
        }
        goal_reached_printed[i] = false;
    }

    cout << BLUE << node_name << ":  setup_scenario_5 goals success!" << TAIL << endl;
}
