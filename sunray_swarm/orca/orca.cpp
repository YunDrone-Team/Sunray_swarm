#include "orca.h"

void ORCA::init(ros::NodeHandle& nh)
{
    // 【参数】智能体类型
    //  由于无人机和无人车不在一个平面，无人机和无人车需要分别启动两个不同的ORCA算法节点
    nh.param<int>("agent_type", agent_type, 1);
    // 【参数】ORCA算法智能体数量
    nh.param<int>("agent_num", agent_num, 8);
    // 【参数】智能体的固定高度
    nh.param<float>("agent_height", agent_height, 1.0);
    // 【参数】智能体之间的假想感知距离
    nh.param<float>("orca_params/neighborDist", orca_params.neighborDist, 2.0);
    orca_params.maxNeighbors = agent_num;
    // 【参数】数字越大，智能体响应相邻智能体的碰撞越快，但速度可选的自由度越小
    nh.param<float>("orca_params/timeHorizon", orca_params.timeHorizon, 2.0);
    // 【参数】数字越大，智能体响应障碍物的碰撞越快，但速度可选的自由度越小
    nh.param<float>("orca_params/timeHorizonObst", orca_params.timeHorizonObst, 1.0);
    // 【参数】智能体体积半径
    nh.param<float>("orca_params/radius", orca_params.radius, 0.35);
    // 【参数】智能体最大移动速度
    nh.param<float>("orca_params/maxSpeed", orca_params.maxSpeed, 0.3);
    // 【参数】时间步长
    nh.param<float>("orca_params/time_step", orca_params.time_step, 0.1);

    string agent_prefix;
    if(agent_type == sunray_msgs::agent_state::RMTT)
    {
        agent_prefix = "/rmtt";
    }
    else if(agent_type == sunray_msgs::agent_state::UGV)
    {
        agent_prefix = "/ugv";
    }

    // 【订阅】ORCA算法指令 地面站/其他节点 -> 本节点
    orca_cmd_sub = nh.subscribe<sunray_msgs::orca_cmd>("/sunray_swarm" + agent_prefix + "/orca_cmd", 10, &ORCA::orca_cmd_cb, this);

    string agent_name;
    for(int i = 0; i < agent_num; i++) 
    {
        agent_name = agent_prefix + "_" + std::to_string(i+1);
        // 【订阅】智能体状态数据 智能体控制节点 -> 本节点
        agent_state_sub[i] = nh.subscribe<sunray_msgs::agent_state>("/sunray_swarm" + agent_name + "/agent_state", 20, boost::bind(&ORCA::agent_state_cb,this ,_1,i));
        // 【订阅】智能体目标点（xy+yaw） 地面站/其他节点 -> 本节点
        agent_goal_sub[i] = nh.subscribe<geometry_msgs::Point>("/sunray_swarm" + agent_name + "/goal_point", 10, boost::bind(&ORCA::agent_goal_cb,this ,_1,i));
        // 【发布】智能体控制指令 本节点 -> 智能体控制节点
        agent_cmd_pub[i] = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 10); 
        // 【发布】无人机orca状态 本节点 -> 地面站/其他节点
		agent_orca_state_pub[i] = nh.advertise<sunray_msgs::orca_state>("/sunray_swarm" + agent_name + "/agent_orca_state", 1);
        // 【发布】目标点marker 本节点 -> RVIZ （仿真）
        goal_point_pub[i] = nh.advertise<visualization_msgs::Marker>("/sunray_swarm" + agent_name + "/goal_point_rviz", 1);   
    }

    // 【发布】文字提示消息（回传至地面站显示）
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);

    // 【定时器】 定时打印状态
    timer_debug = nh.createTimer(ros::Duration(10.0), &ORCA::timercb_debug, this);

    for(int i = 0; i < agent_num; i++) 
    {
        arrived_goal[i] = false;
    }
    arrived_all_goal = false;     

    // 初始化 goal_reached_printed
    goal_reached_printed.resize(agent_num, false);  

    // 打印本节点参数，用于检查
    printf_param();

    // ORCA算法初始化 - 添加智能体
    setup_agents();
    // ORCA算法初始化 - 添加障碍物
    // setup_obstacles();

    // 设置仿真时RVIZ中不同智能体目标点的颜色，与真机无关
    setup_color();

    node_name = ros::this_node::getName();
    text_info.data = node_name + ": ORCA init!";
    text_info_pub.publish(text_info);
    cout << BLUE << text_info.data << TAIL << endl;
}

bool ORCA::orca_run()
{
    // 没有收到ORCA算法启动的指令，不执行主循环
    if(!start_flag)
    {
        return false;
    }

    // 默认全部已经达到目标点，在后续执行中会重新判定本FLAG
    arrived_all_goal = true;

    // 判断每个智能体是否达到目标点附近
	for(int i = 0; i < agent_num; ++i) 
	{	
		if(!arrived_goal[i])
		{
			arrived_goal[i] = reachedGoal(i);
		}
    }

	// 更新ORCA算法中每个智能体的位置和速度
	for(int i = 0; i < agent_num; ++i) 
	{	
		RVO::Vector2 pos = RVO::Vector2(agent_state[i].pos[0],agent_state[i].pos[1]);
		sim->setAgentPosition(i, pos);
	}

	// 计算每一个智能体的期望速度
    // ORCA算法会根据每个智能体当前的位置和速度，以及设定好的参数，计算一个不会发生碰撞的控制速度
	sim->computeVel();

    // 遍历每一个智能体
	for(int i = 0; i < agent_num; ++i) 
	{	
        // 如果智能体到达目标点附近，则使用位置控制的方式直接移动到目标点
		if(arrived_goal[i])
		{
            agent_cmd[i].agent_id = i+1;
            agent_cmd[i].control_state = sunray_msgs::agent_cmd::POS_CONTROL;
            agent_cmd[i].cmd_source = "ORCA";
            agent_cmd[i].desired_pos.x = sim->getAgentGoal(i).x();
            agent_cmd[i].desired_pos.y = sim->getAgentGoal(i).y();
            agent_cmd[i].desired_pos.z = agent_height; 
            agent_cmd[i].desired_yaw = goal_pose[i].yaw;
            agent_cmd_pub[i].publish(agent_cmd[i]);
            sleep(0.01);
            // 当智能体抵达目标点时，会打印一次
            if (!goal_reached_printed[i])
            {
                text_info.data = "agent_" + std::to_string(i+1) + " Arrived.";
                cout << BLUE << text_info.data << TAIL << endl;
                text_info_pub.publish(text_info);
                goal_reached_printed[i] = true;
            }
		}
        // 如果智能体没有达到目标点附近，则将ORCA算法计算得到速度以智能体控制指令发送出去（使用VEL_CONTROL_ENU模式）
		else
		{
            // 从ORCA算法中读取期望速度 注：这个速度是ENU坐标系的
            RVO::Vector2 vel = sim->getAgentVelCMD(i);   
            agent_cmd[i].agent_id = i+1;
            agent_cmd[i].control_state = sunray_msgs::agent_cmd::VEL_CONTROL_ENU;
            agent_cmd[i].cmd_source = "ORCA";
            agent_cmd[i].desired_vel.linear.x = vel.x();
            agent_cmd[i].desired_vel.linear.y = vel.y();
            agent_cmd[i].desired_vel.linear.z = 0.0;
            agent_cmd[i].desired_yaw = goal_pose[i].yaw; 
            agent_cmd_pub[i].publish(agent_cmd[i]);
            sleep(0.01);
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
        agent_orca_state[i].yaw = goal_pose[i].yaw;
        RVO::Vector2 vel = sim->getAgentVelCMD(i);
        agent_orca_state[i].vel_orca[0] = agent_cmd[i].desired_vel.linear.x;
        agent_orca_state[i].vel_orca[1] = agent_cmd[i].desired_vel.linear.y;
        agent_orca_state_pub[i].publish(agent_orca_state[i]);

        // 发布目标点mesh - RVIZ（仿真用）
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
        goal_marker.color = goal_point_color[i];
        goal_marker.mesh_use_embedded_materials = false;
        goal_point_pub[i].publish(goal_marker);
    }

    return arrived_all_goal;
}

// ORCA算法初始化 - 设置智能体
void ORCA::setup_agents()
{
    // 设置算法参数
    // sim->setAgentDefaults(15.0f, 10, 5.0f, 5.0f, 2.0f, 2.0f);
    sim->setAgentDefaults(orca_params.neighborDist, orca_params.maxNeighbors, orca_params.timeHorizon, 
                    orca_params.timeHorizonObst, orca_params.radius, orca_params.maxSpeed);
	// 设置时间间隔（这个似乎没有用？）
	sim->setTimeStep(orca_params.time_step);
	// 根据agent_num添加智能体
	for (int i = 0; i < agent_num; i++) 
	{	
		RVO::Vector2 pos = RVO::Vector2(agent_state[i].pos[0],agent_state[i].pos[1]);
		sim->addAgent(pos);	
        cout << BLUE << node_name << " ORCA add agents_" << i+1 << " at [" << agent_state[i].pos[0] << "," << agent_state[i].pos[1] << "]"<< TAIL << endl;
	}

    cout << BLUE << node_name << ": Set agents success!" << TAIL << endl;
}

// ORCA算法初始化 - 设置障碍物
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

// 定时器回调函数 - 打印状态信息
void ORCA::timercb_debug(const ros::TimerEvent &e)
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
        cout << GREEN << "GOAL [X Y YAW]  : " << agent_orca_state[i].goal[0] << " [ m ] " << agent_orca_state[i].goal[1] << " [ m ] " << agent_orca_state[i].yaw *180/M_PI << " [ deg ] " << TAIL << endl;
        cout << GREEN << "CMD  [X Y]  : " << agent_orca_state[i].vel_orca[0] << " [m/s] " << agent_orca_state[i].vel_orca[1] << " [m/s] " << TAIL << endl;
    }
}

// 判断智能体是否达到目标点
bool ORCA::reachedGoal(int i)
{
    bool xy_arrived{false}, z_arrived{false}, yaw_arrived{false};
    RVO::Vector2 rvo_goal = sim->getAgentGoal(i);
    // 主要判断xy平面是否到达
    float xy_distance = (agent_state[i].pos[0] - rvo_goal.x())*(agent_state[i].pos[0] - rvo_goal.x()) + (agent_state[i].pos[1] - rvo_goal.y())*(agent_state[i].pos[1] - rvo_goal.y()); 
	
    // 0.15*0.15是距离阈值，可以根据实际情况调整该阈值
    if (xy_distance < 0.15f * 0.15f) 
	{
        xy_arrived = true;
	}

    // 对于无人机来说，由于高度控制有一些噪声，因此高度上设置一个判断阈值
    if(abs(agent_state[i].pos[2] - agent_height) < 0.08f)
    {
        z_arrived = true;
    }

    // 由于偏航角控制有一些噪声，因此设置达到阈值
    if(abs(agent_state[i].att[2] - agent_orca_state[i].yaw) /M_PI *180< 3.0f)
    {
        yaw_arrived = true;
    }

    // xy平面、z高度、yaw偏航角均达到，才认为智能体抵达了目标点
    if(xy_arrived && z_arrived && yaw_arrived)
    {
	    return true;
    }

    return false;
}

// 回调函数：智能体状态回调函数，根据ID存放到agent_state数组里
void ORCA::agent_state_cb(const sunray_msgs::agent_state::ConstPtr& msg, int i)
{
    agent_state[i] = *msg;
}

// 回调函数：智能体目标点回调函数，根据ID存放到agent_goal数组里，并同时在ORCA算法中设置目标点
void ORCA::agent_goal_cb(const geometry_msgs::Point::ConstPtr& msg, int i)
{
    // 当收到新的目标点时，将状态位都设置为false
    arrived_goal[i] = false;
    arrived_all_goal = false;  
    goal_reached_printed[i] = false; 

    // 读取期望目标位置(x,y)和期望偏航角（使用z来代为传递）
    goal_pose[i].x = msg->x;
    goal_pose[i].y = msg->y;
    goal_pose[i].yaw = msg->z;

    // 在ORCA算法中设置目标点
    sim->setAgentGoal(i, RVO::Vector2(goal_pose[i].x, goal_pose[i].y));
    // cout << BLUE << node_name << ": Set agents_" << i+1 << " goal at [" << goal_pose[i].x << "," << goal_pose[i].y << "]"<< TAIL << endl;
    // text_info.data = "[ORCA] Set agents_"+std::to_string(i+1)+" goal";
    // text_info_pub.publish(text_info);
}

// 回调函数：ORCA算法指令回调函数，根据msg->orca_cmd的值来判断处理
void ORCA::orca_cmd_cb(const sunray_msgs::orca_cmd::ConstPtr& msg)
{
    // 当orca_cmd为SET_HOME时，将每个智能体的当前所在点设置为home点，并同时启动ORCA算法
    if(msg->orca_cmd == sunray_msgs::orca_cmd::SET_HOME)
    {
        start_flag = true;
        // 记录home点
        for(int i = 0; i < agent_num; i++) 
        {
            home_pose[i].x = agent_state[i].pos[0];
            home_pose[i].y = agent_state[i].pos[1];
            home_pose[i].yaw = agent_state[i].att[2];
            cout << BLUE << node_name << ":  Set agents_" << i+1 << " home at [" << home_pose[i].x << "," << home_pose[i].y << "] with "<< home_pose[i].yaw * 180 / M_PI << "deg" << TAIL << endl;
        }
        // ORCA算法初始化 - 添加当前为目标点（意味着当ORCA没有收到新的目标点时，智能体已经抵达对应目标点）
        setup_init_goals();
        text_info.data = "Get orca_cmd: SET_HOME, ORCA start!"; 
        cout << BLUE << node_name << text_info.data << TAIL << endl;
    }

    // 当orca_cmd为RETURN_HOME时，将每个智能体的home点设置为目标点，智能体会直接返回初始位置
    if(msg->orca_cmd == sunray_msgs::orca_cmd::RETURN_HOME)
    {
        // 相当于收到了新的目标点，将状态位都设置为false
        for(int i = 0; i < agent_num; i++) 
        {
            arrived_goal[i] = false;
            goal_reached_printed[i] = false;
        }
        arrived_all_goal = false;

        // 将home点设置为目标点
        goals.clear();
        for (int i = 0; i < agent_num; i++)  
        {  
            goals.push_back(RVO::Vector2(home_pose[i].x, home_pose[i].y));
            goal_pose[i].x =  home_pose[i].x;
            goal_pose[i].y =  home_pose[i].y;
            goal_pose[i].yaw = home_pose[i].yaw;
        }

        for (int i = 0; i < agent_num; i++)  
        {   
            if (i < goals.size()) 
            {
                sim->setAgentGoal(i, goals[i]);
                cout << BLUE << node_name << ":  Set agents_" << i+1 << " goal at [" << goals[i].x() << "," << goals[i].y() << "]"<< TAIL << endl;
            }
            
        }
        text_info.data = "Get orca_cmd: RETURN_HOME"; 
        cout << BLUE << node_name << text_info.data << TAIL << endl;
    }

    // 当orca_cmd为SETUP_OBS时，在ORCA算法中设置障碍物
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

    // 当orca_cmd为ORCA_STOP时，不更改目标点，但是停止ORCA算法运行
    if(msg->orca_cmd == sunray_msgs::orca_cmd::ORCA_STOP)
    {
        start_flag = false;
        // 发送HOLD指令
        for(int i = 0; i < agent_num; ++i) 
        {	
            agent_cmd[i].agent_id = i+1;
            agent_cmd[i].control_state = sunray_msgs::agent_cmd::HOLD;
            agent_cmd[i].cmd_source = "ORCA";
            agent_cmd_pub[i].publish(agent_cmd[i]);
        }
        text_info.data = "Get orca_cmd: ORCA_STOP!"; 
        cout << BLUE << node_name << text_info.data << TAIL << endl;
    }

    // 当orca_cmd为ORCA_RESTART时，不更改此前目标点，继续运行ORCA算法
    if(msg->orca_cmd == sunray_msgs::orca_cmd::ORCA_RESTART)
    {
        start_flag = true;
        text_info.data = "Get orca_cmd: ORCA_RESTART!"; 
        cout << BLUE << node_name << text_info.data << TAIL << endl;
    }

    // ORCA_SCENARIO_1 - ORCA_SCENARIO_5 : 直接使用预设好的5组目标点
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
    }else if(agent_type == sunray_msgs::agent_state::UGV)
    {
        cout << GREEN << "agent_type : UGV" << TAIL << endl;
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
        goal_pose[i].x = home_pose[i].x;
        goal_pose[i].y = home_pose[i].y;
        goal_pose[i].yaw = home_pose[i].yaw;
        agent_set_goal = RVO::Vector2(goal_pose[i].x, goal_pose[i].y);
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
    goal_point_color[0].a = 1.0;
    goal_point_color[0].r = 1.0;
    goal_point_color[0].g = 0.0;
    goal_point_color[0].b = 0.0;
    goal_point_color[1].a = 1.0;
    goal_point_color[1].r = 0.0;
    goal_point_color[1].g = 1.0;
    goal_point_color[1].b = 0.0;
    goal_point_color[2].a = 1.0;
    goal_point_color[2].r = 0.0;
    goal_point_color[2].g = 0.0;
    goal_point_color[2].b = 1.0;
    goal_point_color[3].a = 1.0;
    goal_point_color[3].r = 1.0;
    goal_point_color[3].g = 1.0;
    goal_point_color[3].b = 0.0;
    goal_point_color[4].a = 1.0;
    goal_point_color[4].r = 1.0;
    goal_point_color[4].g = 0.0;
    goal_point_color[4].b = 1.0;
    goal_point_color[5].a = 1.0;
    goal_point_color[5].r = 0.0;
    goal_point_color[5].g = 1.0;
    goal_point_color[5].b = 1.0;
    goal_point_color[6].a = 1.0;
    goal_point_color[6].r = 0.5;
    goal_point_color[6].g = 0.5;
    goal_point_color[6].b = 0.5;
    goal_point_color[7].a = 1.0;
    goal_point_color[7].r = 0.3;
    goal_point_color[7].g = 0.7;
    goal_point_color[7].b = 0.2;
}

void ORCA::setup_scenario_1()
{
    // 容器清零
    goals.clear();
    RVO::Vector2 agent_set_goal;

    // 设置预设目标点 agent_1 -> agent_2
    goal_pose[0].x = -0.85;
    goal_pose[0].y = 1.35;
    goal_pose[0].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[0].x, goal_pose[0].y);
    goals.push_back(agent_set_goal);

    goal_pose[1].x = 1.05;
    goal_pose[1].y = 1.35;
    goal_pose[1].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[1].x, goal_pose[1].y);
    goals.push_back(agent_set_goal);

    goal_pose[2].x = 0.1;
    goal_pose[2].y = -0.15;
    goal_pose[2].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[2].x, goal_pose[2].y);
    goals.push_back(agent_set_goal);

    goal_pose[3].x = 1.05;
    goal_pose[3].y = -1.25;
    goal_pose[3].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[3].x, goal_pose[3].y);
    goals.push_back(agent_set_goal);

    goal_pose[4].x = -1.0;
    goal_pose[4].y = -1.3;
    goal_pose[4].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[4].x, goal_pose[4].y);
    goals.push_back(agent_set_goal);

    goal_pose[5].x = -1.5;
    goal_pose[5].y = 0.75;
    goal_pose[5].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[5].x, goal_pose[5].y);
    goals.push_back(agent_set_goal);

    goal_pose[6].x = -1.5;
    goal_pose[6].y = -0.75;
    goal_pose[6].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[6].x, goal_pose[6].y);
    goals.push_back(agent_set_goal);

    goal_pose[7].x = -1.5;
    goal_pose[7].y = -1.5;
    goal_pose[7].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[7].x, goal_pose[7].y);
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
    goal_pose[0].x = 0.1;
    goal_pose[0].y = 1.35;
    goal_pose[0].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[0].x, goal_pose[0].y);
    goals.push_back(agent_set_goal);

    goal_pose[1].x = 1.4;
    goal_pose[1].y = 0.5;
    goal_pose[1].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[1].x, goal_pose[1].y);
    goals.push_back(agent_set_goal);

    goal_pose[2].x = -0.8;
    goal_pose[2].y = -0.0;
    goal_pose[2].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[2].x, goal_pose[2].y);
    goals.push_back(agent_set_goal);

    goal_pose[3].x = 1.4;
    goal_pose[3].y = -0.5;
    goal_pose[3].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[3].x, goal_pose[3].y);
    goals.push_back(agent_set_goal);

    goal_pose[4].x = 0.1;
    goal_pose[4].y = -1.35;
    goal_pose[4].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[4].x, goal_pose[4].y);
    goals.push_back(agent_set_goal);

    goal_pose[5].x = -1.5;
    goal_pose[5].y = 0.75;
    goal_pose[5].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[5].x, goal_pose[5].y);
    goals.push_back(agent_set_goal);

    goal_pose[6].x = -1.5;
    goal_pose[6].y = -0.75;
    goal_pose[6].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[6].x, goal_pose[6].y);
    goals.push_back(agent_set_goal);

    goal_pose[7].x = -1.5;
    goal_pose[7].y = 1.5;
    goal_pose[7].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[7].x, goal_pose[7].y);
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
    goal_pose[0].x = 0.1;
    goal_pose[0].y = 0.9;
    goal_pose[0].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[0].x, goal_pose[0].y);
    goals.push_back(agent_set_goal);

    goal_pose[1].x = 1.2;
    goal_pose[1].y = 0.9;
    goal_pose[1].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[1].x, goal_pose[1].y);
    goals.push_back(agent_set_goal);

    goal_pose[2].x = -0.8;
    goal_pose[2].y = 0.9;
    goal_pose[2].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[2].x, goal_pose[2].y);
    goals.push_back(agent_set_goal);

    goal_pose[3].x = 1.2;
    goal_pose[3].y = -0.75;
    goal_pose[3].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[3].x, goal_pose[3].y);
    goals.push_back(agent_set_goal);

    goal_pose[4].x = -0.8;
    goal_pose[4].y = -0.75;
    goal_pose[4].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[4].x, goal_pose[4].y);
    goals.push_back(agent_set_goal);

    goal_pose[5].x = 0.0;
    goal_pose[5].y = 1.5;
    goal_pose[5].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[5].x, goal_pose[5].y);
    goals.push_back(agent_set_goal);

    goal_pose[6].x = 1.5;
    goal_pose[6].y = 0.0;
    goal_pose[6].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[6].x, goal_pose[6].y);
    goals.push_back(agent_set_goal);

    goal_pose[7].x = 0.0;
    goal_pose[7].y = -1.5;
    goal_pose[7].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[7].x, goal_pose[7].y);
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
    goal_pose[0].x = 0.1;
    goal_pose[0].y = 1.35;
    goal_pose[0].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[0].x, goal_pose[0].y);
    goals.push_back(agent_set_goal);

    goal_pose[1].x = 1.4;
    goal_pose[1].y = 0.5;
    goal_pose[1].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[1].x, goal_pose[1].y);
    goals.push_back(agent_set_goal);

    goal_pose[2].x = -0.8;
    goal_pose[2].y = 0.0;
    goal_pose[2].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[2].x, goal_pose[2].y);
    goals.push_back(agent_set_goal);

    goal_pose[3].x = 1.4;
    goal_pose[3].y = -0.5;
    goal_pose[3].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[3].x, goal_pose[3].y);
    goals.push_back(agent_set_goal);

    goal_pose[4].x = 0.1;
    goal_pose[4].y = -1.35;
    goal_pose[4].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[4].x, goal_pose[4].y);
    goals.push_back(agent_set_goal);

    goal_pose[5].x = -1.5;
    goal_pose[5].y = -0.75;
    goal_pose[5].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[5].x, goal_pose[5].y);
    goals.push_back(agent_set_goal);

    goal_pose[6].x = -1.5;
    goal_pose[6].y = -1.5;
    goal_pose[6].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[6].x, goal_pose[6].y);
    goals.push_back(agent_set_goal);

    goal_pose[7].x = -1.5;
    goal_pose[7].y = 1.5;
    goal_pose[7].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[7].x, goal_pose[7].y);
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
    goal_pose[0].x = 0.1;
    goal_pose[0].y = 0.6;
    goal_pose[0].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[0].x, goal_pose[0].y);
    goals.push_back(agent_set_goal);

// N——2
    goal_pose[1].x = 1.1;
    goal_pose[1].y = 1.1;
    goal_pose[1].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[1].x, goal_pose[1].y);
    goals.push_back(agent_set_goal);
// K——3
    goal_pose[2].x = -0.7;
    goal_pose[2].y = 0.0;
    goal_pose[2].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[2].x, goal_pose[2].y);
    goals.push_back(agent_set_goal);
// V——4
    goal_pose[3].x = 1.1;
    goal_pose[3].y = -1.1;
    goal_pose[3].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[3].x, goal_pose[3].y);
    goals.push_back(agent_set_goal);
// O——5
    goal_pose[4].x = 0.1;
    goal_pose[4].y = -0.6;
    goal_pose[4].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[4].x, goal_pose[4].y);
    goals.push_back(agent_set_goal);

    goal_pose[5].x = -1.5;
    goal_pose[5].y = -0.75;
    goal_pose[5].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[5].x, goal_pose[5].y);
    goals.push_back(agent_set_goal);

    goal_pose[6].x = -1.5;
    goal_pose[6].y = 0.75;
    goal_pose[6].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[6].x, goal_pose[6].y);
    goals.push_back(agent_set_goal);

    goal_pose[7].x = -1.5;
    goal_pose[7].y = 1.5;
    goal_pose[7].yaw = 0.0;
    agent_set_goal = RVO::Vector2(goal_pose[7].x, goal_pose[7].y);
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
