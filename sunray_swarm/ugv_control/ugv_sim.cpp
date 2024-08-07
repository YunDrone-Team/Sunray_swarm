#include "ugv_sim.h"

void UGV_SIM::init(ros::NodeHandle& nh)
{
    node_name = ros::this_node::getName();

    // 【参数】智能体类型
    nh.param<int>("agent_type", agent_type, 1);
    nh.param<int>("agent_id", agent_id, 0);
    nh.param<double>("init_pos_x", agent_pos.pose.position.x, 0.0);
    nh.param<double>("init_pos_y", agent_pos.pose.position.y, 0.0);
    nh.param<double>("init_yaw", agent_yaw, 0.0);

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

    string agent_name = agent_prefix + std::to_string(agent_id);
    // 【订阅】地面站指令 地面站 -> 本节点 
    ugv_cmd_sub = nh.subscribe<sunray_msgs::ugv_cmd>("/sunray_swarm" + agent_name + "/ugv_cmd", 1, &UGV_SIM::ugv_cmd_cb, this);  
    // 【订阅】无人机cmd数据
    ugv_cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/sunray_swarm" + agent_name + "/cmd_vel", 1, &UGV_SIM::ugv_cmd_vel_cb, this);
    // 【订阅】ORCA算法状态
    orca_state_sub = nh.subscribe<sunray_msgs::orca_state>("/sunray_swarm" + agent_name + "/orca_state", 1, &UGV_SIM::agent_orca_state_cb, this);
    // 【发布】位置&速度数据（仿真中模拟）
    mocap_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/vrpn_client_node"+ agent_name + "/pose", 1);
    // 【发布】无人机电池（在仿真中，电池电量设置为101，代表仿真模式）
    battery_pub = nh.advertise<std_msgs::Float32>("/sunray_swarm" + agent_name + "/battery", 1);

    agent_height = 0.1;
    battery.data = 101;

    // 初始化无人机初始位置
    agent_pos.header.stamp = ros::Time::now();
    agent_pos.header.frame_id = "world";
    agent_pos.pose.position.z = agent_height;
    mocap_pos_pub.publish(agent_pos);

    cout << GREEN << node_name << " init! " << TAIL << endl;
    control_state = CONTROL_STATE::INIT;
}

bool UGV_SIM::mainloop()
{
    float dt = 0.01;
    switch (control_state)
    {
    case CONTROL_STATE::INIT:
        agent_pos.header.stamp = ros::Time::now();
        agent_pos.header.frame_id = "world";
        agent_pos.pose.position.x = agent_pos.pose.position.x;
        agent_pos.pose.position.y = agent_pos.pose.position.y;
        agent_pos.pose.position.z = agent_height;
        agent_pos.pose.orientation = ros_quaternion_from_rpy(0.0, 0.0, agent_yaw);
        mocap_pos_pub.publish(agent_pos);
        break;

    case CONTROL_STATE::HOLD:
        agent_pos.header.stamp = ros::Time::now();
        agent_pos.header.frame_id = "world";
        agent_pos.pose.position.x = agent_pos.pose.position.x;
        agent_pos.pose.position.y = agent_pos.pose.position.y;
        agent_pos.pose.position.z = agent_height;
        agent_pos.pose.orientation = ros_quaternion_from_rpy(0.0, 0.0, agent_yaw);
        mocap_pos_pub.publish(agent_pos);
        break;

    case CONTROL_STATE::POS_CONTROL:
        agent_pos.header.stamp = ros::Time::now();
        agent_pos.header.frame_id = "world";
        agent_pos.pose.position.x = agent_pos.pose.position.x + cmd_vel.linear.x * dt;
        agent_pos.pose.position.y = agent_pos.pose.position.y + cmd_vel.linear.y * dt;
        agent_pos.pose.position.z = agent_height;
        agent_pos.pose.orientation = ros_quaternion_from_rpy(0.0, 0.0, agent_yaw);
        mocap_pos_pub.publish(agent_pos);
        break;

    case CONTROL_STATE::VEL_CONTROL:
        agent_pos.header.stamp = ros::Time::now();
        agent_pos.header.frame_id = "world";
        agent_pos.pose.position.x = agent_pos.pose.position.x + cmd_vel.linear.x * dt;
        agent_pos.pose.position.y = agent_pos.pose.position.y + cmd_vel.linear.y * dt;
        agent_pos.pose.position.z = agent_height;
        agent_pos.pose.orientation = ros_quaternion_from_rpy(0.0, 0.0, agent_yaw);
        mocap_pos_pub.publish(agent_pos);
        break;

    case CONTROL_STATE::ORCA_MODE:
        if(agent_orca_state.arrived_goal)
        {
            agent_pos.header.stamp = ros::Time::now();
            agent_pos.header.frame_id = "world";
            agent_pos.pose.position.x = agent_orca_state.goal[0];
            agent_pos.pose.position.y = agent_orca_state.goal[1];
            agent_pos.pose.position.z = agent_height;
        }else
        {
            // 根据cmd_vel更新位置（直接积分）
            agent_pos.header.stamp = ros::Time::now();
            agent_pos.header.frame_id = "world";
            agent_pos.pose.position.x = agent_pos.pose.position.x + cmd_vel.linear.x * dt;
            agent_pos.pose.position.y = agent_pos.pose.position.y + cmd_vel.linear.y * dt;
            agent_pos.pose.position.z = agent_height;
        }
        agent_pos.pose.orientation = ros_quaternion_from_rpy(0.0, 0.0, agent_yaw);
        mocap_pos_pub.publish(agent_pos);
        break;

    case CONTROL_STATE::TRACK_MODE:
        agent_pos.header.stamp = ros::Time::now();
        agent_pos.header.frame_id = "world";
        agent_pos.pose.position.x = agent_pos.pose.position.x + cmd_vel.linear.x * dt;
        agent_pos.pose.position.y = agent_pos.pose.position.y + cmd_vel.linear.y * dt;
        agent_pos.pose.position.z = agent_height;
        agent_pos.pose.orientation = ros_quaternion_from_rpy(0.0, 0.0, agent_yaw);
        mocap_pos_pub.publish(agent_pos);
        break; 

    case CONTROL_STATE::RETURN_HOME:
        if(agent_orca_state.arrived_goal)
        {
            agent_pos.header.stamp = ros::Time::now();
            agent_pos.header.frame_id = "world";
            agent_pos.pose.position.x = agent_orca_state.goal[0];
            agent_pos.pose.position.y = agent_orca_state.goal[1];
            agent_pos.pose.position.z = agent_height;
        }else
        {
            // 根据cmd_vel更新位置（直接积分）
            agent_pos.header.stamp = ros::Time::now();
            agent_pos.header.frame_id = "world";
            agent_pos.pose.position.x = agent_pos.pose.position.x + cmd_vel.linear.x * dt;
            agent_pos.pose.position.y = agent_pos.pose.position.y + cmd_vel.linear.y * dt;
            agent_pos.pose.position.z = agent_height;
        }
        agent_pos.pose.orientation = ros_quaternion_from_rpy(0.0, 0.0, agent_yaw);
        mocap_pos_pub.publish(agent_pos);
        break; 

    default:
        break;
    }

    battery_pub.publish(battery);

    if(!agent_orca_state.arrived_goal)
    {
        return false;
    }

    return true;
}

void UGV_SIM::ugv_cmd_cb(const sunray_msgs::ugv_cmd::ConstPtr& msg)
{
    if(msg->agent_id != agent_id && msg->agent_id != 99)
    {
        return;
    }

    current_ugv_cmd = *msg; 
    switch(msg->control_state) 
    {
        case CONTROL_STATE::INIT:
            control_state = CONTROL_STATE::INIT;
            break;

        case CONTROL_STATE::HOLD:
            control_state = CONTROL_STATE::HOLD;
            break;

        case CONTROL_STATE::POS_CONTROL:
            control_state = CONTROL_STATE::POS_CONTROL;
            break;

        case CONTROL_STATE::VEL_CONTROL:
            control_state = CONTROL_STATE::VEL_CONTROL;
            break;

        case CONTROL_STATE::ORCA_MODE:
            control_state = CONTROL_STATE::ORCA_MODE;
            agent_orca_state.arrived_goal = false; 
            break;

        case CONTROL_STATE::TRACK_MODE:
            control_state = CONTROL_STATE::TRACK_MODE;
            agent_orca_state.arrived_goal = false;          
            break;

        case CONTROL_STATE::RETURN_HOME:
            control_state = CONTROL_STATE::RETURN_HOME;
            agent_orca_state.arrived_goal = false;      
            break;
    }
}

void UGV_SIM::agent_orca_state_cb(const sunray_msgs::orca_state::ConstPtr& msg)
{
    agent_orca_state = *msg;
}

void UGV_SIM::ugv_cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmd_vel = *msg;
}

// 从(roll,pitch,yaw)创建四元数  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
geometry_msgs::Quaternion UGV_SIM::ros_quaternion_from_rpy(double roll, double pitch, double yaw)
{
    Eigen::Quaterniond q = Eigen::Quaterniond(
                        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitX()));

    geometry_msgs::Quaternion ros_q;

    ros_q.x = q.x();
    ros_q.y = q.y();
    ros_q.z = q.z();
    ros_q.w = q.w();

    return ros_q;    
}



