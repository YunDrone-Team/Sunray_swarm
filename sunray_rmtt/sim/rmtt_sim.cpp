#include "rmtt_sim.h"
#include <std_msgs/String.h>

void RMTT_SIM::init(ros::NodeHandle& nh)
{
    node_name = ros::this_node::getName();

    nh.param<int>("uav_id", uav_id, 0);
    nh.param<double>("init_pos_x", rmtt_pos.pose.position.x, 0.0);
    nh.param<double>("init_pos_y", rmtt_pos.pose.position.y, 0.0);
    nh.param<double>("init_pos_z", rmtt_pos.pose.position.z, 0.0);

    // 【订阅】地面站指令 地面站 -> 本节点
    station_cmd_sub = nh.subscribe<sunray_msgs::station_cmd>("/sunray_rmtt/station_cmd", 1, &RMTT_SIM::station_cmd_cb, this);  

    string agent_name = "/rmtt_" + std::to_string(uav_id);
    // 【订阅】地面站指令 地面站 -> 本节点 show专用
    station_cmd_sub2 = nh.subscribe<sunray_msgs::station_cmd>("/sunray_rmtt" + agent_name + "/station_cmd", 1, &RMTT_SIM::station_cmd_cb2, this);  
    // 【订阅】无人机cmd数据
    rmtt_cmd_sub = nh.subscribe<geometry_msgs::Twist>("/sunray_rmtt" + agent_name + "/cmd_vel", 1, &RMTT_SIM::rmtt_cmd_cb, this);
    // 【订阅】无人机orca算法状态
    rmtt_orca_state_sub = nh.subscribe<sunray_msgs::rmtt_orca>("/sunray_rmtt" + agent_name + "/rmtt_orca", 1, &RMTT_SIM::rmtt_orca_state_cb, this);
    // 【发布】无人机位置&速度数据（仿真中模拟）
    mocap_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/vrpn_client_node"+ agent_name + "/pose", 1);
    // 【发布】无人机目标点
    rmtt_goal_pub = nh.advertise<geometry_msgs::Point>("/sunray_rmtt" + agent_name + "/goal_point", 1);
    // 【发布】无人机电池（在仿真中，电池电量设置为101，代表仿真模式）
    battery_pub = nh.advertise<std_msgs::Float32>("/sunray_rmtt" + agent_name + "/battery", 1);

    // 初始化无人机初始位置
    rmtt_pos.header.stamp = ros::Time::now();
    rmtt_pos.header.frame_id = "world";
    mocap_pos_pub.publish(rmtt_pos);

    cout << GREEN << node_name << " init! " << TAIL << endl;
    mission_state = MISSION_STATE::INIT;
    battery.data = 101;
}

bool RMTT_SIM::mainloop()
{
    float dt = 0.01;
    switch (mission_state)
    {
    case MISSION_STATE::INIT:
        mission_state = MISSION_STATE::HOLD;
        break;

    case MISSION_STATE::TAKEOFF:
        rmtt_pos.header.stamp = ros::Time::now();
        rmtt_pos.header.frame_id = "world";
        rmtt_pos.pose.position.x = rmtt_pos.pose.position.x;
        rmtt_pos.pose.position.y = rmtt_pos.pose.position.y;
        rmtt_pos.pose.position.z = 1.5;
        mocap_pos_pub.publish(rmtt_pos);
        break;

    case MISSION_STATE::LAND:
        rmtt_pos.header.stamp = ros::Time::now();
        rmtt_pos.header.frame_id = "world";
        rmtt_pos.pose.position.x = rmtt_pos.pose.position.x;
        rmtt_pos.pose.position.y = rmtt_pos.pose.position.y;
        rmtt_pos.pose.position.z = 0.01;
        mocap_pos_pub.publish(rmtt_pos);
        break;

    case MISSION_STATE::HOLD:
        rmtt_pos.header.stamp = ros::Time::now();
        rmtt_pos.header.frame_id = "world";
        rmtt_pos.pose.position.x = rmtt_pos.pose.position.x;
        rmtt_pos.pose.position.y = rmtt_pos.pose.position.y;
        rmtt_pos.pose.position.z = 1.5;
        mocap_pos_pub.publish(rmtt_pos);
        break;

    case MISSION_STATE::ORCA_SETUP:
        mission_state = MISSION_STATE::HOLD;
        break;

    case MISSION_STATE::ORCA_RUN:
        if(rmtt_orca_state.arrived_goal)
        {
            rmtt_pos.header.stamp = ros::Time::now();
            rmtt_pos.header.frame_id = "world";
            rmtt_pos.pose.position.x = rmtt_orca_state.goal[0];
            rmtt_pos.pose.position.y = rmtt_orca_state.goal[1];
            rmtt_pos.pose.position.z = rmtt_pos.pose.position.z;
        }else
        {
            // 根据cmd_vel更新位置（直接积分）
            rmtt_pos.header.stamp = ros::Time::now();
            rmtt_pos.header.frame_id = "world";
            rmtt_pos.pose.position.x = rmtt_pos.pose.position.x + cmd_vel.linear.x * dt;
            rmtt_pos.pose.position.y = rmtt_pos.pose.position.y + cmd_vel.linear.y * dt;
            rmtt_pos.pose.position.z = rmtt_pos.pose.position.z;
        }
        mocap_pos_pub.publish(rmtt_pos);
        break; 

    case MISSION_STATE::RETURN_HOME:
        if(rmtt_orca_state.arrived_goal)
        {
            rmtt_pos.header.stamp = ros::Time::now();
            rmtt_pos.header.frame_id = "world";
            rmtt_pos.pose.position.x = rmtt_orca_state.goal[0];
            rmtt_pos.pose.position.y = rmtt_orca_state.goal[1];
            rmtt_pos.pose.position.z = rmtt_pos.pose.position.z;
        }else
        {
            // 根据cmd_vel更新位置（直接积分）
            rmtt_pos.header.stamp = ros::Time::now();
            rmtt_pos.header.frame_id = "world";
            rmtt_pos.pose.position.x = rmtt_pos.pose.position.x + cmd_vel.linear.x * dt;
            rmtt_pos.pose.position.y = rmtt_pos.pose.position.y + cmd_vel.linear.y * dt;
            rmtt_pos.pose.position.z = rmtt_pos.pose.position.z;
        }
        mocap_pos_pub.publish(rmtt_pos);
        break; 

    default:
        break;
    }

    battery_pub.publish(battery);

    if(!rmtt_orca_state.arrived_goal)
    {
        return false;
    }
    return true;
}

void RMTT_SIM::station_cmd_cb(const sunray_msgs::station_cmd::ConstPtr& msg)
{
    current_station_cmd = *msg; 
    switch(msg->mission_state) 
    {
        case MISSION_STATE::INIT:
            if(msg->uav_id == uav_id || msg->uav_id == 99)
            {
                mission_state = MISSION_STATE::INIT;
            }
            break;

        case MISSION_STATE::TAKEOFF:
            if(msg->uav_id == uav_id || msg->uav_id == 99)
            {
                mission_state = MISSION_STATE::TAKEOFF;
            }
            break;
        case MISSION_STATE::LAND:
            if(msg->uav_id == uav_id || msg->uav_id == 99)
            {
                mission_state = MISSION_STATE::LAND;
            }
            break;
        case MISSION_STATE::HOLD:
            if(msg->uav_id == uav_id || msg->uav_id == 99)
            {
                mission_state = MISSION_STATE::HOLD;
            }
            break;
        case MISSION_STATE::ORCA_SETUP:
            break;
        case MISSION_STATE::ORCA_RUN:
            if(msg->uav_id == uav_id || msg->uav_id == 99)
            {
                mission_state = MISSION_STATE::ORCA_RUN;
                rmtt_orca_state.arrived_goal = false;
            }            
            break;
        case MISSION_STATE::RETURN_HOME:
            if(msg->uav_id == uav_id || msg->uav_id == 99)
            {
                mission_state = MISSION_STATE::RETURN_HOME;
                rmtt_orca_state.arrived_goal = false;
            }    
            break;
    }
}


void RMTT_SIM::station_cmd_cb2(const sunray_msgs::station_cmd::ConstPtr& msg)
{
    current_station_cmd = *msg; 
    switch(msg->mission_state) 
    {
        case MISSION_STATE::INIT:
            if(msg->uav_id == uav_id || msg->uav_id == 99)
            {
                mission_state = MISSION_STATE::INIT;
            }
            break;
        case MISSION_STATE::TAKEOFF:
            if(msg->uav_id == uav_id || msg->uav_id == 99)
            {
                mission_state = MISSION_STATE::TAKEOFF;
            }
            break;
        case MISSION_STATE::LAND:
            if(msg->uav_id == uav_id || msg->uav_id == 99)
            {
                mission_state = MISSION_STATE::LAND;
            }
            break;
        case MISSION_STATE::HOLD:
            if(msg->uav_id == uav_id || msg->uav_id == 99)
            {
                mission_state = MISSION_STATE::HOLD;
            }
            break;
        case MISSION_STATE::ORCA_SETUP:
            break;
        case MISSION_STATE::ORCA_RUN:
            if(msg->uav_id == uav_id || msg->uav_id == 99)
            {
                mission_state = MISSION_STATE::ORCA_RUN;
            }            
            break;
        case MISSION_STATE::RETURN_HOME:
            if(msg->uav_id == uav_id || msg->uav_id == 99)
            {
                mission_state = MISSION_STATE::RETURN_HOME;
            }    
            break;
    }
}

void RMTT_SIM::rmtt_orca_state_cb(const sunray_msgs::rmtt_orca::ConstPtr& msg)
{
    rmtt_orca_state = *msg;
}

void RMTT_SIM::rmtt_cmd_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
    cmd_vel = *msg;
}





