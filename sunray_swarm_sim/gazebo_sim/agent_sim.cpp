#include "agent_sim.h"
// agent_sim仿真节点：
// 仿真逻辑：在仿真模式中，每一个无人车/无人机都需要单独启动一个本节点，来模拟真实无人机/车的运动
// 本节点通过订阅智能体指令话题以及底层控制指令话题，根据期望的指令仿真得到无人机的实时位置，并伪装发布为动捕更新的位置发回至智能体控制节点

void AGENT_SIM::init(ros::NodeHandle& nh)
{
    node_name = ros::this_node::getName();

    // 【参数】智能体类型
    nh.param<int>("agent_type", agent_type, 0);
    // 【参数】智能体编号
    nh.param<int>("agent_id", agent_id, 0);
    // 【参数】智能体初始位置
    nh.param<double>("init_pos_x", agent_pos.pose.position.x, 0.0);
    nh.param<double>("init_pos_y", agent_pos.pose.position.y, 0.0);
    nh.param<double>("init_pos_z", agent_pos.pose.position.z, 0.01);
    nh.param<double>("init_yaw", agent_yaw, 0.0);
    // 【参数】智能体固定高度
    nh.param<float>("agent_height", agent_height, 0.1);

    string agent_name;
    if(agent_type == sunray_msgs::agent_state::RMTT)
    {
        agent_name = "/rmtt_" + std::to_string(agent_id);
    }else if(agent_type == sunray_msgs::agent_state::UGV)
    {
        agent_name = "/ugv_" + std::to_string(agent_id);
    }

    // 【订阅】智能体控制指令 地面站/ORCA等上层算法 -> 本节点 
    agent_cmd_sub = nh.subscribe<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 1, &AGENT_SIM::agent_cmd_cb, this); 
    // 【订阅】智能体底层控制指令 智能体控制节点(rmtt_control/ugv_control) -> 本节点
    agent_cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/sunray_swarm" + agent_name + "/cmd_vel", 1, &AGENT_SIM::agent_cmd_vel_cb, this);
    // 【发布】伪装成动捕的位置数据发布 本节点 -> 智能体控制节点(rmtt_control/ugv_control)
    mocap_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/vrpn_client_node"+ agent_name + "/pose", 1);
    // 【发布】电池电量（在仿真中，电池电量设置为101，代表仿真模式）本节点 -> 智能体控制节点(rmtt_control/ugv_control)
    battery_pub = nh.advertise<std_msgs::Float32>("/sunray_swarm" + agent_name + "/battery", 1);

    battery.data = 101;

    // 初始化无人机初始位置
    agent_pos.header.stamp = ros::Time::now();
    agent_pos.header.frame_id = "world";
    agent_pos.pose.orientation = ros_quaternion_from_rpy(0.0, 0.0, agent_yaw);
    mocap_pos_pub.publish(agent_pos);

    cout << GREEN << node_name << " ---------------> init! " << TAIL << endl;
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
    cout << GREEN << "agent_id: " << agent_id << TAIL << endl;
    cout << GREEN << "init_pos_x: " << agent_pos.pose.position.x << TAIL << endl;
    cout << GREEN << "init_pos_y: " << agent_pos.pose.position.y << TAIL << endl;
    cout << GREEN << "init_pos_z: " << agent_pos.pose.position.z << TAIL << endl;
    cout << GREEN << "init_yaw: " << agent_yaw << TAIL << endl;
    cout << GREEN << "agent_height: " << agent_height << TAIL << endl;
    current_agent_cmd.control_state = sunray_msgs::agent_cmd::INIT;
}

bool AGENT_SIM::mainloop()
{
    // 仿真步长，该循环执行频率为100Hz，因此dt为0.01秒
    float dt = 0.01;
    switch (current_agent_cmd.control_state)
    {
        // 当控制指令为INIT时，无人机/车位置维持不动，不更新
        case sunray_msgs::agent_cmd::INIT:
            agent_pos.header.stamp = ros::Time::now();
            agent_pos.header.frame_id = "world";
            agent_pos.pose.position.x = agent_pos.pose.position.x;
            agent_pos.pose.position.y = agent_pos.pose.position.y;
            agent_pos.pose.position.z = agent_pos.pose.position.z;
            agent_pos.pose.orientation = ros_quaternion_from_rpy(0.0, 0.0, agent_yaw);
            mocap_pos_pub.publish(agent_pos);
            break;

        // 当控制指令为HOLD时，无人机/车位置维持不动，不更新
        case sunray_msgs::agent_cmd::HOLD:
            agent_pos.header.stamp = ros::Time::now();
            agent_pos.header.frame_id = "world";
            agent_pos.pose.position.x = agent_pos.pose.position.x;
            agent_pos.pose.position.y = agent_pos.pose.position.y;
            agent_pos.pose.position.z = agent_pos.pose.position.z;
            agent_pos.pose.orientation = ros_quaternion_from_rpy(0.0, 0.0, agent_yaw);
            mocap_pos_pub.publish(agent_pos);
            break;

        // 当控制指令为POS_CONTROL时，无人机/车位置依据底层控制指令的速度进行更新（一阶积分器模型）
        // 注意：此处底层控制指令是由控制节点中的位置控制算法产生的
        case sunray_msgs::agent_cmd::POS_CONTROL:
            agent_pos.header.stamp = ros::Time::now();
            agent_pos.header.frame_id = "world";
            agent_pos.pose.position.x = agent_pos.pose.position.x + cmd_vel.linear.x * dt;
            agent_pos.pose.position.y = agent_pos.pose.position.y + cmd_vel.linear.y * dt;
            agent_pos.pose.position.z = agent_height;
            agent_yaw = agent_yaw + cmd_vel.angular.z * dt;
            agent_pos.pose.orientation = ros_quaternion_from_rpy(0.0, 0.0, agent_yaw);
            mocap_pos_pub.publish(agent_pos);
            break;

        // 当控制指令为VEL_CONTROL_BODY时，无人机/车位置依据底层控制指令的速度进行更新（一阶积分器模型）
        case sunray_msgs::agent_cmd::VEL_CONTROL_BODY:
            agent_pos.header.stamp = ros::Time::now();
            agent_pos.header.frame_id = "world";
            agent_pos.pose.position.x = agent_pos.pose.position.x + cmd_vel.linear.x * dt;
            agent_pos.pose.position.y = agent_pos.pose.position.y + cmd_vel.linear.y * dt;
            agent_pos.pose.position.z = agent_height;
            agent_yaw = agent_yaw + cmd_vel.angular.z * dt;
            agent_pos.pose.orientation = ros_quaternion_from_rpy(0.0, 0.0, agent_yaw);
            mocap_pos_pub.publish(agent_pos);
            break;

        // 当控制指令为VEL_CONTROL_ENU时，无人机/车位置依据底层控制指令的速度进行更新（一阶积分器模型）
        case sunray_msgs::agent_cmd::VEL_CONTROL_ENU:
            agent_pos.header.stamp = ros::Time::now();
            agent_pos.header.frame_id = "world";
            agent_pos.pose.position.x = agent_pos.pose.position.x + cmd_vel.linear.x * dt;
            agent_pos.pose.position.y = agent_pos.pose.position.y + cmd_vel.linear.y * dt;
            agent_pos.pose.position.z = agent_height;
            agent_yaw = agent_yaw + cmd_vel.angular.z * dt;
            agent_pos.pose.orientation = ros_quaternion_from_rpy(0.0, 0.0, agent_yaw);
            mocap_pos_pub.publish(agent_pos);
            break;

        // 当控制指令为TAKEOFF时，无人机直接高度更新到起飞高度
        case sunray_msgs::agent_cmd::TAKEOFF:
            agent_pos.header.stamp = ros::Time::now();
            agent_pos.header.frame_id = "world";
            agent_pos.pose.position.x = agent_pos.pose.position.x;
            agent_pos.pose.position.y = agent_pos.pose.position.y;
            agent_pos.pose.position.z = agent_height;
            agent_pos.pose.orientation = ros_quaternion_from_rpy(0.0, 0.0, agent_yaw);
            mocap_pos_pub.publish(agent_pos);
            break;

        // 当控制指令为LAND时，无人机直接高度更新到地面高度
        case sunray_msgs::agent_cmd::LAND:
            agent_pos.header.stamp = ros::Time::now();
            agent_pos.header.frame_id = "world";
            agent_pos.pose.position.x = agent_pos.pose.position.x;
            agent_pos.pose.position.y = agent_pos.pose.position.y;
            agent_pos.pose.position.z = 0.05;
            agent_pos.pose.orientation = ros_quaternion_from_rpy(0.0, 0.0, agent_yaw);
            mocap_pos_pub.publish(agent_pos);
            break;
        default:
            break;
    }

    // 定时发布电池状态
    battery_pub.publish(battery);

    return true;
}

void AGENT_SIM::agent_cmd_cb(const sunray_msgs::agent_cmd::ConstPtr& msg)
{
    if(msg->agent_id != agent_id && msg->agent_id != 99)
    {
        return;
    }

    current_agent_cmd = *msg; 
}

void AGENT_SIM::agent_cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg)
{
    // 智能体底层控制指令是车/机体系，而本程序中计算位置更新为惯性系，因此要先将指令转换为惯性系
    cmd_vel = *msg;
    // BODY -> ENU
    float cmd_body[2];
    float cmd_enu[2];
    cmd_body[0] = cmd_vel.linear.x;
    cmd_body[1] = cmd_vel.linear.y;

    cmd_enu[0] = cmd_body[0] * cos(agent_yaw) - cmd_body[1] * sin(agent_yaw);
    cmd_enu[1] = cmd_body[0] * sin(agent_yaw) + cmd_body[1] * cos(agent_yaw);

    cmd_vel.linear.x = cmd_enu[0];
    cmd_vel.linear.y = cmd_enu[1]; 

    // cout << GREEN << "cmd_vel.linear.x"<< cmd_vel.linear.x  << TAIL << endl;
}

// 从(roll,pitch,yaw)创建四元数  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
geometry_msgs::Quaternion AGENT_SIM::ros_quaternion_from_rpy(double roll, double pitch, double yaw)
{
    Eigen::Quaterniond q = Eigen::Quaterniond(
                        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));

    geometry_msgs::Quaternion ros_q;

    ros_q.x = q.x();
    ros_q.y = q.y();
    ros_q.z = q.z();
    ros_q.w = q.w();

    return ros_q;    
}



