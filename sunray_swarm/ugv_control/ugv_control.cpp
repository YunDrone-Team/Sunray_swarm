#include "ugv_control.h"
    
void UGV_CONTROL::init(ros::NodeHandle& nh)
{
    // 【参数】智能体类型
    nh.param<int>("agent_type", agent_type, 1);
    // 【参数】智能体编号
    nh.param<int>("agent_id", agent_id, 1);
    // 【参数】智能体IP
    nh.param<std::string>("agent_ip", agent_ip, "192.168.1.1");
    // 【参数】智能体高度
    nh.param<float>("agent_height", agent_height, 1.0);
    // 【参数】追踪目标名称
    nh.param<std::string>("target_name", target_name, "none");
    // 【参数】sim_mode
    nh.param<bool>("sim_mode", sim_mode, false);
    // 【参数】desired_yaw
    nh.param<double>("desired_yaw", desired_yaw, 0.0f);
    // 【参数】是否打印
    nh.param<bool>("flag_printf", flag_printf, true);
    // 【参数】悬停控制参数 - xy
    nh.param<float>("ugv_control_param/Kp_xy", ugv_control_param.Kp_xy, 1.5);
    // 【参数】悬停控制参数 - yaw
    nh.param<float>("ugv_control_param/Kp_yaw", ugv_control_param.Kp_yaw, 0.8);
    // 【参数】悬停控制参数 - max_vel_xy
    nh.param<float>("ugv_control_param/max_vel_xy", ugv_control_param.max_vel_xy, 0.3);
    // 【参数】悬停控制参数 - max_vel_yaw
    nh.param<float>("ugv_control_param/max_vel_yaw", ugv_control_param.max_vel_yaw, 10.0/180.0*M_PI);
    // 【参数】地理围栏参数（超出围栏自动降落）
    nh.param<float>("ugv_geo_fence/max_x", ugv_geo_fence.max_x, 3.5);
    nh.param<float>("ugv_geo_fence/min_x", ugv_geo_fence.min_x, -3.5);
    nh.param<float>("ugv_geo_fence/max_y", ugv_geo_fence.max_y, 4.0);
    nh.param<float>("ugv_geo_fence/min_y", ugv_geo_fence.min_y, -4.0);
    nh.param<float>("ugv_geo_fence/max_z", ugv_geo_fence.max_z, 2.0);
    nh.param<float>("ugv_geo_fence/min_z", ugv_geo_fence.min_z, -0.1);

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
    // 【订阅】订阅动捕的数据(位置+速度) vrpn -> 本节点
    mocap_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node"+ agent_name + "/pose", 1, &UGV_CONTROL::mocap_pos_cb, this);
    mocap_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/vrpn_client_node"+ agent_name + "/twist", 1, &UGV_CONTROL::mocap_vel_cb, this);
    // 【订阅】地面站指令 地面站 -> 本节点
    ugv_cmd_sub = nh.subscribe<sunray_msgs::ugv_cmd>("/sunray_swarm" + agent_name + "/ugv_cmd", 1, &UGV_CONTROL::ugv_cmd_cb, this);
    // 【订阅】ugv电池的数据 ugv_driver -> 本节点
    battery_sub = nh.subscribe<std_msgs::Float32>("/sunray_swarm" + agent_name + "/battery", 1, &UGV_CONTROL::battery_cb, this);  
    // 【订阅】ORCA算法计算得到的期望速度 orca -> 本节点
    orca_cmd_sub = nh.subscribe<geometry_msgs::Twist>("/sunray_swarm" + agent_name + "/orca_cmd_vel", 1, &UGV_CONTROL::orca_cmd_vel_cb, this);  
    // 【订阅】ORCA算法状态 orca -> 本节点
    ugv_orca_state_sub = nh.subscribe<sunray_msgs::orca_state>("/sunray_swarm" + agent_name + "/agent_orca_state", 1, &UGV_CONTROL::ugv_orca_state_cb, this);    
    // 【订阅】订阅目标动捕数据 vrpn -> 本节点
    mocap_target_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/"+ target_name + "/pose", 1, &UGV_CONTROL::mocap_target_pos_cb, this);
    
    // 【发布】控制指令（机体系，单位：米/秒，Rad/秒）
    ugv_cmd_pub = nh.advertise<geometry_msgs::Twist>("/sunray_swarm" + agent_name + "/cmd_vel", 1); 
    // 【发布】led灯 本节点 -> ugv_driver
    led_pub = nh.advertise<std_msgs::ColorRGBA>("/sunray_swarm" + agent_name + "/led", 1);
    // 【发布】智能体状态 本节点 -> 地面站
    agent_state_pub = nh.advertise<sunray_msgs::agent_state>("/sunray_swarm" + agent_name + "/agent_state", 1); 
    // 【发布】无人车marker 本节点 -> RVIZ
    ugv_mesh_pub = nh.advertise<visualization_msgs::Marker>("/sunray_swarm" + agent_name + "/mesh", 1);
    // 【发布】无人车运动轨迹  本节点 -> RVIZ
    ugv_trajectory_pub = nh.advertise<nav_msgs::Path>("/sunray_swarm" + agent_name + "/trajectory", 1);
    // 【发布】文字提示消息  本节点 -> 地面站
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);
    // 【发布】目标点marker 本节点 -> RVIZ
    goal_point_pub = nh.advertise<visualization_msgs::Marker>("/sunray_swarm" + agent_name + "/goal_point_rviz", 1);

    // 【定时器】 定时发布ugv_state - 10Hz
    timer_state_pub = nh.createTimer(ros::Duration(0.1), &UGV_CONTROL::timercb_state, this);
    // 【定时器】 定时发布RVIZ显示相关话题 - 10Hz
    timer_rivz = nh.createTimer(ros::Duration(0.1), &UGV_CONTROL::timercb_rviz, this);
    // 【定时器】 定时打印 - 1Hz
    timer_debug = nh.createTimer(ros::Duration(3.0), &UGV_CONTROL::timercb_debug, this);

    ugv_state.header.stamp = ros::Time::now();
    ugv_state.header.frame_id = "world";
    ugv_state.agent_type = agent_type;
    ugv_state.agent_id = agent_id;
    ugv_state.agent_ip = agent_ip;  
    ugv_state.connected = false;
    ugv_state.odom_valid = false;
    ugv_state.pos[0] = 0.0;
    ugv_state.pos[1] = 0.0;
    ugv_state.pos[2] = agent_height;
    ugv_state.vel[0] = 0.0;
    ugv_state.vel[1] = 0.0;
    ugv_state.vel[2] = 0.0;
    ugv_state.att[0] = 0.0;
    ugv_state.att[1] = 0.0;
    ugv_state.att[2] = 0.0;
    ugv_state.attitude_q.x = 0.0;
    ugv_state.attitude_q.y = 0.0;
    ugv_state.attitude_q.z = 0.0;
    ugv_state.attitude_q.w = 1.0;
    ugv_state.battery = -1.0;
    ugv_state.control_state = CONTROL_STATE::INIT;
    control_state = CONTROL_STATE::INIT;

    setup_color();
    printf_param();

    node_name = ros::this_node::getName();
    text_info.data = node_name + ": ugv_" + to_string(agent_id) + " init!";
    text_info_pub.publish(text_info);
    cout << BLUE << text_info.data << TAIL << endl;
}

void UGV_CONTROL::mainloop()
{
    check_geo_fence();

    // 动捕丢失情况下，不执行控制指令，直到动捕恢复
    if(!ugv_state.odom_valid)
    {
        cmd_vel_orca_body.linear.x = 0.0;
        cmd_vel_orca_body.linear.y = 0.0;
        cmd_vel_orca_body.linear.z = 0.0;
        cmd_vel_orca_body.angular.x = 0.0;
        cmd_vel_orca_body.angular.y = 0.0;
        cmd_vel_orca_body.angular.z = 0.0;
        ugv_cmd_pub.publish(cmd_vel_orca_body);
        return;
    }

    switch (control_state)
    {
    case CONTROL_STATE::INIT:
        // 初始模式
        // do nothing
        break;

    case CONTROL_STATE::HOLD:
        // 原地停止
        desired_vel.linear.x = 0.0;
        desired_vel.linear.y = 0.0;
        desired_vel.linear.z = 0.0;
        desired_vel.angular.z = 0.0;
        ugv_cmd_pub.publish(desired_vel);
        break;

    case CONTROL_STATE::POS_CONTROL:
        // 位置控制
        pos_control(desired_position, desired_yaw);
        break;

    case CONTROL_STATE::VEL_CONTROL:
        // 速度控制 
        // 控制指令限幅
        desired_vel.linear.x = constrain_function(desired_vel.linear.x, ugv_control_param.max_vel_xy, 0.0);
        desired_vel.linear.y = constrain_function(desired_vel.linear.y, ugv_control_param.max_vel_xy, 0.0);
        desired_vel.angular.z = constrain_function(desired_vel.angular.z, ugv_control_param.max_vel_yaw, 0.01);
        // 发布控制指令
        ugv_cmd_pub.publish(desired_vel);        
        break;

    case CONTROL_STATE::ORCA_MODE:
        // ORCA控制模式
        if(ugv_orca_state.arrived_goal)
        {
            desired_position.x = ugv_orca_state.goal[0];
            desired_position.y = ugv_orca_state.goal[1];
            desired_position.z = agent_height;
            pos_control(desired_position, desired_yaw);
        }else
        {
            // orca control
            orca_control();
        }
        break; 

    case CONTROL_STATE::TRACK_MODE:
        // 追踪控制模式 TODO
        pos_control(target_pos, target_yaw);
        break;

    case CONTROL_STATE::RETURN_HOME:
        // 返回起始点
        if(ugv_orca_state.arrived_goal)
        {
            control_state = CONTROL_STATE::INIT;
        }else
        {
            // orca control
            orca_control();
        }
        break; 
    default:
        break;
    }

    ugv_state_last = ugv_state;
}

void UGV_CONTROL::orca_control()
{
    // 惯性系 -> body frame 
    float orca_cmd_body[2];
    float orca_cmd_enu[2];
    orca_cmd_enu[0] = cmd_vel_orca_enu.linear.x;
    orca_cmd_enu[1] = cmd_vel_orca_enu.linear.y;
    rotation_yaw(ugv_state.att[2], orca_cmd_body, orca_cmd_enu);   
    cmd_vel_orca_body.linear.x = orca_cmd_body[0];
    cmd_vel_orca_body.linear.y = orca_cmd_body[1];
    // 控制指令计算：使用简易P控制 - Z
    cmd_vel_orca_body.linear.z = 0.0;
    // 控制指令计算：使用简易P控制 - YAW
    cmd_vel_orca_body.angular.x = 0.0;
    cmd_vel_orca_body.angular.y = 0.0;

    double yaw_error = get_yaw_error(desired_yaw, ugv_state.att[2]);
    cmd_vel_orca_body.angular.z = yaw_error * ugv_control_param.Kp_yaw;

    // 控制指令限幅
    cmd_vel_orca_body.linear.x = constrain_function(cmd_vel_orca_body.linear.x, ugv_control_param.max_vel_xy, 0.0);
    cmd_vel_orca_body.linear.y = constrain_function(cmd_vel_orca_body.linear.y, ugv_control_param.max_vel_xy, 0.0);
    cmd_vel_orca_body.angular.z = constrain_function(cmd_vel_orca_body.angular.z, ugv_control_param.max_vel_yaw, 0.01);

    ugv_cmd_pub.publish(cmd_vel_orca_body);
}

void UGV_CONTROL::set_desired_position()
{
    desired_position.x = ugv_state.pos[0];
    desired_position.y = ugv_state.pos[1];
    desired_position.z = 0.0;
}

double UGV_CONTROL::get_yaw_error(double yaw_ref, double yaw_now)
{
    double error = yaw_ref - yaw_now;

    if(error > M_PI)
    {
        error = error - 2*M_PI;
    }else if(error < -M_PI)
    {
        error = error + 2*M_PI;
    }

    return error;
}

void UGV_CONTROL::pos_control(geometry_msgs::Point pos_ref, double yaw_ref)
{
    float cmd_body[2];
    float cmd_enu[2];
    // 控制指令计算：使用简易P控制 - XY
    cmd_enu[0] = (pos_ref.x - ugv_state.pos[0]) * ugv_control_param.Kp_xy;
    cmd_enu[1] = (pos_ref.y - ugv_state.pos[1]) * ugv_control_param.Kp_xy;
    // 惯性系 -> body frame
    rotation_yaw(ugv_state.att[2], cmd_body, cmd_enu);             
    desired_vel.linear.x = cmd_body[0];
    desired_vel.linear.y = cmd_body[1];
    desired_vel.linear.z = 0.0;
    // YAW误差计算
    double yaw_error = get_yaw_error(yaw_ref, ugv_state.att[2]);
    // 控制指令计算：使用简易P控制 - YAW
    desired_vel.angular.z = yaw_error * ugv_control_param.Kp_yaw;

    // 控制指令限幅
    desired_vel.linear.x = constrain_function(desired_vel.linear.x, ugv_control_param.max_vel_xy, 0.0);
    desired_vel.linear.y = constrain_function(desired_vel.linear.y, ugv_control_param.max_vel_xy, 0.0);
    desired_vel.angular.z = constrain_function(desired_vel.angular.z, ugv_control_param.max_vel_yaw, 0.01);

    // 发布控制指令
    ugv_cmd_pub.publish(desired_vel);
}

// 【坐标系旋转函数】- enu系到body系
void UGV_CONTROL::rotation_yaw(double yaw_angle, float body_frame[2], float enu_frame[2])
{
    body_frame[0] = enu_frame[0] * cos(yaw_angle) + enu_frame[1] * sin(yaw_angle);
    body_frame[1] = -enu_frame[0] * sin(yaw_angle) + enu_frame[1] * cos(yaw_angle);
}

void UGV_CONTROL::ugv_cmd_cb(const sunray_msgs::ugv_cmd::ConstPtr& msg)
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
            text_info.data = node_name + ": ugv_" + to_string(agent_id) + " Get ugv_cmd: INIT!";
            cout << BLUE << text_info.data << TAIL << endl;
            break;
        case CONTROL_STATE::HOLD:
            control_state = CONTROL_STATE::HOLD;
            text_info.data = node_name + ": ugv_" + to_string(agent_id) + " Get ugv_cmd: HOLD!";
            cout << BLUE << text_info.data << TAIL << endl;
            break;
        case CONTROL_STATE::POS_CONTROL:
            desired_position = msg->desired_pos;
            desired_yaw = msg->desired_yaw;
            control_state = CONTROL_STATE::POS_CONTROL;
            text_info.data = node_name + ": ugv_" + to_string(agent_id) + " Get ugv_cmd: POS_CONTROL!";
            cout << BLUE << text_info.data << TAIL << endl;
            break;
        case CONTROL_STATE::VEL_CONTROL:
            desired_vel = msg->desired_vel;
            control_state = CONTROL_STATE::VEL_CONTROL;
            text_info.data = node_name + ": ugv_" + to_string(agent_id) + " Get ugv_cmd: VEL_CONTROL!";
            cout << BLUE << text_info.data << TAIL << endl;
            break;
        case CONTROL_STATE::ORCA_MODE:
            control_state = CONTROL_STATE::ORCA_MODE;
            text_info.data = node_name + ": ugv_" + to_string(agent_id) + " Get ugv_cmd: ORCA_MODE!";
            cout << BLUE << text_info.data << TAIL << endl;
            break;
        case CONTROL_STATE::TRACK_MODE:
            if(get_target_pos)
            {
                control_state = CONTROL_STATE::TRACK_MODE;
                text_info.data = node_name + ": ugv_" + to_string(agent_id) + " Get ugv_cmd: TRACK_MODE!";
                cout << BLUE << text_info.data << TAIL << endl;
            }
            break;
        case CONTROL_STATE::RETURN_HOME:
            control_state = CONTROL_STATE::RETURN_HOME;
            text_info.data = node_name + ": ugv_" + to_string(agent_id) + " Get ugv_cmd: RETURN_HOME!";
            cout << BLUE << text_info.data << TAIL << endl;
            sleep(1.0);
            ugv_orca_state.arrived_goal = false;
            break;
        default:
            text_info.data = node_name + ": ugv_" + to_string(agent_id) + " Get ugv_cmd: Wrong!";
            cout << RED << text_info.data << TAIL << endl;
            break;
    }
    text_info_pub.publish(text_info);
}

void UGV_CONTROL::ugv_orca_state_cb(const sunray_msgs::orca_stateConstPtr& msg)
{
    ugv_orca_state = *msg;
}

//定时打印
void UGV_CONTROL::timercb_debug(const ros::TimerEvent &e)
{
    if(!flag_printf)
    {
        return;
    }
    cout << GREEN << ">>>>>>>>>>>>>> UGV [" << agent_id << "] Control ";
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

    if(ugv_state.battery < 15.0f)
    {
        cout << RED << "Battery: " << ugv_state.battery << " [%] <<<<<<<<<<<<<" << TAIL << endl;
    }else if(ugv_state.battery < 30.0f)
    {
        cout << YELLOW << "Battery: " << ugv_state.battery << " [%] <<<<<<<<<<<<<" << TAIL << endl;
    }else
    {
        cout << GREEN << "Battery: " << ugv_state.battery << " [%] <<<<<<<<<<<<<" << TAIL << endl;
    }

    cout << GREEN << "UAV_pos [X Y] : " << ugv_state.pos[0] << " [ m ] " << ugv_state.pos[1] << " [ m ] " << TAIL << endl;
    cout << GREEN << "UAV_vel [X Y] : " << ugv_state.vel[0] << " [m/s] " << ugv_state.vel[1] << " [m/s] " << TAIL << endl;
    cout << GREEN << "UAV_att [Yaw] : " << ugv_state.att[2] * 180 / M_PI << " [deg] " << TAIL << endl;

    // 动捕丢失情况下，不执行控制指令，直到动捕恢复
    if(!ugv_state.odom_valid)
    {
        cout << RED << "Odom_valid: [Invalid] <<<<<<<<<<<<<" << TAIL << endl;
        return;
    }

    //集群控制命令状态打印
    if (control_state == CONTROL_STATE::INIT)
    {
        cout << GREEN << "CONTROL_STATE : [ INIT ]" << TAIL << endl;
    }
    else if (control_state == CONTROL_STATE::HOLD)
    {
        cout << GREEN << "CONTROL_STATE : [ HOLD ]" << TAIL << endl;
    }
    else if (control_state == CONTROL_STATE::POS_CONTROL)
    {
        cout << GREEN << "CONTROL_STATE : [ POS_CONTROL ]" << TAIL << endl;
        cout << GREEN << "POS_REF [X Y] : " << desired_position.x << " [ m ] " << desired_position.y << " [ m ] " << TAIL << endl;
        cout << GREEN << "YAW_REF       : " << desired_yaw * 180 / M_PI << " [deg] " << TAIL << endl;    
        cout << GREEN << "CMD_PUB [X Y] : " << desired_vel.linear.x   << " [m/s] " << desired_vel.linear.y  << " [m/s] " << TAIL << endl;
        cout << GREEN << "CMD_PUB [Yaw] : " << desired_vel.angular.z * 180 / M_PI << " [deg/s] " << TAIL << endl;
    }
    else if (control_state == CONTROL_STATE::VEL_CONTROL)
    {
        cout << GREEN << "CONTROL_STATE : [ VEL_CONTROL ]" << TAIL << endl;
        cout << GREEN << "CMD_PUB [X Y] : " << desired_vel.linear.x   << " [m/s] " << desired_vel.linear.y  << " [m/s] " << TAIL << endl;
        cout << GREEN << "CMD_PUB [Yaw] : " << desired_vel.angular.z * 180 / M_PI << " [deg/s] " << TAIL << endl;
    }
    else if (control_state == CONTROL_STATE::ORCA_MODE)
    {
        cout << GREEN << "CONTROL_STATE : [ ORCA_MODE ]  " ;
        if(ugv_orca_state.arrived_goal)
        {
            cout << GREEN << "Arrived_goal : [ true ]" << TAIL << endl;
        }else
        {
            cout << RED << "Arrived_goal : [ false ]" << TAIL << endl;
        }
        if(ugv_orca_state.arrived_goal)
        {
            cout << GREEN << "GOAL     [X Y] : " << ugv_orca_state.goal[0]      << " [ m ] " << ugv_orca_state.goal[1]     << " [ m ] " << TAIL << endl;
            cout << GREEN << "CMD_PUB  [X Y] : " << desired_vel.linear.x   << " [m/s] " << desired_vel.linear.y  << " [m/s] " << TAIL << endl;
        }else
        {
            cout << GREEN << "GOAL     [X Y] : " << ugv_orca_state.goal[0]      << " [ m ] " << ugv_orca_state.goal[1]     << " [ m ] " << TAIL << endl;
            cout << GREEN << "CMD_ORCA [X Y] : " << ugv_orca_state.vel_orca[0]  << " [m/s] " << ugv_orca_state.vel_orca[1] << " [m/s] " << TAIL << endl;
            cout << GREEN << "CMD_PUB  [X Y] : " << cmd_vel_orca_body.linear.x   << " [m/s] " << cmd_vel_orca_body.linear.y  << " [m/s] " << TAIL << endl;
            cout << GREEN << "CMD_PUB  [Yaw] : " << cmd_vel_orca_body.angular.z * 180 / M_PI << " [deg/s] " << TAIL << endl;
        }
    }
    else if (control_state == CONTROL_STATE::TRACK_MODE)
    {
        cout << GREEN << "CONTROL_STATE : [ TRACK_MODE ]" << TAIL << endl;
        cout << GREEN << "POS_REF [X Y] : " << target_pos.x << " [ m ] " << target_pos.y << " [ m ] " << TAIL << endl;
        cout << GREEN << "YAW_REF       : " << target_yaw * 180 / M_PI << " [deg] " << TAIL << endl;    
        cout << GREEN << "CMD_PUB [X Y] : " << desired_vel.linear.x   << " [m/s] " << desired_vel.linear.y  << " [m/s] " << TAIL << endl;
        cout << GREEN << "CMD_PUB [Yaw] : " << desired_vel.angular.z * 180 / M_PI << " [deg/s] " << TAIL << endl;
    }
    else if (control_state == CONTROL_STATE::RETURN_HOME)
    {
        cout << GREEN << "CONTROL_STATE : [ RETURN_HOME ]  " ;
        if(ugv_orca_state.arrived_goal)
        {
            cout << GREEN << "Arrived_goal : [ true ]" << TAIL << endl;
        }else
        {
            cout << RED << "Arrived_goal : [ false ]" << TAIL << endl;
        }
        if(ugv_orca_state.arrived_goal)
        {
            cout << GREEN << "GOAL     [X Y] : " << ugv_orca_state.goal[0]      << " [ m ] " << ugv_orca_state.goal[1]     << " [ m ] " << TAIL << endl;
            cout << GREEN << "CMD_PUB  [X Y] : " << desired_vel.linear.x   << " [m/s] " << desired_vel.linear.y  << " [m/s] " << TAIL << endl;
        }else
        {
            cout << GREEN << "GOAL     [X Y] : " << ugv_orca_state.goal[0]      << " [ m ] " << ugv_orca_state.goal[1]     << " [ m ] " << TAIL << endl;
            cout << GREEN << "CMD_ORCA [X Y] : " << ugv_orca_state.vel_orca[0]  << " [m/s] " << ugv_orca_state.vel_orca[1] << " [m/s] " << TAIL << endl;
            cout << GREEN << "CMD_PUB  [X Y] : " << cmd_vel_orca_body.linear.x   << " [m/s] " << cmd_vel_orca_body.linear.y  << " [m/s] " << TAIL << endl;
        }
    }
    else
    {
        cout << RED << "CONTROL_STATE : [ ERROR ]" << TAIL << endl;
    }
}

void UGV_CONTROL::orca_cmd_vel_cb(const geometry_msgs::TwistConstPtr& msg)
{
    if(control_state == CONTROL_STATE::ORCA_MODE || control_state == CONTROL_STATE::RETURN_HOME)
    {
        cmd_vel_orca_enu = *msg;
    }else
    {
        cmd_vel_orca_enu.linear.x = 0.0;
        cmd_vel_orca_enu.linear.y = 0.0;
        cmd_vel_orca_enu.linear.z = 0.0;
        cmd_vel_orca_enu.angular.x = 0.0;
        cmd_vel_orca_enu.angular.y = 0.0;
        cmd_vel_orca_enu.angular.z = 0.0; 
    }
}

//定时发布ugv_state
void UGV_CONTROL::timercb_state(const ros::TimerEvent &e)
{
    // 发布 ugv_state
    ugv_state.header.stamp = ros::Time::now();

    if((ros::Time::now() - get_mocap_time).toSec() > MOCAP_TIMEOUT)
    {
        ugv_state.odom_valid = false;
    }

    ugv_state.control_state = control_state;
    agent_state_pub.publish(ugv_state);
}

void UGV_CONTROL::mocap_target_pos_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
    get_target_pos = true;
    target_pos.x = msg->pose.position.x;
    target_pos.y = msg->pose.position.y;
    target_pos.z = msg->pose.position.z;

    Eigen::Quaterniond q_mocap = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Vector3d target_att = quaternion_to_euler(q_mocap);
    target_yaw = target_att.z();
}

void UGV_CONTROL::mocap_pos_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
    get_mocap_time = ros::Time::now(); // 记录时间戳，防止超时
	ugv_state.pos[0] = msg->pose.position.x;
    ugv_state.pos[1] = msg->pose.position.y;
	ugv_state.pos[2] = agent_height;
    ugv_state.attitude_q = msg->pose.orientation;

    Eigen::Quaterniond q_mocap = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Vector3d agent_att = quaternion_to_euler(q_mocap);

	ugv_state.att[0] = agent_att.x();
    ugv_state.att[1] = agent_att.y();
	ugv_state.att[2] = agent_att.z();

    ugv_state.odom_valid = true;
}

void UGV_CONTROL::mocap_vel_cb(const geometry_msgs::TwistStampedConstPtr& msg)
{
	ugv_state.vel[0] = msg->twist.linear.x;
    ugv_state.vel[1] = msg->twist.linear.y;
	ugv_state.vel[2] = msg->twist.linear.z;
}

void UGV_CONTROL::battery_cb(const std_msgs::Float32ConstPtr& msg)
{
    ugv_state.connected = true;
    ugv_state.battery = msg->data;
}

void UGV_CONTROL::setup_color()
{
    led_color.a = 1.0;
    switch(agent_id) // 控制仿真时RVIZ中无人机的颜色，与真机无关
    {
        case 1:
            led_color.r = 1.0;
            led_color.g = 0.0;
            led_color.b = 0.0;
            break;
        case 2:
            led_color.r = 0.0;
            led_color.g = 1.0;
            led_color.b = 0.0;
            break;
        case 3:
            led_color.r = 0.0;
            led_color.g = 0.0;
            led_color.b = 1.0;
            break;
        case 4:
            led_color.r = 1.0;
            led_color.g = 1.0;
            led_color.b = 0.0;
            break;
        case 5:
            led_color.r = 1.0;
            led_color.g = 0.0;
            led_color.b = 1.0;
            break;
        case 6:
            led_color.r = 0.0;
            led_color.g = 1.0;
            led_color.b = 1.0;
            break;
        case 7:
            led_color.r = 0.5;
            led_color.g = 0.5;
            led_color.b = 0.5;
            break;
        case 8:
            led_color.r = 0.3;
            led_color.g = 0.7;
            led_color.b = 0.2;
            break;
        default:
            led_color.r = 1.0;
            led_color.g = 1.0;
            led_color.b = 1.0;
            break;
    }
    // led_color.r = int(led_color.r * 255);
    // led_color.g = int(led_color.g * 255);
    // led_color.b = int(led_color.b * 255);
}

void UGV_CONTROL::setup_led()
{
    std_msgs::ColorRGBA ugv_led;
    ugv_led.r = int(led_color.r * 255);
    ugv_led.g = int(led_color.g * 255);
    ugv_led.b = int(led_color.b * 255);
    ugv_led.a = led_color.a;
    led_pub.publish(ugv_led);
}

void UGV_CONTROL::timercb_rviz(const ros::TimerEvent &e)
{
    // 发布无人机marker
    visualization_msgs::Marker ugv_marker;
    ugv_marker.header.frame_id = "world";
    ugv_marker.header.stamp = ros::Time::now();
    ugv_marker.ns = "mesh";
    ugv_marker.id = 0;
    ugv_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    ugv_marker.scale.x = 0.1;  
    ugv_marker.scale.y = 0.1;  
    ugv_marker.scale.z = 0.1;  
    ugv_marker.action = visualization_msgs::Marker::ADD;
    ugv_marker.pose.position.x = ugv_state.pos[0];
    ugv_marker.pose.position.y = ugv_state.pos[1];
    ugv_marker.pose.position.z = ugv_state.pos[2];
    ugv_marker.pose.orientation.w = ugv_state.attitude_q.w;
    ugv_marker.pose.orientation.x = ugv_state.attitude_q.x;
    ugv_marker.pose.orientation.y = ugv_state.attitude_q.y;
    ugv_marker.pose.orientation.z = ugv_state.attitude_q.z;
    ugv_marker.color = led_color;
    ugv_marker.mesh_use_embedded_materials = false;
    ugv_marker.mesh_resource = std::string("package://sunray_swarm/meshes/car.dae");
    ugv_mesh_pub.publish(ugv_marker);

    // 发布无人机运动轨迹，用于rviz显示
    geometry_msgs::PoseStamped ugv_pos;
    ugv_pos.header.stamp = ros::Time::now();
    ugv_pos.header.frame_id = "world";
    ugv_pos.pose.position.x = ugv_state.pos[0];
    ugv_pos.pose.position.y = ugv_state.pos[1];
    ugv_pos.pose.position.z = ugv_state.pos[2];
    ugv_pos.pose.orientation = ugv_state.attitude_q;
    pos_vector.insert(pos_vector.begin(), ugv_pos);
    if (pos_vector.size() > TRA_WINDOW)
    {
        pos_vector.pop_back();
    }
    nav_msgs::Path ugv_trajectory;
    ugv_trajectory.header.stamp = ros::Time::now();
    ugv_trajectory.header.frame_id = "world";
    ugv_trajectory.poses = pos_vector;
    ugv_trajectory_pub.publish(ugv_trajectory);

    // 发布目标点mesh
    visualization_msgs::Marker goal_marker;
    goal_marker.header.frame_id = "world";
    goal_marker.header.stamp = ros::Time::now();
    goal_marker.ns = "ugv_goal";
    goal_marker.id = agent_id;
    goal_marker.type = visualization_msgs::Marker::SPHERE;
    goal_marker.action = visualization_msgs::Marker::ADD;
    goal_marker.pose.position.x = ugv_orca_state.goal[0];
    goal_marker.pose.position.y = ugv_orca_state.goal[1];
    goal_marker.pose.position.z = agent_height;
    goal_marker.pose.orientation.x = 0.0;
    goal_marker.pose.orientation.y = 0.0;
    goal_marker.pose.orientation.z = 0.0;
    goal_marker.pose.orientation.w = 1.0;
    goal_marker.scale.x = 0.2;
    goal_marker.scale.y = 0.2;
    goal_marker.scale.z = 0.2;
    goal_marker.color = led_color;
    ugv_marker.mesh_use_embedded_materials = false;
    goal_point_pub.publish(goal_marker);
}

float UGV_CONTROL::constrain_function(float data, float Max, float Min)
{
    if(abs(data)>Max)
    {
        return (data > 0) ? Max : -Max;
    }else if(abs(data)<Min)
    {
        return 0.0;
    }
    else
    {
        return data;
    }
}

void UGV_CONTROL::check_geo_fence()
{
    // 安全检查，超出地理围栏自动降落,打印相关位置信息
    if (ugv_state.pos[0] > ugv_geo_fence.max_x || ugv_state.pos[0] < ugv_geo_fence.min_x || 
        ugv_state.pos[1] > ugv_geo_fence.max_y || ugv_state.pos[1] < ugv_geo_fence.min_y || 
        ugv_state.pos[2] > ugv_geo_fence.max_z || ugv_state.pos[2] < ugv_geo_fence.min_z)
    {
        control_state = CONTROL_STATE::INIT;
        ROS_WARN_STREAM("ugv [" << agent_id << "] out of geofence land! Position: [" 
                        << ugv_state.pos[0] << ", " << ugv_state.pos[1] << ", " 
                        << ugv_state.pos[2] << "], Geofence: ["
                        << ugv_geo_fence.min_x << ", " << ugv_geo_fence.max_x << ", "
                        << ugv_geo_fence.min_y << ", " << ugv_geo_fence.max_y << ", "
                        << ugv_geo_fence.min_z << ", " << ugv_geo_fence.max_z << "]");
    }
}

Eigen::Vector3d UGV_CONTROL::quaternion_to_euler(const Eigen::Quaterniond &q)
{
    float quat[4];
    quat[0] = q.w();
    quat[1] = q.x();
    quat[2] = q.y();
    quat[3] = q.z();

    Eigen::Vector3d ans;
    ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
    return ans;
}

void UGV_CONTROL::printf_param()
{
    cout << GREEN << ">>>>>>>>>>>>>>>>>>> UGV_CONTROL Parameters <<<<<<<<<<<<<<<<" << TAIL << endl;

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
    cout << GREEN << "agent_id : " << agent_id << "" << TAIL << endl;
    cout << GREEN << "agent_ip : " << agent_ip << "" << TAIL << endl;
    cout << GREEN << "sim_mode : " << sim_mode << "" << TAIL << endl;
    cout << GREEN << "flag_printf : " << flag_printf << "" << TAIL << endl;
    cout << GREEN << "agent_height : " << agent_height << TAIL << endl;

    // 悬停控制参数
    cout << GREEN << "Kp_xy : " << ugv_control_param.Kp_xy << TAIL << endl;
    cout << GREEN << "Kp_z : " << ugv_control_param.Kp_z << TAIL << endl;
    cout << GREEN << "Kp_yaw : " << ugv_control_param.Kp_yaw << TAIL << endl;
    cout << GREEN << "max_vel_xy : " << ugv_control_param.max_vel_xy << " [m/s]" << TAIL << endl;
    cout << GREEN << "max_vel_z : " << ugv_control_param.max_vel_z << " [m/s]" << TAIL << endl;
    cout << GREEN << "max_vel_yaw : " << ugv_control_param.max_vel_yaw << " [rad/s]" << TAIL << endl;

    // 地理围栏参数
    cout << GREEN << "geo_fence max_x : " << ugv_geo_fence.max_x << " [m]" << TAIL << endl;
    cout << GREEN << "geo_fence min_x : " << ugv_geo_fence.min_x << " [m]" << TAIL << endl;
    cout << GREEN << "geo_fence max_y : " << ugv_geo_fence.max_y << " [m]" << TAIL << endl;
    cout << GREEN << "geo_fence min_y : " << ugv_geo_fence.min_y << " [m]" << TAIL << endl;
    cout << GREEN << "geo_fence max_z : " << ugv_geo_fence.max_z << " [m]" << TAIL << endl;
    cout << GREEN << "geo_fence min_z : " << ugv_geo_fence.min_z << " [m]" << TAIL << endl;
}