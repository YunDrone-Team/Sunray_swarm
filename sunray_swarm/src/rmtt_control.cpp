#include "rmtt_control.h"
    
void RMTT_CONTROL::init(ros::NodeHandle& nh)
{
    // 【参数】无人机编号
    nh.param<int>("uav_id", uav_id, 1);
    // 【参数】无人机IP
    nh.param<std::string>("uav_ip", uav_ip, "192.168.1.1");
    // 【参数】无人机固定飞行高度
    nh.param<float>("rmtt_height", rmtt_height, 1.0);
    // 【参数】sim_mode
    nh.param<bool>("sim_mode", sim_mode, false);
    // 【参数】sim_mode
    nh.param<bool>("track_mode", track_mode, false);
    // 【参数】desired_yaw
    nh.param<double>("desired_yaw", desired_yaw, 0.0f);
    // 【参数】是否打印
    nh.param<bool>("flag_printf", flag_printf, true);
    // 【参数】悬停控制参数 - xy
    nh.param<float>("rmtt_control_param/Kp_xy", rmtt_control_param.Kp_xy, 1.5);
    // 【参数】悬停控制参数 - z
    nh.param<float>("rmtt_control_param/Kp_z", rmtt_control_param.Kp_z, 1.5);
    // 【参数】悬停控制参数 - yaw
    nh.param<float>("rmtt_control_param/Kp_yaw", rmtt_control_param.Kp_yaw, 0.8);
    // 【参数】悬停控制参数 - max_vel_xy
    nh.param<float>("rmtt_control_param/max_vel_xy", rmtt_control_param.max_vel_xy, 0.3);
    // 【参数】悬停控制参数 - max_vel_z
    nh.param<float>("rmtt_control_param/max_vel_z", rmtt_control_param.max_vel_z, 0.3);
    // 【参数】悬停控制参数 - max_vel_yaw
    nh.param<float>("rmtt_control_param/max_vel_yaw", rmtt_control_param.max_vel_yaw, 10.0/180.0*M_PI);
    // 【参数】地理围栏参数（超出围栏自动降落）
    nh.param<float>("rmtt_geo_fence/max_x", rmtt_geo_fence.max_x, 3.5);
    nh.param<float>("rmtt_geo_fence/min_x", rmtt_geo_fence.min_x, -3.5);
    nh.param<float>("rmtt_geo_fence/max_y", rmtt_geo_fence.max_y, 4.0);
    nh.param<float>("rmtt_geo_fence/min_y", rmtt_geo_fence.min_y, -4.0);
    nh.param<float>("rmtt_geo_fence/max_z", rmtt_geo_fence.max_z, 2.0);
    nh.param<float>("rmtt_geo_fence/min_z", rmtt_geo_fence.min_z, -0.1);
    // 【参数】led color
    nh.param<float>("led_color/r", led_color.r, 255.0);
    nh.param<float>("led_color/g", led_color.g, 255.0);
    nh.param<float>("led_color/b", led_color.b, 255.0);
    nh.param<float>("led_color/a", led_color.a, 1.0);
    // 【参数】mLED 字符
    nh.param<string>("mled_text", mled_text.data, "yundrone");

    string agent_name = "/rmtt_" + std::to_string(uav_id);
    // 【订阅】订阅rmtt动捕的数据(位置+速度) vrpn -> 本节点
    string target_name = "/tianbot_0" + std::to_string(uav_id);
    mocap_target_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node"+ target_name + "/pose", 1, &RMTT_CONTROL::mocap_target_pos_cb, this);
    // 【订阅】订阅rmtt动捕的数据(位置+速度) vrpn -> 本节点
    mocap_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node"+ agent_name + "/pose", 1, &RMTT_CONTROL::mocap_pos_cb, this);
    mocap_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/vrpn_client_node"+ agent_name + "/twist", 1, &RMTT_CONTROL::mocap_vel_cb, this);
    // 【订阅】地面站指令 地面站 -> 本节点
    station_cmd_sub = nh.subscribe<sunray_msgs::station_cmd>("/sunray_swarm/station_cmd", 1, &RMTT_CONTROL::station_cmd_cb, this);   
    // 【订阅】地面站指令 地面站 -> 本节点(show专用)
    station_cmd_sub2 = nh.subscribe<sunray_msgs::station_cmd>("/sunray_swarm" + agent_name + "/station_cmd", 1, &RMTT_CONTROL::station_cmd_cb2, this);
    // 【订阅】rmtt电池的数据 rmtt_driver -> 本节点
    battery_sub = nh.subscribe<std_msgs::Float32>("/sunray_swarm" + agent_name + "/battery", 1, &RMTT_CONTROL::battery_cb, this);  
    // 【订阅】orca_cmd_vel rmtt_orca -> 本节点
    orca_cmd_sub = nh.subscribe<geometry_msgs::Twist>("/sunray_swarm" + agent_name + "/orca_cmd_vel", 1, &RMTT_CONTROL::orca_cmd_vel_cb, this);  
    // 【发布】无人机orca状态 rmtt_orca -> 本节点
    rmtt_orca_state_sub = nh.subscribe<sunray_msgs::rmtt_orca>("/sunray_swarm" + agent_name + "/rmtt_orca_state", 1, &RMTT_CONTROL::rmtt_orca_state_cb, this);    
    // 【发布】无人机控制指令（机体系，单位：米/秒，Rad/秒）
    rmtt_cmd_pub = nh.advertise<geometry_msgs::Twist>("/sunray_swarm" + agent_name + "/cmd_vel", 1); 
    // 【发布】无人机起飞指令 本节点 -> rmtt_driver
    takeoff_pub = nh.advertise<std_msgs::Empty>("/sunray_swarm" + agent_name + "/takeoff", 1); 
    // 【发布】无人机降落指令 本节点 -> rmtt_driver
    land_pub = nh.advertise<std_msgs::Empty>("/sunray_swarm" + agent_name + "/land", 1); 
    // 【发布】led灯 本节点 -> rmtt_driver
    led_pub = nh.advertise<std_msgs::ColorRGBA>("/sunray_swarm" + agent_name + "/led", 1);
    // 【发布】字符显示 本节点 -> rmtt_driver
    mled_pub = nh.advertise<std_msgs::String>("/sunray_swarm" + agent_name + "/mled", 1);
    // 【发布】rmtt_state 本节点 -> 地面站
    rmtt_state_pub = nh.advertise<sunray_msgs::rmtt_state>("/sunray_swarm" + agent_name + "/rmtt_state", 1); 
    // 【发布】无人机位置(带图标) 本节点 -> RVIZ
    rmtt_mesh_pub = nh.advertise<visualization_msgs::Marker>("/sunray_swarm" + agent_name + "/rmtt_mesh", 1);
    // 【发布】运动轨迹  本节点 -> RVIZ
    rmtt_trajectory_pub = nh.advertise<nav_msgs::Path>("/sunray_swarm" + agent_name + "/rmtt_trajectory", 1);
    // 【发布】文字提示消息  本节点 -> 地面站
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);
    // 【发布】无人机orca状态 本节点 -> 地面站
    rmtt_orca_state_pub = nh.advertise<sunray_msgs::rmtt_orca>("/sunray_swarm" + agent_name + "/rmtt_orca", 1);    
    // 【发布】目标点marker 本节点 -> RVIZ
    goal_point_pub = nh.advertise<visualization_msgs::Marker>("/sunray_swarm" + agent_name + "/goal_point_rviz", 1);

    // 【定时器】 定时发布rmtt_state - 10Hz
    timer_state_pub = nh.createTimer(ros::Duration(0.1), &RMTT_CONTROL::timercb_state, this);
    // 【定时器】 定时发布轨迹 - 10Hz
    timer_trajectory_pub = nh.createTimer(ros::Duration(0.1), &RMTT_CONTROL::timercb_trajectory, this);
    // 【定时器】 定时打印 - 1Hz
    timer_debug = nh.createTimer(ros::Duration(3.0), &RMTT_CONTROL::timercb_debug, this);

    rmtt_state.header.stamp = ros::Time::now();
    rmtt_state.header.frame_id = "world";
    rmtt_state.uav_id = uav_id;
    rmtt_state.uav_ip = uav_ip;  
    rmtt_state.connected = false;
    rmtt_state.odom_valid = false;
    rmtt_state.pos[0] = 0.0;
    rmtt_state.pos[1] = 0.0;
    rmtt_state.pos[2] = 0.0;
    rmtt_state.vel[0] = 0.0;
    rmtt_state.vel[1] = 0.0;
    rmtt_state.vel[2] = 0.0;
    rmtt_state.att[0] = 0.0;
    rmtt_state.att[1] = 0.0;
    rmtt_state.att[2] = 0.0;
    rmtt_state.battery = -1.0;
    mission_state = MISSION_STATE::INIT;
    setup_color();

    // 【函数】打印参数
    printf_param();

    node_name = ros::this_node::getName();
    cout << BLUE << node_name << ": rmtt_" << uav_id << " init! " << TAIL << endl;
    text_info.data = node_name + ": rmtt_" + to_string(uav_id) + " init!";
    text_info_pub.publish(text_info);
}

void RMTT_CONTROL::mainloop()
{
    check_geo_fence();

    // 动捕丢失情况下，不执行控制指令，直到动捕恢复
    if(!rmtt_state.odom_valid)
    {
        cmd_vel_orca_body.linear.x = 0.0;
        cmd_vel_orca_body.linear.y = 0.0;
        cmd_vel_orca_body.linear.z = 0.0;
        cmd_vel_orca_body.angular.x = 0.0;
        cmd_vel_orca_body.angular.y = 0.0;
        cmd_vel_orca_body.angular.z = 0.0;
        rmtt_cmd_pub.publish(cmd_vel_orca_body);
        return;
    }

    switch (mission_state)
    {
    case MISSION_STATE::INIT:
        // 初始化
        if(rmtt_state_last.connected==false && rmtt_state.connected==true)
        {
            setup_led();
            setup_mled();
        }
        break;

    case MISSION_STATE::TAKEOFF:
        // 起飞
        takeoff_pub.publish(takeoff); 
        // 等待飞机起飞，此时不能发送其他指令
        sleep(5.0);
        // 起飞后进入悬停状态，并设定起飞点上方为悬停点
        set_hold_position();
        mission_state = MISSION_STATE::HOLD;
        break;

    case MISSION_STATE::LAND:
        // 降落
        land_pub.publish(land);
        // 等待飞机降落，此时不能发送其他指令
        sleep(5.0); 
        // 降落后进入INIT
        mission_state = MISSION_STATE::INIT;
        break;

    case MISSION_STATE::HOLD:
        // 悬停
        if(track_mode)
        {
            hold_position.x = target_pos.pose.position.x;
            hold_position.y = target_pos.pose.position.y;
            hold_position.z = rmtt_height;
            // desired_yaw = target_yaw;
        }
        hold_control(hold_position);
        break;

    case MISSION_STATE::ORCA_SETUP:
        // 设置后进入悬停状态，并设定当前点为悬停点
        set_hold_position();
        mission_state = MISSION_STATE::HOLD;
        break;

    case MISSION_STATE::ORCA_RUN:
        if(rmtt_orca_state.arrived_goal)
        {
            hold_position.x = rmtt_orca_state.goal[0];
            hold_position.y = rmtt_orca_state.goal[1];
            hold_position.z = rmtt_height;
            hold_control(hold_position);
        }else
        {
            // orca control
            orca_control();
        }
        break; 

    case MISSION_STATE::RETURN_HOME:
        if(rmtt_orca_state.arrived_goal)
        {
            mission_state = MISSION_STATE::LAND;
        }else
        {
            // orca control
            orca_control();
        }
        break; 
    default:
        break;
    }

    rmtt_state_last = rmtt_state;
}

void RMTT_CONTROL::orca_control()
{
    // 惯性系 -> body frame 
    float orca_cmd_body[2];
    float orca_cmd_enu[2];
    orca_cmd_enu[0] = cmd_vel_orca_enu.linear.x;
    orca_cmd_enu[1] = cmd_vel_orca_enu.linear.y;
    rotation_yaw(rmtt_state.att[2], orca_cmd_body, orca_cmd_enu);   
    cmd_vel_orca_body.linear.x = orca_cmd_body[0];
    cmd_vel_orca_body.linear.y = orca_cmd_body[1];
    // 控制指令计算：使用简易P控制 - Z
    cmd_vel_orca_body.linear.z = (rmtt_height - rmtt_state.pos[2]) * rmtt_control_param.Kp_z;
    // 控制指令计算：使用简易P控制 - YAW
    cmd_vel_orca_body.angular.x = 0.0;
    cmd_vel_orca_body.angular.y = 0.0;

    double yaw_error = get_yaw_error(desired_yaw, rmtt_state.att[2]);
    cmd_vel_orca_body.angular.z = yaw_error * rmtt_control_param.Kp_yaw;


    // cmd_vel_orca_body.angular.z = (desired_yaw - rmtt_state.att[2]) * rmtt_control_param.Kp_yaw;

    // 控制指令限幅
    cmd_vel_orca_body.linear.x = constrain_function(cmd_vel_orca_body.linear.x, rmtt_control_param.max_vel_xy, 0.0);
    cmd_vel_orca_body.linear.y = constrain_function(cmd_vel_orca_body.linear.y, rmtt_control_param.max_vel_xy, 0.0);
    cmd_vel_orca_body.linear.z = constrain_function(cmd_vel_orca_body.linear.z, rmtt_control_param.max_vel_z, 0.0);
    cmd_vel_orca_body.angular.z = constrain_function(cmd_vel_orca_body.angular.z, rmtt_control_param.max_vel_yaw, 0.01);

    rmtt_cmd_pub.publish(cmd_vel_orca_body);
}

void RMTT_CONTROL::set_hold_position()
{
    hold_position.x = rmtt_state.pos[0];
    hold_position.y = rmtt_state.pos[1];
    hold_position.z = rmtt_height;
}

double RMTT_CONTROL::get_yaw_error(double desired_yaw, double yaw_now)
{
    double error = desired_yaw - yaw_now;

    if(error > M_PI)
    {
        error = error - 2*M_PI;
    }else if(error < -M_PI)
    {
        error = error + 2*M_PI;
    }

    return error;
}

void RMTT_CONTROL::hold_control(geometry_msgs::Point desired_pos)
{
    // 惯性系 -> body frame
    float hold_cmd_body[2];
    float hold_cmd_enu[2];
    // 控制指令计算：使用简易P控制 - XY
    hold_cmd_enu[0] = (desired_pos.x - rmtt_state.pos[0]) * rmtt_control_param.Kp_xy;
    hold_cmd_enu[1] = (desired_pos.y - rmtt_state.pos[1]) * rmtt_control_param.Kp_xy;
    rotation_yaw(rmtt_state.att[2], hold_cmd_body, hold_cmd_enu);             
    cmd_vel_hold.linear.x = hold_cmd_body[0];
    cmd_vel_hold.linear.y = hold_cmd_body[1];
    // 控制指令计算：使用简易P控制 - Z
    cmd_vel_hold.linear.z = (desired_pos.z - rmtt_state.pos[2]) * rmtt_control_param.Kp_z;
    // 控制指令计算：使用简易P控制 - YAW

    double yaw_error = get_yaw_error(desired_yaw, rmtt_state.att[2]);
    cmd_vel_hold.angular.z = yaw_error * rmtt_control_param.Kp_yaw;

    // cmd_vel_hold.angular.z = (desired_yaw - rmtt_state.att[2]) * rmtt_control_param.Kp_yaw;

    // 控制指令限幅
    cmd_vel_hold.linear.x = constrain_function(cmd_vel_hold.linear.x, rmtt_control_param.max_vel_xy, 0.0);
    cmd_vel_hold.linear.y = constrain_function(cmd_vel_hold.linear.y, rmtt_control_param.max_vel_xy, 0.0);
    cmd_vel_hold.linear.z = constrain_function(cmd_vel_hold.linear.z, rmtt_control_param.max_vel_z, 0.0);
    cmd_vel_hold.angular.z = constrain_function(cmd_vel_hold.angular.z, rmtt_control_param.max_vel_yaw, 0.01);

    // 发布控制指令
    rmtt_cmd_pub.publish(cmd_vel_hold);
}

// 【坐标系旋转函数】- enu系到body系
void RMTT_CONTROL::rotation_yaw(double yaw_angle, float body_frame[2], float enu_frame[2])
{
    body_frame[0] = enu_frame[0] * cos(yaw_angle) + enu_frame[1] * sin(yaw_angle);
    body_frame[1] = -enu_frame[0] * sin(yaw_angle) + enu_frame[1] * cos(yaw_angle);
}

void RMTT_CONTROL::station_cmd_cb2(const sunray_msgs::station_cmd::ConstPtr& msg)
{
    current_station_cmd = *msg; 
    switch(msg->mission_state) 
    {
        case MISSION_STATE::TAKEOFF:
            if(msg->uav_id == uav_id || msg->uav_id == 99)
            {
                mission_state = MISSION_STATE::TAKEOFF;
                text_info.data = node_name + ": rmtt_" + to_string(uav_id) + " Get station_cmd: TAKEOFF!";
                cout << BLUE << text_info.data << TAIL << endl;
            }
            break;
        case MISSION_STATE::LAND:
            if(msg->uav_id == uav_id || msg->uav_id == 99)
            {
                mission_state = MISSION_STATE::LAND;
                text_info.data = node_name + ": rmtt_" + to_string(uav_id) + " Get station_cmd: LAND!";
                cout << BLUE << text_info.data << TAIL << endl;
            }
            break;
        case MISSION_STATE::ORCA_RUN:
            if(msg->uav_id == uav_id || msg->uav_id == 99)
            {
                mission_state = MISSION_STATE::ORCA_RUN;
                text_info.data = node_name + ": rmtt_" + to_string(uav_id) + " Get station_cmd: ORCA_RUN!";
                cout << BLUE << text_info.data << TAIL << endl;
            }
            break;
        case MISSION_STATE::RETURN_HOME:
            if(msg->uav_id == uav_id || msg->uav_id == 99)
            {
                mission_state = MISSION_STATE::RETURN_HOME;
                text_info.data = node_name + ": rmtt_" + to_string(uav_id) + " Get station_cmd: RETURN_HOME!";
                cout << BLUE << text_info.data << TAIL << endl;
                sleep(1.0);
                rmtt_orca_state.arrived_goal = false;
            }
            break;
        default:
            text_info.data = node_name + ": rmtt_" + to_string(uav_id) + " Get station_cmd: Wrong!";
            cout << RED << text_info.data << TAIL << endl;
            break;
    }
    text_info_pub.publish(text_info);
}


void RMTT_CONTROL::station_cmd_cb(const sunray_msgs::station_cmd::ConstPtr& msg)
{
    current_station_cmd = *msg; 
    switch(msg->mission_state) 
    {
        case MISSION_STATE::INIT:
            if(msg->uav_id == uav_id || msg->uav_id == 99)
            {
                mission_state = MISSION_STATE::INIT;
                text_info.data = node_name + ": rmtt_" + to_string(uav_id) + " Get station_cmd: INIT!";
                cout << BLUE << text_info.data << TAIL << endl;
            }
            break;
        case MISSION_STATE::TAKEOFF:
            if(msg->uav_id == uav_id || msg->uav_id == 99)
            {
                mission_state = MISSION_STATE::TAKEOFF;
                text_info.data = node_name + ": rmtt_" + to_string(uav_id) + " Get station_cmd: TAKEOFF!";
                cout << BLUE << text_info.data << TAIL << endl;
            }
            break;
        case MISSION_STATE::LAND:
            if(msg->uav_id == uav_id || msg->uav_id == 99)
            {
                mission_state = MISSION_STATE::LAND;
                text_info.data = node_name + ": rmtt_" + to_string(uav_id) + " Get station_cmd: LAND!";
                cout << BLUE << text_info.data << TAIL << endl;
            }
            break;
        case MISSION_STATE::HOLD:
            if(msg->uav_id == uav_id || msg->uav_id == 99)
            {
                set_hold_position();
                mission_state = MISSION_STATE::HOLD;
                text_info.data = node_name + ": rmtt_" + to_string(uav_id) + " Get station_cmd: HOLD!";
                cout << BLUE << text_info.data << TAIL << endl;
            }
            break;
        case MISSION_STATE::ORCA_SETUP:
            // do nothing;
            break;
        case MISSION_STATE::ORCA_RUN:
            if(msg->uav_id == uav_id || msg->uav_id == 99)
            {
                mission_state = MISSION_STATE::ORCA_RUN;
                text_info.data = node_name + ": rmtt_" + to_string(uav_id) + " Get station_cmd: ORCA_RUN!";
                cout << BLUE << text_info.data << TAIL << endl;
            }
            break;
        case MISSION_STATE::RETURN_HOME:
            if(msg->uav_id == uav_id || msg->uav_id == 99)
            {
                mission_state = MISSION_STATE::RETURN_HOME;
                text_info.data = node_name + ": rmtt_" + to_string(uav_id) + " Get station_cmd: RETURN_HOME!";
                cout << BLUE << text_info.data << TAIL << endl;
                sleep(1.0);
                rmtt_orca_state.arrived_goal = false;
            }
            break;
        default:
            text_info.data = node_name + ": rmtt_" + to_string(uav_id) + " Get station_cmd: Wrong!";
            cout << RED << text_info.data << TAIL << endl;
            break;
    }
    text_info_pub.publish(text_info);
}

void RMTT_CONTROL::rmtt_orca_state_cb(const sunray_msgs::rmtt_orcaConstPtr& msg)
{
    if(mission_state == MISSION_STATE::ORCA_RUN)
    {
        rmtt_orca_state = *msg;
    }else
    {
        rmtt_orca_state = *msg;
    }
    rmtt_orca_state.mission_state = mission_state;
    rmtt_orca_state_pub.publish(rmtt_orca_state);
}

//定时打印
void RMTT_CONTROL::timercb_debug(const ros::TimerEvent &e)
{
    if(!flag_printf)
    {
        return;
    }
    cout << GREEN << ">>>>>>>>>>>>>> RMTT [" << uav_id << "] Control ";
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

    if(rmtt_state.battery < 15.0f)
    {
        cout << RED << "Battery: " << rmtt_state.battery << " [%] <<<<<<<<<<<<<" << TAIL << endl;
    }else if(rmtt_state.battery < 30.0f)
    {
        cout << YELLOW << "Battery: " << rmtt_state.battery << " [%] <<<<<<<<<<<<<" << TAIL << endl;
    }else
    {
        cout << GREEN << "Battery: " << rmtt_state.battery << " [%] <<<<<<<<<<<<<" << TAIL << endl;
    }

    cout << GREEN << "UAV_pos [X Y Z] : " << rmtt_state.pos[0] << " [ m ] " << rmtt_state.pos[1] << " [ m ] " << rmtt_state.pos[2] << " [ m ] " << TAIL << endl;
    // cout << GREEN << "UAV_vel [X Y Z] : " << rmtt_state.vel[0] << " [m/s] " << rmtt_state.vel[1] << " [m/s] " << rmtt_state.vel[2] << " [m/s] " << TAIL << endl;
    cout << GREEN << "UAV_att [R P Y] : " << rmtt_state.att[0] * 180 / M_PI << " [deg] " << rmtt_state.att[1] * 180 / M_PI << " [deg] " << rmtt_state.att[2] * 180 / M_PI << " [deg] " << TAIL << endl;

    // 动捕丢失情况下，不执行控制指令，直到动捕恢复
    if(!rmtt_state.odom_valid)
    {
        cout << RED << "Odom_valid: [Invalid] <<<<<<<<<<<<<" << TAIL << endl;
        return;
    }

    //集群控制命令状态打印
    if (mission_state == MISSION_STATE::INIT)
    {
        cout << GREEN << "MISSION_STATE : [ Ready ]" << TAIL << endl;
    }
    else if (mission_state == MISSION_STATE::TAKEOFF)
    {
        cout << GREEN << "MISSION_STATE : [ TAKEOFF ]" << TAIL << endl;
    }
    else if (mission_state == MISSION_STATE::LAND)
    {
        cout << GREEN << "MISSION_STATE : [ LAND ]" << TAIL << endl;
    }
    else if (mission_state == MISSION_STATE::HOLD)
    {
        cout << GREEN << "MISSION_STATE : [ HOLD ]" << TAIL << endl;
        cout << GREEN << "Hold [X Y Z]  : " << hold_position.x   << " [ m ] " << hold_position.y   << " [ m ] " << hold_position.z   << " [ m ] " << TAIL << endl;
        cout << GREEN << "CMD  [X Y Z]  : " << cmd_vel_hold.linear.x << " [m/s] " << cmd_vel_hold.linear.y << " [m/s] " << cmd_vel_hold.linear.z << " [m/s] " << TAIL << endl;
        cout << GREEN << "CMD  [ Yaw ]  : " << cmd_vel_hold.angular.z * 180 / M_PI<< " [deg/s] " << TAIL << endl;
    }
    else if (mission_state == MISSION_STATE::ORCA_SETUP)
    {
        cout << GREEN << "MISSION_STATE : [ ORCA_SETUP ]" << TAIL << endl;
    }
    else if (mission_state == MISSION_STATE::ORCA_RUN)
    {
        cout << GREEN << "MISSION_STATE : [ ORCA_RUN ]  " ;
        if(rmtt_orca_state.arrived_goal)
        {
            cout << GREEN << "Arrived_goal : [ true ]" << TAIL << endl;
        }else
        {
            cout << RED << "Arrived_goal : [ false ]" << TAIL << endl;
        }
        if(rmtt_orca_state.arrived_goal)
        {
            cout << GREEN << "GOAL     [X Y Z]  : " << rmtt_orca_state.goal[0]      << " [ m ] " << rmtt_orca_state.goal[1]     << " [ m ] " << TAIL << endl;
            cout << GREEN << "CMD_PUB  [X Y Z]  : " << cmd_vel_hold.linear.x   << " [m/s] " << cmd_vel_hold.linear.y  << " [m/s] " << cmd_vel_hold.linear.z << " [m/s] " << TAIL << endl;
        }else
        {
            cout << GREEN << "GOAL     [X Y Z]  : " << rmtt_orca_state.goal[0]      << " [ m ] " << rmtt_orca_state.goal[1]     << " [ m ] " << TAIL << endl;
            cout << GREEN << "CMD_ORCA [X Y Z]  : " << rmtt_orca_state.vel_orca[0]  << " [m/s] " << rmtt_orca_state.vel_orca[1] << " [m/s] " << TAIL << endl;
            cout << GREEN << "CMD_PUB  [X Y Z]  : " << cmd_vel_orca_body.linear.x   << " [m/s] " << cmd_vel_orca_body.linear.y  << " [m/s] " << cmd_vel_orca_body.linear.z << " [m/s] " << TAIL << endl;
            // cout << GREEN << "CMD_PUB  [ Yaw ]  : " << cmd_vel_orca_body.angular.z * 180 / M_PI << " [deg/s] " << TAIL << endl;
        }
    }
    else if (mission_state == MISSION_STATE::RETURN_HOME)
    {
        cout << GREEN << "MISSION_STATE : [ RETURN_HOME ]  " ;
        if(rmtt_orca_state.arrived_goal)
        {
            cout << GREEN << "Arrived_goal : [ true ]" << TAIL << endl;
        }else
        {
            cout << RED << "Arrived_goal : [ false ]" << TAIL << endl;
        }
        if(rmtt_orca_state.arrived_goal)
        {
            cout << GREEN << "GOAL     [X Y Z]  : " << rmtt_orca_state.goal[0]      << " [ m ] " << rmtt_orca_state.goal[1]     << " [ m ] " << TAIL << endl;
            cout << GREEN << "CMD_PUB  [X Y Z]  : " << cmd_vel_hold.linear.x   << " [m/s] " << cmd_vel_hold.linear.y  << " [m/s] " << cmd_vel_hold.linear.z << " [m/s] " << TAIL << endl;
        }else
        {
            cout << GREEN << "GOAL     [X Y Z]  : " << rmtt_orca_state.goal[0]      << " [ m ] " << rmtt_orca_state.goal[1]     << " [ m ] " << TAIL << endl;
            cout << GREEN << "CMD_ORCA [X Y Z]  : " << rmtt_orca_state.vel_orca[0]  << " [m/s] " << rmtt_orca_state.vel_orca[1] << " [m/s] " << TAIL << endl;
            cout << GREEN << "CMD_PUB  [X Y Z]  : " << cmd_vel_orca_body.linear.x   << " [m/s] " << cmd_vel_orca_body.linear.y  << " [m/s] " << cmd_vel_orca_body.linear.z << " [m/s] " << TAIL << endl;
        }
    }
    else
    {
        cout << RED << "MISSION_STATE : [ ERROR ]" << TAIL << endl;
    }
}

void RMTT_CONTROL::orca_cmd_vel_cb(const geometry_msgs::TwistConstPtr& msg)
{
    if(mission_state == MISSION_STATE::ORCA_RUN || mission_state == MISSION_STATE::RETURN_HOME)
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

//定时发布rmtt_state
void RMTT_CONTROL::timercb_state(const ros::TimerEvent &e)
{
    // 发布 rmtt_state
    rmtt_state.header.stamp = ros::Time::now();

    if((ros::Time::now() - get_mocap_time).toSec() > MOCAP_TIMEOUT)
    {
        rmtt_state.odom_valid = false;
    }

    rmtt_state_pub.publish(rmtt_state);
}

void RMTT_CONTROL::mocap_target_pos_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
    target_pos = *msg;

    Eigen::Quaterniond q_mocap = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Vector3d target_att = quaternion_to_euler(q_mocap);
    target_yaw = target_att.z();
}

void RMTT_CONTROL::mocap_pos_cb(const geometry_msgs::PoseStampedConstPtr& msg)
{
    get_mocap_time = ros::Time::now(); // 记录时间戳，防止超时
	rmtt_state.pos[0] = msg->pose.position.x;
    rmtt_state.pos[1] = msg->pose.position.y;
	rmtt_state.pos[2] = msg->pose.position.z;
    rmtt_state.attitude_q = msg->pose.orientation;

    Eigen::Quaterniond q_mocap = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Vector3d agent_att = quaternion_to_euler(q_mocap);

	rmtt_state.att[0] = agent_att.x();
    rmtt_state.att[1] = agent_att.y();
	rmtt_state.att[2] = agent_att.z();

    rmtt_state.odom_valid = true;
}

void RMTT_CONTROL::mocap_vel_cb(const geometry_msgs::TwistStampedConstPtr& msg)
{
	rmtt_state.vel[0] = msg->twist.linear.x;
    rmtt_state.vel[1] = msg->twist.linear.y;
	rmtt_state.vel[2] = msg->twist.linear.z;
}

void RMTT_CONTROL::battery_cb(const std_msgs::Float32ConstPtr& msg)
{
    rmtt_state.connected = true;
    rmtt_state.battery = msg->data;
}

void RMTT_CONTROL::setup_color()
{
    led_color.a = 1.0;
    switch(uav_id) // 控制仿真时RVIZ中无人机的颜色，与真机无关
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

void RMTT_CONTROL::setup_led()
{
    std_msgs::ColorRGBA rmtt_led;
    rmtt_led.r = int(led_color.r * 255);
    rmtt_led.g = int(led_color.g * 255);
    rmtt_led.b = int(led_color.b * 255);
    rmtt_led.a = led_color.a;
    led_pub.publish(rmtt_led);
}

void RMTT_CONTROL::setup_mled()
{
    mled_pub.publish(mled_text);
}

void RMTT_CONTROL::timercb_trajectory(const ros::TimerEvent &e)
{
    // 发布无人机marker
    visualization_msgs::Marker rmtt_marker;
    rmtt_marker.header.frame_id = "world";
    rmtt_marker.header.stamp = ros::Time::now();
    rmtt_marker.ns = "mesh";
    rmtt_marker.id = 0;
    rmtt_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    rmtt_marker.scale.x = 0.4;  
    rmtt_marker.scale.y = 0.4;  
    rmtt_marker.scale.y = 0.4;  
    rmtt_marker.action = visualization_msgs::Marker::ADD;
    rmtt_marker.pose.position.x = rmtt_state.pos[0];
    rmtt_marker.pose.position.y = rmtt_state.pos[1];
    rmtt_marker.pose.position.z = rmtt_state.pos[2];
    rmtt_marker.pose.orientation.w = rmtt_state.attitude_q.w;
    rmtt_marker.pose.orientation.x = rmtt_state.attitude_q.x;
    rmtt_marker.pose.orientation.y = rmtt_state.attitude_q.y;
    rmtt_marker.pose.orientation.z = rmtt_state.attitude_q.z;
    rmtt_marker.color = led_color;
    rmtt_marker.mesh_use_embedded_materials = false;
    rmtt_marker.mesh_resource = std::string("package://sunray_swarm/meshes/hummingbird.mesh");
    rmtt_mesh_pub.publish(rmtt_marker);

    // 发布无人机运动轨迹，用于rviz显示
    geometry_msgs::PoseStamped uav_pos;
    uav_pos.header.stamp = ros::Time::now();
    uav_pos.header.frame_id = "world";
    uav_pos.pose.position.x = rmtt_state.pos[0];
    uav_pos.pose.position.y = rmtt_state.pos[1];
    uav_pos.pose.position.z = rmtt_state.pos[2];
    uav_pos.pose.orientation = rmtt_state.attitude_q;
    pos_vector.insert(pos_vector.begin(), uav_pos);
    if (pos_vector.size() > TRA_WINDOW)
    {
        pos_vector.pop_back();
    }
    nav_msgs::Path uav_trajectory;
    uav_trajectory.header.stamp = ros::Time::now();
    uav_trajectory.header.frame_id = "world";
    uav_trajectory.poses = pos_vector;
    rmtt_trajectory_pub.publish(uav_trajectory);

    // 发布目标点mesh
    visualization_msgs::Marker goal_marker;
    goal_marker.header.frame_id = "world";
    goal_marker.header.stamp = ros::Time::now();
    goal_marker.ns = "rmtt_goal";
    goal_marker.id = uav_id;
    goal_marker.type = visualization_msgs::Marker::SPHERE;
    goal_marker.action = visualization_msgs::Marker::ADD;
    goal_marker.pose.position.x = rmtt_orca_state.goal[0];
    goal_marker.pose.position.y = rmtt_orca_state.goal[1];
    goal_marker.pose.position.z = rmtt_height;
    goal_marker.pose.orientation.x = 0.0;
    goal_marker.pose.orientation.y = 0.0;
    goal_marker.pose.orientation.z = 0.0;
    goal_marker.pose.orientation.w = 1.0;
    goal_marker.scale.x = 0.1;
    goal_marker.scale.y = 0.1;
    goal_marker.scale.z = 0.1;
    goal_marker.color = led_color;
    rmtt_marker.mesh_use_embedded_materials = false;
    goal_point_pub.publish(goal_marker);
}

float RMTT_CONTROL::constrain_function(float data, float Max, float Min)
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

void RMTT_CONTROL::check_geo_fence()
{
    // 安全检查，超出地理围栏自动降落,打印相关位置信息
    if (rmtt_state.pos[0] > rmtt_geo_fence.max_x || rmtt_state.pos[0] < rmtt_geo_fence.min_x || 
        rmtt_state.pos[1] > rmtt_geo_fence.max_y || rmtt_state.pos[1] < rmtt_geo_fence.min_y || 
        rmtt_state.pos[2] > rmtt_geo_fence.max_z || rmtt_state.pos[2] < rmtt_geo_fence.min_z)
    {
        mission_state = MISSION_STATE::LAND;
        ROS_WARN_STREAM("RMTT [" << uav_id << "] out of geofence land! Position: [" 
                        << rmtt_state.pos[0] << ", " << rmtt_state.pos[1] << ", " 
                        << rmtt_state.pos[2] << "], Geofence: ["
                        << rmtt_geo_fence.min_x << ", " << rmtt_geo_fence.max_x << ", "
                        << rmtt_geo_fence.min_y << ", " << rmtt_geo_fence.max_y << ", "
                        << rmtt_geo_fence.min_z << ", " << rmtt_geo_fence.max_z << "]");
    }
}

Eigen::Vector3d RMTT_CONTROL::quaternion_to_euler(const Eigen::Quaterniond &q)
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

void RMTT_CONTROL::printf_param()
{
    cout << GREEN << ">>>>>>>>>>>>>>>>>>> RMTT_CONTROL Parameters <<<<<<<<<<<<<<<<" << TAIL << endl;

    // 无人机数量和飞行高度
    cout << GREEN << "uav_id : " << uav_id << "" << TAIL << endl;
    cout << GREEN << "uav_ip : " << uav_ip << "" << TAIL << endl;
    cout << GREEN << "sim_mode : " << sim_mode << "" << TAIL << endl;
    cout << GREEN << "flag_printf : " << flag_printf << "" << TAIL << endl;
    cout << GREEN << "rmtt_height : " << rmtt_height << " [m]" << TAIL << endl;

    // 悬停控制参数
    cout << GREEN << "Kp_xy : " << rmtt_control_param.Kp_xy << TAIL << endl;
    cout << GREEN << "Kp_z : " << rmtt_control_param.Kp_z << TAIL << endl;
    cout << GREEN << "Kp_yaw : " << rmtt_control_param.Kp_yaw << TAIL << endl;
    cout << GREEN << "max_vel_xy : " << rmtt_control_param.max_vel_xy << " [m/s]" << TAIL << endl;
    cout << GREEN << "max_vel_z : " << rmtt_control_param.max_vel_z << " [m/s]" << TAIL << endl;
    cout << GREEN << "max_vel_yaw : " << rmtt_control_param.max_vel_yaw << " [rad/s]" << TAIL << endl;

    // 地理围栏参数
    cout << GREEN << "geo_fence max_x : " << rmtt_geo_fence.max_x << " [m]" << TAIL << endl;
    cout << GREEN << "geo_fence min_x : " << rmtt_geo_fence.min_x << " [m]" << TAIL << endl;
    cout << GREEN << "geo_fence max_y : " << rmtt_geo_fence.max_y << " [m]" << TAIL << endl;
    cout << GREEN << "geo_fence min_y : " << rmtt_geo_fence.min_y << " [m]" << TAIL << endl;
    cout << GREEN << "geo_fence max_z : " << rmtt_geo_fence.max_z << " [m]" << TAIL << endl;
    cout << GREEN << "geo_fence min_z : " << rmtt_geo_fence.min_z << " [m]" << TAIL << endl;
}