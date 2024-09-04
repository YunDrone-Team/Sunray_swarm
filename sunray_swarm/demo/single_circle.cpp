#include <ros/ros.h>
#include <signal.h>
#include <cmath>
#include "sunray_msgs/agent_cmd.h"
#include "printf_utils.h"
#include "math_utils.h"
using namespace std;

ros::Publisher cmd_pub; // 发布控制命令
float desired_yaw; //期望的偏航角
float circle_radius; //圆周轨迹的半径
float linear_vel; //线速度
float time_trajectory = 0.0; //轨迹时间计数器
float trajectory_total_time; //轨迹总时间
float omega; //角速度

int start_cmd = 0; //开始命令标志
int agent_type; // 代理类型，用于区分无人机和无人车
float agent_height;//设置无人机高度
int agent_num;      //设置无人机数量

// 处理信号
void mySigintHandler(int sig) {
    ROS_INFO("[rmtt_circle_trajectory] exit...");
    ros::shutdown();
}

// 初始化参数
void initParams(ros::NodeHandle& nh) {
    // 从参数服务器获取期望的偏航角，默认为0.0
    nh.param<float>("desired_yaw", desired_yaw, 0.0f); 
    // 从参数服务器获取圆周轨迹的半径，默认为1.0
    nh.param<float>("circle_radius", circle_radius, 1.0f); 
    // 从参数服务器获取线速度，默认为0.3
    nh.param<float>("linear_vel", linear_vel, 0.3f); 
    // 默认为1 (RMTT)
    nh.param<int>("agent_type", agent_type, 1); 
    //从参数服务器获取高度,默认为1
    nh.param<float>("agent_height", agent_height, 1.0f); 
    // 默认为数量为1 
    nh.param<int>("agent_num", agent_num, 1); 

    if (circle_radius != 0) {
        // 匀速圆周运动的角速度
        omega = linear_vel / circle_radius;  
    } else {
        // 如果半径为0，则角速度为0
        omega = 0.0;
    }
}

// 发布到初始位置的命令
void publishInitialPosition() {
    sunray_msgs::agent_cmd cmd;
    // ID 默认为1
    cmd.agent_id = 1; 
    // 设置控制状态为位置控制模式
    cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
    // 设置起始位置的x坐标为圆周轨迹的半径
    cmd.desired_pos.x = circle_radius; 
    // 设置起始位置的y坐标为0
    cmd.desired_pos.y = 0.0;
    // 根据代理类型设置高度
    cmd.desired_pos.z = agent_type == sunray_msgs::agent_state::RMTT || agent_type == sunray_msgs::agent_state::SIKONG ? agent_height : 0.0; // 高度设定
    // 设置起始位置的偏航角
    cmd.desired_yaw = desired_yaw;
    // 发布初始位置命令
    cmd_pub.publish(cmd);
    cout << GREEN << "Moving to start position: x=" << circle_radius << " y=0" << " z=" << cmd.desired_pos.z << " yaw=" << desired_yaw << TAIL << endl;
}

// 发布圆形轨迹的控制命令
void publishCircleCommand() {
    // 计算当前的角度
    float angle = time_trajectory * omega;
    sunray_msgs::agent_cmd cmd;
    // 设置代理的ID，默认为1
    cmd.agent_id = 1;  
    // 设置控制状态为位置控制模式
    cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
    // 计算并设置当前位置的x坐标
    cmd.desired_pos.x = circle_radius * cos(angle);
    // 计算并设置当前位置的y坐标
    cmd.desired_pos.y = circle_radius * sin(angle);
     // 根据代理类型设置高度
    cmd.desired_pos.z = agent_type == sunray_msgs::agent_state::RMTT || agent_type == sunray_msgs::agent_state::SIKONG ? agent_height : 0.0; 
    // 设置起始位置的偏航角
    cmd.desired_yaw = desired_yaw;
    // 发布初始位置命令
    cmd_pub.publish(cmd);
    cout << BLUE << "Moving in circle: x=" << cmd.desired_pos.x << " y=" << cmd.desired_pos.y << " z=" << cmd.desired_pos.z << " yaw=" << desired_yaw << TAIL << endl;
}

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "agent_circle_control");
    // 创建节点句柄
    ros::NodeHandle nh;
    // 设置信号处理函数
    signal(SIGINT, mySigintHandler);
    // 初始化参数
    initParams(nh);

    // 定义一个字符串变量，用于存储代理前缀
    string agent_prefix;
    switch(agent_type) {
        case sunray_msgs::agent_state::RMTT:
            agent_prefix = "rmtt_";
            break;
        case sunray_msgs::agent_state::TIANBOT:
            agent_prefix = "tianbot_";
            break;
        case sunray_msgs::agent_state::WHEELTEC:
            agent_prefix = "wheeltec_";
            break;
        case sunray_msgs::agent_state::SIKONG:
            agent_prefix = "sikong_";
            break;
        default:
            agent_prefix = "unknown_";
            break;
    }
    string agent_name;
    // 初始化发布者
    for (int i = 0; i < agent_num; i++)
    {
        agent_name = "/" + std::to_string(i+1);
        cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 10);
    }
    // cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm/" + agent_prefix + "1/agent_cmd", 10);
    

    // cout << BLUE << "Params -> Yaw: " << desired_yaw << ", Radius: " << circle_radius << ", Linear Velocity: " << linear_vel << ", Agent Prefix: " << agent_prefix << TAIL << endl;


    cout << GREEN << "Enter 1 to move to start position..." << TAIL << endl;
    
    // 获取用户输入
    cin >> start_cmd;
    if (start_cmd == 1) {
        // 发布初始位置命令
        publishInitialPosition();
        // 等待2秒以确保移动完成
        ros::Duration(2.0).sleep();
    }
    // 提示用户输入总轨迹时间
    cout << GREEN << "Enter total trajectory time (seconds): " << TAIL;
    // 获取用户输入的轨迹总时间
    cin >> trajectory_total_time;
    // 主循环，在轨迹时间内发布圆形轨迹的控制命令
    while (ros::ok() && time_trajectory < trajectory_total_time) {
        // 发布圆形轨迹控制命令
        publishCircleCommand();
        // 增加时间计数器
        time_trajectory += 0.1;
        // 处理回调函数
        ros::spinOnce();
        // 休眠0.1秒
        ros::Duration(0.1).sleep();
    }

    return 0;
}