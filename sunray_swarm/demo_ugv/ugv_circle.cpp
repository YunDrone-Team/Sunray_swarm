/***********************************************************************************
 *  文件名: ugv_circle.cpp                                                          
 *  作者: Yun Drone                                                                 
 *  描述: 无人车demo：圆形轨迹移动
 *     1、从参数列表里面获取圆形轨迹参数
 *     2、等待demo启动指令
 *     3、启动后根据时间计算圆形轨迹位置并发送到控制节点执行（POS_CONTROL模式）
 *     4、可通过demo_start_flag暂停或恢复圆形轨迹
 ***********************************************************************************/

#include <ros/ros.h>
#include "printf_utils.h"
#include "ros_msg_utils.h"

using namespace std;

int agent_id;                          // 智能体编号
float circle_radius;                   // 圆参数：圆周轨迹的半径
float linear_vel;                      // 圆参数：线速度
float omega;                           // 圆参数：角速度
float time_trajectory = 0.0;           // 圆参数：轨迹时间计数器
bool demo_start_flag = false;          // 是否接收到开始命令
sunray_msgs::agent_cmd agent_cmd;      // 智能体控制指令
float desired_yaw;                     // 期望的偏航角
std_msgs::String text_info;            // 打印消息

ros::Publisher agent_cmd_pub;          // 发布控制命令
ros::Subscriber demo_start_flag_sub;   // 订阅开始命令
ros::Publisher text_info_pub;          // 发布信息到地面站

void mySigintHandler(int sig)
{
    ROS_INFO("[ugv_circle] exit...");
    ros::shutdown();
}

void demo_start_flag_cb(const std_msgs::Bool::ConstPtr &msg)
{
    demo_start_flag = msg->data;  

    if(demo_start_flag)
    {
        text_info.data = "Get demo start cmd";
        cout << GREEN << text_info.data << TAIL << endl;
        text_info_pub.publish(text_info);
    }else
    {
        text_info.data = "Get demo stop cmd";
        cout << GREEN << text_info.data << TAIL << endl;
        text_info_pub.publish(text_info);
    }
}

// 主函数
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "ugv_circle");
    // 创建节点句柄
    ros::NodeHandle nh("~");
    // 设置节点的执行频率为10Hz
    ros::Rate rate(10);
    
    // 【参数】智能体编号
    nh.param<int>("agent_id", agent_id, 1);
    //【参数】从参数服务器获取圆周轨迹的半径，默认为1.0米
    nh.param<float>("circle_radius", circle_radius, 1.0f);
    //【参数】从参数服务器获取线速度，默认为0.3米/秒
    nh.param<float>("linear_vel", linear_vel, 0.3f);
    //【参数】从参数服务器获取期望的偏航角，默认为0.0
    nh.param<float>("desired_yaw", desired_yaw, 0.0f);

    cout << GREEN << ros::this_node::getName() << " start." << TAIL << endl;

    cout << GREEN << "agent_id      : " << agent_id << TAIL << endl;
    cout << GREEN << "circle_radius : " << circle_radius << TAIL << endl;
    cout << GREEN << "linear_vel    : " << linear_vel << TAIL << endl;
    cout << GREEN << "desired_yaw   : " << desired_yaw << TAIL << endl;


    string agent_name = "/ugv_" + std::to_string(agent_id);
    // 【订阅】触发指令 外部 -> 本节点 
    demo_start_flag_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/demo/ugv_circle", 1, demo_start_flag_cb);
    // 【发布】控制指令 本节点 -> 无人车控制节点
    agent_cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 10);
    // 【发布】文字提示消息  本节点 -> 地面站
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);

    // 重置时间计数器，准备执行圆周运动
    time_trajectory = 0.0;  
    // 主循环
    while (ros::ok())
    {   
        // 等待demo启动
        if(!demo_start_flag)
        {
            // 处理一次回调函数
            ros::spinOnce();
            // sleep
            rate.sleep();
            continue;
        }

        // 执行圆周运动，当demo_start_flag为false时退出
        while(ros::ok() && demo_start_flag)
        {
            // 根据设定的圆参数计算每一时刻的期望位置
            omega = linear_vel / circle_radius;
            float angle = time_trajectory * omega;
            agent_cmd.header.stamp = ros::Time::now();
            agent_cmd.header.frame_id = "world";
            agent_cmd.agent_id = agent_id;
            agent_cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
            agent_cmd.desired_pos.x = circle_radius * cos(angle);  // 计算当前x坐标
            agent_cmd.desired_pos.y = circle_radius * sin(angle);  // 计算当前y坐标
            agent_cmd.desired_pos.z = 0.1;
            agent_cmd.desired_yaw = desired_yaw;  
            agent_cmd_pub.publish(agent_cmd);
            // 更新时间计数器，由于循环频率为10Hz，因此设置为0.1秒
            time_trajectory += 0.1;
            ros::spinOnce();
            rate.sleep();
        }
    }

    return 0;
}