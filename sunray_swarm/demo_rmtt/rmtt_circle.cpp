/***********************************************************************************
 *  文件名: rmtt_circle.cpp                                                          
 *  作者: Yun Drone                                                                 
 *  描述: 无人机demo：圆形轨迹移动
 *     1、从参数列表里面获取圆形轨迹参数
 *     2、起飞
 *     3、等待demo启动指令
 *     4、启动后根据时间计算圆形轨迹位置并发送到控制节点执行（POS_CONTROL模式）
 *     5、本程序没有设置自动降落，可以停止圆形轨迹移动后，通过地面站降落
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
sunray_swarm_msgs::agent_cmd agent_cmd;      // 智能体控制指令
float desired_yaw;                     // 期望的偏航角
std_msgs::String text_info;            // 打印消息
string node_name;                      // 节点名称

ros::Publisher agent_cmd_pub;          // 发布控制命令
ros::Publisher text_info_pub;          // 发布信息到地面站

std_msgs::Empty takeoff;
std_msgs::Empty land;

void mySigintHandler(int sig)
{
    ROS_INFO("[rmtt_circle] exit...");
    ros::shutdown();
}

// 主函数
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "rmtt_circle");
    // 创建节点句柄
    ros::NodeHandle nh("~");
    // 设置节点的执行频率为10Hz
    ros::Rate rate(10);

    //【参数】智能体编号
    nh.param<int>("agent_id", agent_id, 1);
    //【参数】圆形轨迹参数：圆周轨迹的半径，默认为1.0米
    nh.param<float>("circle_radius", circle_radius, 1.0f);
    //【参数】圆形轨迹参数：线速度，默认为0.3米/秒
    nh.param<float>("linear_vel", linear_vel, 0.3f);
    //【参数】期望的偏航角，默认为0.0
    nh.param<float>("desired_yaw", desired_yaw, 0.0f);

    cout << GREEN << "agent_id      : " << agent_id << TAIL << endl;
    cout << GREEN << "circle_radius : " << circle_radius << TAIL << endl;
    cout << GREEN << "linear_vel    : " << linear_vel << TAIL << endl;
    cout << GREEN << "desired_yaw   : " << desired_yaw << TAIL << endl;

    string agent_name = "/rmtt_" + std::to_string(agent_id);
    // 【发布】控制指令 本节点 -> 无人机控制节点
    agent_cmd_pub = nh.advertise<sunray_swarm_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 10);
    // 【发布】文字提示消息  本节点 -> 地面站
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);

    sleep(1.0);
    node_name = "[" + ros::this_node::getName() + "] ---> ";

    text_info.data = node_name + "Demo init...";
    cout << GREEN << text_info.data << TAIL << endl;
    text_info_pub.publish(text_info);

    sleep(5.0);
    // 发送起飞指令
    agent_cmd.header.stamp = ros::Time::now();
    agent_cmd.header.frame_id = "world";
    agent_cmd.agent_id = agent_id;
    agent_cmd.cmd_source = ros::this_node::getName();
    agent_cmd.control_state = sunray_swarm_msgs::agent_cmd::TAKEOFF;
    agent_cmd_pub.publish(agent_cmd); 

    text_info.data = node_name + "Takeoff...";
    cout << GREEN << text_info.data << TAIL << endl;
    text_info_pub.publish(text_info);
    
    sleep(10.0);

    // 重置时间计数器，准备执行圆周运动
    time_trajectory = 0.0;  
    // 主循环
    while (ros::ok())
    {   
        // 执行圆周运动
        while(ros::ok())
        {
            // 根据设定的圆参数计算每一时刻的期望位置
            omega = linear_vel / circle_radius;
            float angle = time_trajectory * omega;
            agent_cmd.header.stamp = ros::Time::now();
            agent_cmd.header.frame_id = "world";
            agent_cmd.agent_id = agent_id;
            agent_cmd.cmd_source = ros::this_node::getName();
            agent_cmd.control_state = sunray_swarm_msgs::agent_cmd::POS_CONTROL;
            agent_cmd.desired_pos.x = circle_radius * cos(angle);  // 计算当前x坐标
            agent_cmd.desired_pos.y = circle_radius * sin(angle);  // 计算当前y坐标
            agent_cmd.desired_pos.z = 1.0;
            // 偏航角跟随圆形轨迹计算
            double vx,vy;
            vx = -omega * circle_radius * sin(angle);
            vy = omega * circle_radius * cos(angle);
            agent_cmd.desired_yaw = atan2(vy, vx);
            agent_cmd_pub.publish(agent_cmd);
            // 更新时间计数器，由于循环频率为10Hz，因此设置为0.1秒
            time_trajectory += 0.1;
            ros::spinOnce();
            rate.sleep();
        }
    }

    text_info.data = node_name + "Demo finished...";
    cout << GREEN << text_info.data << TAIL << endl;
    text_info_pub.publish(text_info);

    return 0;
}