/***********************************************************************************
 *  文件名: rmtt_hover.cpp                                                          
 *  作者: Yun Drone                                                                 
 *  描述: 无人机demo：悬停
 *     1、起飞，等待
 *     2、移动到指定点，等待
 *     3、悬停，等待
 *     4、降落
 ***********************************************************************************/

#include <ros/ros.h>
#include "ros_msg_utils.h"
#include "printf_utils.h"

int agent_id;                            // 智能体ID
sunray_msgs::agent_cmd agent_cmd;        // 智能体控制指令
std_msgs::String text_info;              // 打印消息
string node_name;              // 节点名称

ros::Publisher agent_cmd_pub;            // 发布控制命令
ros::Publisher text_info_pub;            // 发布文字提示消息

// 主函数
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "rmtt_hover");
    // 创建节点句柄
    ros::NodeHandle nh("~");
    // 设置循环频率为10Hz
    ros::Rate rate(10);

    // 【参数】智能体编号
    nh.param<int>("agent_id", agent_id, 1);

    string agent_name = "/rmtt_" + std::to_string(agent_id); 
    // 【发布】控制指令 本节点 -> rmtt控制节点
    agent_cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 10);
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
    agent_cmd.cmd_source = "rmtt_hover";
    agent_cmd.control_state = sunray_msgs::agent_cmd::TAKEOFF;
    agent_cmd_pub.publish(agent_cmd); 

    text_info.data = node_name + "Takeoff...";
    cout << GREEN << text_info.data << TAIL << endl;
    text_info_pub.publish(text_info);

    sleep(10.0);

    // 发送移动到悬停点的指令
    agent_cmd.header.stamp = ros::Time::now();
    agent_cmd.header.frame_id = "world";
    agent_cmd.agent_id = agent_id;
    agent_cmd.cmd_source = "rmtt_hover";
    agent_cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
    // 设置目标位置为悬停点
    agent_cmd.desired_pos.x = 0.0;
    agent_cmd.desired_pos.y = 0.0;
    agent_cmd.desired_pos.z = 1.0;
    // 设置悬停点偏航角
    agent_cmd.desired_yaw = 0.0;
    agent_cmd_pub.publish(agent_cmd); 

    text_info.data = node_name + "Move to hover position...";
    cout << GREEN << text_info.data << TAIL << endl;
    text_info_pub.publish(text_info);

    sleep(5.0);

    // 发送悬停指令
    agent_cmd.header.stamp = ros::Time::now();
    agent_cmd.header.frame_id = "world";
    agent_cmd.agent_id = agent_id;
    agent_cmd.cmd_source = "rmtt_hover";
    agent_cmd.control_state = sunray_msgs::agent_cmd::HOLD;
    agent_cmd_pub.publish(agent_cmd); 

    text_info.data = node_name + "Hover at current position...";
    cout << GREEN << text_info.data << TAIL << endl;
    text_info_pub.publish(text_info);

    sleep(10.0);

    // 发送降落指令
    agent_cmd.header.stamp = ros::Time::now();
    agent_cmd.header.frame_id = "world";
    agent_cmd.agent_id = agent_id;
    agent_cmd.cmd_source = "rmtt_hover";
    agent_cmd.control_state = sunray_msgs::agent_cmd::LAND;
    agent_cmd_pub.publish(agent_cmd); 

    text_info.data = node_name + "Land...";
    cout << GREEN << text_info.data << TAIL << endl;
    text_info_pub.publish(text_info);

    sleep(5.0);

    text_info.data = node_name + "Demo finished...";
    cout << GREEN << text_info.data << TAIL << endl;
    text_info_pub.publish(text_info);
    return 0;
}