/***********************************************************************************
 *  文件名: ugv_waypoint.cpp                                                          
 *  作者: Yun Drone                                                                 
 *  描述: 无人车demo：路径点移动
 *     1、从参数列表里面获取目标点数目及目标点位置
 *     2、等待demo启动指令
 *     3、启动后逐个发送目标点至智能体控制节点执行（POS_CONTROL模式）
 *     4、执行完后，重置状态位
 ***********************************************************************************/

#include <ros/ros.h>
#include "printf_utils.h"
#include "ros_msg_utils.h"

using namespace std;

int agent_id;                            // 智能体ID
int waypoint_count;                      // 目标点数量
vector<geometry_msgs::Point> waypoints;  // 目标点列表
bool demo_start_flag = false;            // 标记是否接收到开始命令
sunray_msgs::agent_cmd agent_cmd;        // 智能体控制指令
std_msgs::String text_info;              // 打印消息

ros::Publisher agent_cmd_pub;            // 发布控制命令
ros::Publisher text_info_pub;            // 发布文字提示消息
ros::Subscriber demo_start_flag_sub;     // demo启动订阅

// 触发信号的回调函数，处理接收到的位置
void demo_start_flag_cb(const std_msgs::Bool::ConstPtr &msg) 
{
    demo_start_flag = msg->data; 

    if(demo_start_flag)
    {
        text_info.data = "Get demo start cmd";
        cout << GREEN << text_info.data << TAIL << endl;
        text_info_pub.publish(text_info);
    }
}

// 主函数
int main(int argc, char **argv) 
{
    // 初始化ROS节点
    ros::init(argc, argv, "ugv_waypoint");
    // 创建节点句柄
    ros::NodeHandle nh("~");
    // 设置循环频率为10Hz
    ros::Rate rate(10);

    cout << GREEN << ros::this_node::getName() << " start." << TAIL << endl;

    // 【参数】智能体编号
    nh.param<int>("agent_id", agent_id, 1);
    // 【参数】目标点数量，默认为1
    nh.param("waypoint_count", waypoint_count, 1); 
    waypoints.resize(waypoint_count);
    // 【参数】目标点数值
    for (int i = 0; i < waypoint_count; ++i) 
    {
        string param_base = "waypoint_" + to_string(i+1); // 航点编号从1开始
        nh.param(param_base + "_x", waypoints[i].x, 1.0);
        nh.param(param_base + "_y", waypoints[i].y, 1.0);
        waypoints[i].z = 0.1; 
        cout << GREEN << "Get waypoints_" << i+1 << ": ["<< waypoints[i].x<< ", " << waypoints[i].y<< ", " << waypoints[i].z<<"]" << TAIL << endl;
    }

    string agent_name = "/ugv_" + std::to_string(agent_id); 
    // 【订阅】触发指令 外部 -> 本节点 
    demo_start_flag_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/demo/ugv_waypoint", 1, demo_start_flag_cb);
    // 【发布】控制指令 本节点 -> 无人车控制节点
    agent_cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 10);
    // 【发布】文字提示消息  本节点 -> 地面站
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);

    // 主循环
    while (ros::ok()) 
    {
        if(!demo_start_flag)
        {
            // 处理一次回调函数
            ros::spinOnce();
            // sleep
            rate.sleep();
            continue;
        }

        // 逐个发送目标点，发送后睡眠等待，然后发送下一个，直到遍历完所有目标点
        for (auto &waypoint : waypoints) 
        {
            agent_cmd.header.stamp = ros::Time::now();
            agent_cmd.header.frame_id = "world";
            agent_cmd.agent_id = agent_id;
            agent_cmd.cmd_source = "ugv_waypoint";
            agent_cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
            agent_cmd.desired_pos = waypoint;
            agent_cmd.desired_yaw = 0.0;    
            agent_cmd_pub.publish(agent_cmd); 

            // 发送提示消息
            text_info.data = "Moving to waypoints: [" + to_string(waypoint.x) + ", " + to_string(waypoint.y) + ", " + to_string(waypoint.z)+"]...";
            cout << GREEN << text_info.data << TAIL << endl;
            text_info_pub.publish(text_info);

            // 等待智能体移动（注：可以改为判断来确定是否发送下一个目标点）
            ros::Duration(6.0).sleep();
        }

        // 执行完所有的目标点后，重置状态位
        demo_start_flag = false; 
        text_info.data = "demo finished.";
        cout << GREEN << text_info.data << TAIL << endl;
        text_info_pub.publish(text_info);
    }

    return 0;
}