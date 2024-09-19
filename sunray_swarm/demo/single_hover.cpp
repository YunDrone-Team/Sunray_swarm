#include <ros/ros.h>
#include <signal.h>
#include "sunray_msgs/agent_cmd.h"
#include "geometry_msgs/Point.h"
#include <iostream>
#include "ros_msg_utils.h"
#include "printf_utils.h"
using namespace std;

ros::Publisher cmd_pub;        // 发布控制命令
int agent_type;                // 代理类型，用于区分无人机和无人车
float agent_height;            // 设置无人机高度
int agent_num;                 // 设置智能体数量
ros::Subscriber agent_cmd_sub; // 触发条件

// 信号处理函数，用于关闭节点
void mySigintHandler(int sig)
{
    ROS_INFO("Shutting down agent hover control node...");
    ros::shutdown();
}

// 发布悬停位置的函数
void publishHoverPosition(float x, float y, float z, float yaw)
{
    sunray_msgs::agent_cmd cmd;
    // 代理的ID
    cmd.agent_id = 1;
    // 设置位置控制模式
    cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
    // 设置期望的悬停位置x坐标
    cmd.desired_pos.x = x;
    cmd.desired_pos.y = y;
    // 设置期望的悬停位置z坐标（高度agent_height）
    cmd.desired_pos.z = agent_height;
    // 设置期望的悬停姿态（偏航角）
    cmd.desired_yaw = yaw;
    cmd_pub.publish(cmd);
    cout << BLUE << "Agent moving to hover position: x=" << x << " y=" << y << " z=" << z << " yaw=" << yaw << endl;
}
// 触发信号的回调函数
void startCmdCallback(const std_msgs::Bool::ConstPtr &msg)
{
    if (msg->data)
    {
        ros::Rate rate(10);
        // 定义悬停位置变量，默认z为0，对于无人车适用
        float x, y, yaw, z = 0.0; // 默认Z为0，对于无人车适用
        if (agent_type == sunray_msgs::agent_state::RMTT || agent_type == sunray_msgs::agent_state::SIKONG)
        {
            z = agent_height; // 默认无人机高度为1m
        }

        // 从用户输入获取初始悬停位置
        x = 1;
        y = 1;
        yaw = 0;
        yaw = yaw * M_PI / 180.0; // 转换为弧度
        // 发布起始悬停位置
        publishHoverPosition(x, y, z, yaw);

        // 主循环
        while (ros::ok())
        {
            // cout << GREEN << "Update hover position (x, y, (z), yaw in degrees, and z if UAV): ";
            if (z != 0.0)
            {
                x = 1;
                y = 1;
                z = agent_height;
                yaw = 0;
            }
            else
            {
                x = 1;
                y = 1;
                yaw = 0; // 如果是无人车，输入不包含z轴
            }
            // 将偏航角从度转换为弧度
            yaw = yaw * M_PI / 180.0;
            publishHoverPosition(x, y, z, yaw); // 根据输入更新悬停位置
            rate.sleep();                       //
        }
    }
    else
    {
        // 如果接收到的消息是false，也执行一些操作
        ROS_INFO("Received false ");
        //     // 这里可以添加执行其他任务的代码
        // // 定义悬停位置变量，默认z为0，对于无人车适用
        // float x, y, yaw, z = 0.0;  // 默认Z为0，对于无人车适用
        // if (agent_type == sunray_msgs::agent_state::RMTT || agent_type == sunray_msgs::agent_state::SIKONG) {
        //     z = agent_height; // 默认无人机高度为1m
        // }

        // // 从用户输入获取初始悬停位置
        // cout << GREEN << "Enter initial hover position (x, y, (z), yaw in degrees, and z if UAV): ";
        // if (z != 0.0) {
        //     cin >> x >> y >> z >> yaw;  // 无人机包含z轴
        // } else {
        //     cin >> x >> y >> yaw;  // 无人车不包含z轴
        // }
        // yaw = yaw * M_PI / 180.0; // 转换为弧度
        // // 发布起始悬停位置
        // publishHoverPosition(x, y, z, yaw);

        // // // 主循环
        // while (ros::ok()) {
        //     cout << GREEN << "Update hover position (x, y, (z), yaw in degrees, and z if UAV): ";
        //     if (z != 0.0) {
        //         cin >> x >> y >> z >> yaw; // 如果是无人机，输入包含z轴
        //     } else {
        //         cin >> x >> y >> yaw;  // 如果是无人车，输入不包含z轴
        //     }
        //     // 将偏航角从度转换为弧度
        //     yaw = yaw * M_PI / 180.0;
        //     publishHoverPosition(x, y, z, yaw); // 根据输入更新悬停位置
        //     // 休眠0.1秒
        //     ros::Duration(0.1).sleep();

        // }
        // }
    }

    int main(int argc, char **argv)
    {
        // 初始化ROS节点，节点名为"agent_hover_control"
        ros::init(argc, argv, "agent_hover_control");
        // 创建节点句柄
        ros::NodeHandle nh;
        // 设置信号处理函数，用于关闭节点
        signal(SIGINT, mySigintHandler);
        // 从参数服务器获取agent_type参数的值，默认为1 (tianbot)
        nh.param<int>("agent_type", agent_type, 1);
        // 【参数】智能体高度
        nh.param<float>("agent_height", agent_height, 1.0);
        // 定义一个字符串变量，用于存储代理前缀

        // 【参数】智能体编号
        nh.param<int>("agent_num", agent_num, 1);
        string agent_prefix;

        // 根据类型选择适当的前缀
        switch (agent_type)
        {
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

        // 初始化发布者
        string agent_name;
        for (int i = 0; i < agent_num; i++)
        {
            agent_name = "/" + agent_prefix + std::to_string(i + 1);
            cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 10);
        }

        // [订阅]触发条件
        agent_cmd_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/single_hover", 1, startCmdCallback);

        // // 定义悬停位置变量，默认z为0，对于无人车适用
        // float x, y, yaw, z = 0.0;  // 默认Z为0，对于无人车适用
        // if (agent_type == sunray_msgs::agent_state::RMTT || agent_type == sunray_msgs::agent_state::SIKONG) {
        //     z = agent_height; // 默认无人机高度为1m
        // }

        // // 从用户输入获取初始悬停位置
        // cout << GREEN << "Enter initial hover position (x, y, (z), yaw in degrees, and z if UAV): ";
        // if (z != 0.0) {
        //     cin >> x >> y >> z >> yaw;  // 无人机包含z轴
        // } else {
        //     cin >> x >> y >> yaw;  // 无人车不包含z轴
        // }
        // yaw = yaw * M_PI / 180.0; // 转换为弧度
        // // 发布起始悬停位置
        // publishHoverPosition(x, y, z, yaw);

        // // 主循环
        while (ros::ok())
        {
            //     cout << GREEN << "Update hover position (x, y, (z), yaw in degrees, and z if UAV): ";
            //     if (z != 0.0) {
            //         cin >> x >> y >> z >> yaw; // 如果是无人机，输入包含z轴
            //     } else {
            //         cin >> x >> y >> yaw;  // 如果是无人车，输入不包含z轴
            //     }
            //     // 将偏航角从度转换为弧度
            //     yaw = yaw * M_PI / 180.0;
            //     publishHoverPosition(x, y, z, yaw); // 根据输入更新悬停位置
            ros::spinOnce(); // 处理回调函数
            // 休眠0.1秒
            ros::Duration(0.1).sleep();
        }

        return 0;
    }