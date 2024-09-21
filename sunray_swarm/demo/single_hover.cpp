#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include "ros_msg_utils.h"
#include "printf_utils.h"

ros::Publisher cmd_pub;             // 发布控制命令
int agent_type;                     // 代理类型，用于区分无人机和无人车
float agent_height;                 // 设置无人机高度
int agent_id;                       // 设置智能体编号
ros::Subscriber agent_cmd_sub;      // 触发条件
geometry_msgs::Point position_hover; // 初始化位置
bool received_start_cmd = false;    // 接收到开始命令
string agent_prefix;
string agent_name;

// 信号处理函数，用于关闭节点
void mySigintHandler(int sig) {
    ROS_INFO("Shutting down agent hover control node...");
    ros::shutdown();
}

// 触发信号的回调函数，处理接收到的位置
void startCmdCallback(const geometry_msgs::Point::ConstPtr& msg) {
    received_start_cmd = true;       // 设置标志为true
    position_hover = *msg;            // 更新接收到的位置信息
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "single_hover");
    ros::NodeHandle nh;
    signal(SIGINT, mySigintHandler); // 设置信号处理函数

    nh.param<int>("agent_type", agent_type, 1);
    nh.param<float>("agent_height", agent_height, 1.0);
    nh.param<int>("agent_id", agent_id, 1);

    string agent_prefix;
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
    string agent_name;
    agent_name = "/" + agent_prefix + std::to_string(agent_id);
    cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 10);
    agent_cmd_sub = nh.subscribe<geometry_msgs::Point>("/sunray_swarm/single_hover", 1, startCmdCallback);

    ros::Rate rate(10); // 设置循环频率为10Hz
    while (ros::ok()) {
        if (received_start_cmd) {
            sunray_msgs::agent_cmd cmd;
            cmd.agent_id = agent_id;
            cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
            cmd.desired_pos = position_hover; // 设置目标位置为接收到的位置
            cmd.desired_yaw = 0; // 可以设置偏航角
            cmd_pub.publish(cmd); // 发布命令
            ROS_INFO_STREAM("Publishing hover position: " << position_hover);
            received_start_cmd = false; // 重置标志
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}