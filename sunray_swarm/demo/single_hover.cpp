#include <ros/ros.h>
#include <signal.h>
#include "sunray_msgs/agent_cmd.h"
#include "geometry_msgs/Point.h"
#include <iostream>
#include "ros_msg_utils.h"
#include "printf_utils.h"
using namespace std;

ros::Publisher cmd_pub;  // 发布控制命令
int agent_type; // 代理类型，用于区分无人机和无人车

void mySigintHandler(int sig) {
    ROS_INFO("Shutting down agent hover control node...");
    ros::shutdown();
}

// 发布悬停位置的函数
void publishHoverPosition(float x, float y, float z, float yaw) {
    sunray_msgs::agent_cmd cmd;
    cmd.agent_id = 1;  // 代理的ID
    cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL; // 位置控制模式
    cmd.desired_pos.x = x;
    cmd.desired_pos.y = y;
    cmd.desired_pos.z = z;  // 根据类型设置高度
    cmd.desired_yaw = yaw;
    cmd_pub.publish(cmd);
    cout << BLUE << "Agent moving to hover position: x=" << x << " y=" << y << " z=" << z << " yaw=" << yaw << endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "agent_hover_control");
    ros::NodeHandle nh;

    signal(SIGINT, mySigintHandler); // 设置信号处理函数

    nh.param<int>("agent_type", agent_type, 1); // 默认为1 (RMTT)
    string agent_prefix;

    // 根据代理类型选择适当的前缀
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

    // 初始化发布者
    cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm/" + agent_prefix + "1/agent_cmd", 10);

    float x, y, yaw, z = 0.0;  // 默认Z为0，对于无人车适用
    if (agent_type == sunray_msgs::agent_state::RMTT || agent_type == sunray_msgs::agent_state::SIKONG) {
        z = 1.0; // 默认无人机高度为1m
    }

    cout << GREEN << "Enter initial hover position (x, y, yaw in degrees, and z if UAV): ";
    if (z != 0.0) {
        cin >> x >> y >> z >> yaw;  // 无人机包含z轴
    } else {
        cin >> x >> y >> yaw;  // 无人车不包含z轴
    }
    yaw = yaw * M_PI / 180.0; // 转换为弧度

    publishHoverPosition(x, y, z, yaw); // 发布起始悬停位置

    // 主循环
    while (ros::ok()) {
        cout << GREEN << "Update hover position (x, y, yaw in degrees, and z if UAV): ";
        if (z != 0.0) {
            cin >> x >> y >> z >> yaw;
        } else {
            cin >> x >> y >> yaw;
        }
        yaw = yaw * M_PI / 180.0; // 转换为弧度
        publishHoverPosition(x, y, z, yaw); // 根据输入更新悬停位置
        ros::spinOnce(); // 处理回调函数
    }

    return 0;
}