#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <signal.h>
#include <cmath>
#include "sunray_msgs/agent_cmd.h"
#include "geometry_msgs/Point.h"
#include "printf_utils.h"

using namespace std;

ros::Publisher cmd_pub; // 发布控制命令
vector<geometry_msgs::Point> waypoints; // 航点列表
size_t current_waypoint_index = 0; // 当前航点索引
geometry_msgs::Point current_position; // 当前位置

// 处理信号，确保ROS节点可以正确退出
void mySigintHandler(int sig) {
    ROS_INFO("Shutting down UAV waypoint control node...");
    ros::shutdown();
}


// 发布航点命令的函数
void navigateToNextWaypoint() {
    if (current_waypoint_index < waypoints.size()) {
        sunray_msgs::agent_cmd cmd;
        cmd.agent_id = 1;  // UAV的ID
        cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL; // 位置控制模式
        cmd.desired_pos = waypoints[current_waypoint_index];
        cmd_pub.publish(cmd);
        cout << BLUE << "UAV navigating to waypoint: x=" << cmd.desired_pos.x << " y=" << cmd.desired_pos.y << endl;
        ros::Duration(5.0).sleep(); // 等待2秒以模拟飞行到该点
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "rmtt_waypoint_control");
    ros::NodeHandle nh;

    signal(SIGINT, mySigintHandler); // 设置信号处理函数

    // 初始化发布者
    cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm/tianbot_1/agent_cmd", 1);

    float x, y, z;
    while (ros::ok()) {
        cout << GREEN << "Enter waypoint position (x, y): ";
        cin >> x >> y ;
        geometry_msgs::Point point;
        point.x = x;
        point.y = y;

        waypoints.push_back(point); // 添加航点至列表

        cout << GREEN << "Press 'n' to add another waypoint, or 's' to start flying: ";
        char choice;
        cin >> choice;

        if (choice == 's') {
            navigateToNextWaypoint(); // 开始导航至航点
            waypoints.clear(); // 清空航点列表以便重新规划
        }
    }

    return 0;
}