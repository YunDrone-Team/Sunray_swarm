#include <ros/ros.h>
#include <iostream>
#include <vector>
#include <signal.h>
#include "sunray_msgs/agent_cmd.h"
#include "geometry_msgs/Point.h"
#include "printf_utils.h"

using namespace std;

ros::Publisher cmd_pub; // 发布控制命令
vector<geometry_msgs::Point> waypoints; // 航点列表

// 处理信号，确保ROS节点可以正确退出
void mySigintHandler(int sig) {
    ROS_INFO("Shutting down drone waypoint control node...");
    ros::shutdown();
}

// 发布航点命令的函数
void navigateToWaypoints() {
    sunray_msgs::agent_cmd cmd;
    cmd.agent_id = 1;  // RMTT的ID
    cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL; // 位置控制模式

    for (auto& waypoint : waypoints) {
        cmd.desired_pos = waypoint;
        cmd_pub.publish(cmd);
        // ROS_INFO_STREAM("Drone navigating to waypoint: x=" << waypoint.x << " y=" << waypoint.y << " z=" << waypoint.z);
        cout << BLUE << "Drone navigating to waypoint: x=" << waypoint.x << " y=" << waypoint.y << " z=" << waypoint.z << endl;
        ros::Duration(10.0).sleep(); // 等待2秒以模拟飞行到该点
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "rmtt_waypoint_control");
    ros::NodeHandle nh;

    signal(SIGINT, mySigintHandler); // 设置信号处理函数

    // 初始化发布者
    cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm/rmtt_1/agent_cmd", 1);

    float x, y, z;
    while (ros::ok()) {
        cout << GREEN << "Enter waypoint position (x, y, z): ";
        cin >> x >> y >> z;
        geometry_msgs::Point point;
        point.x = x;
        point.y = y;
        point.z = z;
        waypoints.push_back(point); // 添加航点至列表

        cout << GREEN << "Press 'n' to add another waypoint, or 's' to start flying: ";
        char choice;
        cin >> choice;

        if (choice == 's') {
            navigateToWaypoints(); // 开始导航至航点
            waypoints.clear(); // 清空航点列表以便重新规划
        }
    }

    return 0;
}

