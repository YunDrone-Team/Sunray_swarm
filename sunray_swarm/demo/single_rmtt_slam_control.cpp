#include <ros/ros.h>
#include <iostream>
#include <vector>
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include "sunray_msgs/agent_cmd.h"
#include "sunray_msgs/orca_cmd.h"
#include "printf_utils.h"

using namespace std;

ros::Publisher cmd_pub;  // 发布无人机控制命令
ros::Publisher marker_pub;  // 发布RVIZ标记用于显示障碍物

// 执行路径规划和行驶的函数
void planAndDriveToTarget(const geometry_msgs::Point& target) {
    sunray_msgs::agent_cmd cmd;
    cmd.agent_id = 1;  // RMTT的ID
    cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;  // 位置控制模式
    cmd.desired_pos = target;
    cmd.desired_yaw = 0.0;  // 默认朝向

    // 发布控制命令
    cmd_pub.publish(cmd);
    // ROS_INFO_STREAM("drone is driving to target: x=" << target.x << " y=" << target.y << " z=" << target.z);
    cout << BLUE << "drone is driving to target: x=" << target.x << " y=" << target.y << " z=" << target.z << endl;
}

// 设置障碍物，并发布障碍物信息到RVIZ
void setupObstacles() {
    // 示例障碍物位置
    geometry_msgs::Point obstacle;
    obstacle.x = 2.0;
    obstacle.y = 2.0;
    obstacle.z = 0.0;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "obstacles";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = obstacle;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 2.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // 发布障碍物标记到RVIZ
    marker_pub.publish(marker);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "drone_path_planning");
    ros::NodeHandle nh;

    // 初始化发布者
    cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm/rmtt_1/agent_cmd", 10);
    marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);

    setupObstacles();  // 初始设置障碍物

    geometry_msgs::Point target;
    while (ros::ok()) {
        cout << GREEN << "Enter target position (x y z): ";
        cin >> target.x >> target.y >> target.z;
        planAndDriveToTarget(target);  // 规划并行驶到目标点
        ros::spinOnce();
        ros::Duration(1.0).sleep();  // 等待无人机行驶到目标点
    }

    return 0;
}