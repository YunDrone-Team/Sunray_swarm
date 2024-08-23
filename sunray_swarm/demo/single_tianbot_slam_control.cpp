#include <ros/ros.h>
#include <iostream>
#include <vector>
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include "sunray_msgs/agent_cmd.h"
#include "sunray_msgs/orca_cmd.h"
#include "printf_utils.h"

using namespace std;

ros::Publisher cmd_pub;  // 发布无人车控制命令
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
    // ROS_INFO_STREAM("UGV is driving to target: x=" << target.x << " y=" << target.y << " z=" << target.z);
    cout << BLUE << "Tianbot is driving to target: x=" << target.x << " y=" << target.y << " z=" << target.z << endl;
}


// void ORCA::setup_obstacles()
// {
// 	// 声明障碍物（凸多边形），障碍物建立规则：逆时针依次添加多边形的顶点
// 	std::vector<RVO::Vector2> obstacle1, obstacle2, obstacle3, obstacle4;

// 	// 障碍物示例：中心在原点，边长为1的正方体
// 	// obstacle1.push_back(RVO::Vector2(0.5f, 0.5f));
// 	// obstacle1.push_back(RVO::Vector2(-0.5f, 0.5f));
// 	// obstacle1.push_back(RVO::Vector2(-0.5f, 0.5f));
// 	// obstacle1.push_back(RVO::Vector2(0.5f, -0.5f));

// 	obstacle2.push_back(RVO::Vector2(10.0f, 40.0f));
// 	obstacle2.push_back(RVO::Vector2(10.0f, 10.0f));
// 	obstacle2.push_back(RVO::Vector2(40.0f, 10.0f));
// 	obstacle2.push_back(RVO::Vector2(40.0f, 40.0f));

// 	// obstacle3.push_back(RVO::Vector2(10.0f, -40.0f));
// 	// obstacle3.push_back(RVO::Vector2(40.0f, -40.0f));
// 	// obstacle3.push_back(RVO::Vector2(40.0f, -10.0f));
// 	// obstacle3.push_back(RVO::Vector2(10.0f, -10.0f));

// 	// obstacle4.push_back(RVO::Vector2(-10.0f, -40.0f));
// 	// obstacle4.push_back(RVO::Vector2(-10.0f, -10.0f));
// 	// obstacle4.push_back(RVO::Vector2(-40.0f, -10.0f));
// 	// obstacle4.push_back(RVO::Vector2(-40.0f, -40.0f));
    
//     // 在算法中添加障碍物
// 	// sim->addObstacle(obstacle1);
// 	sim->addObstacle(obstacle2);
// 	// sim->addObstacle(obstacle3);
// 	// sim->addObstacle(obstacle4);

// 	// 在算法中处理障碍物信息
// 	sim->processObstacles();

//     cout << BLUE << node_name << ":  Set obstacles success!" << TAIL << endl;
// }


// 设置障碍物，并发布障碍物信息到RVIZ
void setupObstacles() {
    // 示例障碍物位置
    geometry_msgs::Point obstacle;
    obstacle.x = 3.0;
    obstacle.y = 3.0;
    obstacle.z = 0.0;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "obstacles";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = obstacle;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // 发布障碍物标记到RVIZ
    marker_pub.publish(marker);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ugv_path_planning");
    ros::NodeHandle nh;

    // 初始化发布者
    cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm/tianbot_1/agent_cmd", 10);
    marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);

    setupObstacles();  // 初始设置障碍物

    geometry_msgs::Point target;
    while (ros::ok()) {
        cout << GREEN << "Enter target position (x y z): ";
        cin >> target.x >> target.y >> target.z;
        planAndDriveToTarget(target);  // 规划并行驶到目标点
        ros::spinOnce();
        ros::Duration(1.0).sleep();  // 等待无人车行驶到目标点
    }

    return 0;
}