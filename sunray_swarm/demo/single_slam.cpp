#include <ros/ros.h>
#include <signal.h>
#include <vector>
#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"

using namespace std;

ros::Publisher cmd_pub;  // 发布无人机控制命令
ros::Publisher marker_pub;  // 发布RVIZ标记用于显示障碍物
int agent_type;  // 代理类型

void planAndDriveToTarget(const geometry_msgs::Point& target) {
    sunray_msgs::agent_cmd cmd;
    cmd.agent_id = 1;  // 依据代理类型设置的ID
    cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
    cmd.desired_pos = target;
    cmd.desired_yaw = 0.0;  // 默认朝向

    cmd_pub.publish(cmd);
    cout << BLUE << "Agent driving to target: x=" << target.x << " y=" << target.y << " z=" << target.z << endl;
}

void setupObstacles() {
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

    marker_pub.publish(marker);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "drone_path_planning");
    ros::NodeHandle nh;

    nh.param<int>("agent_type", agent_type, 0); // 默认为1 (Tianbot)

    string agent_prefix;
    switch(agent_type) {
        case sunray_msgs::agent_state::RMTT:
        case sunray_msgs::agent_state::SIKONG:
            agent_prefix = "rmtt_";
            break;
        case sunray_msgs::agent_state::TIANBOT:
        case sunray_msgs::agent_state::WHEELTEC:
            agent_prefix = "tianbot_";
            break;
        default:
            agent_prefix = "unknown_";
            break;
    }

    cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm/" + agent_prefix + "1/agent_cmd", 10);
    marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);

    setupObstacles();

    geometry_msgs::Point target;
    float z = 0.0;
    while (ros::ok()) {
        cout << GREEN << "Enter target position (x y" << ((agent_type == sunray_msgs::agent_state::RMTT || agent_type == sunray_msgs::agent_state::SIKONG) ? " z" : "") << "): ";
        cin >> target.x >> target.y;
        if (agent_type == sunray_msgs::agent_state::RMTT || agent_type == sunray_msgs::agent_state::SIKONG) {
            cin >> target.z;
        } else {
            target.z = 0.0;  // ugv设置
        }
        planAndDriveToTarget(target);  // 规划并行驶到目标点
        ros::spinOnce();
        ros::Duration(1.0).sleep();  // 等待无人机行驶到目标点
    }

    return 0;
}