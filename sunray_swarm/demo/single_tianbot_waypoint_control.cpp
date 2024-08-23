#include <ros/ros.h>
#include <cmath>
#include "geometry_msgs/Point.h"
#include "sunray_msgs/agent_cmd.h"

using namespace std;

ros::Publisher cmd_pub; // 发布控制命令
float circle_radius = 5.0; // 默认圆形轨迹半径
float linear_velocity = 1.0; // 默认线速度
geometry_msgs::Point circle_center; // 圆心位置

// 更新圆形轨迹的函数
void updateCircleTrajectory(float radius, float velocity) {
    circle_radius = radius;
    linear_velocity = velocity;
}

// 执行圆形轨迹飞行的函数
void flyCircleTrajectory() {
    sunray_msgs::agent_cmd cmd;
    cmd.agent_id = 1;  // RMTT的ID
    cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL; // 位置控制模式
    float angle = 0.0;

    while (ros::ok()) {
        // 计算当前航点位置
        cmd.desired_pos.x = circle_center.x + circle_radius * cos(angle);
        cmd.desired_pos.y = circle_center.y + circle_radius * sin(angle);
        cmd.desired_pos.z = circle_center.z; // 保持高度不变
        cmd.desired_yaw = atan2(sin(angle), cos(angle)); // 面向圆心飞行

        cmd_pub.publish(cmd);
        // ROS_INFO_STREAM("UAV driving to circle point: x=" << cmd.desired_pos.x << " y=" << cmd.desired_pos.y << " yaw=" << cmd.desired_yaw);
        cout << BLUE << "Tianbot navigating to waypoint: x=" << cmd.desired_pos.x << " y=" << cmd.desired_pos.y << " yaw=" << cmd.desired_yaw << endl;

        angle += linear_velocity / circle_radius; // 根据速度更新角度
        if (angle > 2 * M_PI) {
            angle -= 2 * M_PI;
        }

        ros::Duration(0.1).sleep(); // 控制更新频率
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "uav_circle_trajectory");
    ros::NodeHandle nh;

    // 初始化发布者
    cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm/tianbot_1/agent_cmd", 10);
    circle_center.x = 0.0;
    circle_center.y = 0.0;
    circle_center.z = 10.0; // 假设起始高度为10米

    float new_radius, new_velocity;
    while (ros::ok()) {
        cout << GREEN << "Enter new circle radius and linear velocity: ";
        cin >> new_radius >> new_velocity;
        updateCircleTrajectory(new_radius, new_velocity); // 更新圆形轨迹参数
        flyCircleTrajectory(); // 开始按新轨迹飞行
    }

    return 0;
}