#include <ros/ros.h>
#include <signal.h>
#include "sunray_msgs/agent_cmd.h"
#include "geometry_msgs/Point.h"
#include "math_utils.h"
#include "printf_utils.h"

using namespace std;

ros::Publisher cmd_pub; // 发布控制命令
float desired_yaw;
float circle_radius;
float linear_vel;
float time_trajectory = 0.0;
float trajectory_total_time;
float omega;

// 处理信号，确保ROS节点可以正确退出
void mySigintHandler(int sig) {
    ROS_INFO("[circle_trajectory] exit...");
    ros::shutdown();
}

// 初始化参数
void initParams(ros::NodeHandle& nh) {
    nh.param<float>("desired_yaw", desired_yaw, 0.0f);
    nh.param<float>("circle_radius", circle_radius, 1.0f);
    nh.param<float>("linear_vel", linear_vel, 0.3f);

    if (circle_radius != 0) {
        omega = linear_vel / circle_radius;  // 匀速圆周运动的角速度
    } else {
        omega = 0.0;
    }

    // ROS_INFO_STREAM("Params -> Yaw: " << desired_yaw << ", Radius: " << circle_radius << ", Linear Velocity: " << linear_vel);
    cout << BLUE << "Params -> Yaw: " << desired_yaw << ", Radius: " << circle_radius << ", Linear Velocity: " << linear_vel << endl;
}

// 发布圆形轨迹的控制命令
void publishCircleCommand() {
    float angle = time_trajectory * omega;
    sunray_msgs::agent_cmd cmd;
    cmd.agent_id = 1;  // Tianbot的ID
    cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
    cmd.desired_pos.x = circle_radius * cos(angle); // 圆心假设为原点
    cmd.desired_pos.y = circle_radius * sin(angle);
    cmd.desired_yaw = desired_yaw;
    cmd_pub.publish(cmd);
    // ROS_INFO_STREAM("Tianbot moving to: x=" << cmd.desired_pos.x << " y=" << cmd.desired_pos.y << " yaw=" << desired_yaw);
    cout << BLUE << "Tianbot moving to: x=" << cmd.desired_pos.x << " y=" << cmd.desired_pos.y << " yaw=" << desired_yaw << endl;

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tianbot_circle_trajectory");
    ros::NodeHandle nh;

    // 注册信号处理函数
    signal(SIGINT, mySigintHandler);
    
    // 初始化参数
    initParams(nh);
    
    // 初始化发布者
    cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm/tianbot_1/agent_cmd", 10);

    // ROS_INFO("Enter total trajectory time (seconds): ");
    cout << GREEN << "Enter total trajectory time (seconds): ";
    cin >> trajectory_total_time;

    // 主循环
    while (ros::ok() && time_trajectory < trajectory_total_time) {
        publishCircleCommand(); // 发布圆形轨迹控制命令
        time_trajectory += 0.1; // 模拟时间增加
        ros::spinOnce();
        ros::Duration(0.1).sleep(); // 模拟周期更新
    }

    return 0;
}