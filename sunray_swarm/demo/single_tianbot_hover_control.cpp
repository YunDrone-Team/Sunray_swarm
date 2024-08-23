#include <ros/ros.h>
#include <signal.h>
#include "sunray_msgs/agent_cmd.h"
#include "geometry_msgs/Point.h"
#include <iostream>
#include "printf_utils.h"

using namespace std;

ros::Publisher cmd_pub;  // 发布控制命令

// 处理信号，确保ROS节点可以正确退出
void mySigintHandler(int sig) {
    ROS_INFO("Shutting down Tianbot hover control node...");
    ros::shutdown();
}

// 发布停车位置的函数
void publishHoverPosition(float x, float y, float yaw) {
    sunray_msgs::agent_cmd cmd;
    cmd.agent_id = 1;  // RMTT的ID
    cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL; // 位置控制模式
    cmd.desired_pos.x = x;
    cmd.desired_pos.y = y;
    cmd.desired_pos.z = 0.0;  // 无人车无Z轴控制
    cmd.desired_yaw = yaw;
    cmd_pub.publish(cmd);
    // ROS_INFO_STREAM("Tianbot moving to hover position: x=" << x << " y=" << y << " yaw=" << yaw);
    cout << BLUE << "Tianbot moving to hover position: x=" << x << " y=" << y << " yaw=" << yaw << endl;

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tianbot_hover_control");
    ros::NodeHandle nh;

    signal(SIGINT, mySigintHandler); // 设置信号处理函数

    // 初始化发布者
    cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm/tianbot_1/agent_cmd", 10);

    float x, y, yaw;
    cout << GREEN << "Enter initial hover position (x, y, yaw in degrees): ";
    cin >> x >> y >> yaw;
    yaw = yaw * M_PI / 180.0; // 转换为弧度

    publishHoverPosition(x, y, yaw); // 发布起始悬停位置

    // 主循环
    while (ros::ok()) {
        cout << GREEN <<  "Update hover position (x, y, yaw in degrees): ";
        cin >> x >> y >> yaw;
        yaw = yaw * M_PI / 180.0; // 转换为弧度
        publishHoverPosition(x, y, yaw); // 根据用户输入更新悬停位置
        ros::spinOnce(); // 处理回调函数
    }

    return 0;
}