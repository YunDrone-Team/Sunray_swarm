#include <ros/ros.h>
#include <signal.h>
#include <cmath>
#include "sunray_msgs/agent_cmd.h"
#include "printf_utils.h"
#include "math_utils.h"
using namespace std;

ros::Publisher cmd_pub; // 发布控制命令
float desired_yaw;
float circle_radius;
float linear_vel;
float time_trajectory = 0.0;
float trajectory_total_time;
float omega;

int start_cmd = 0;
int agent_type; // 代理类型，用于区分无人机和无人车

// 处理信号
void mySigintHandler(int sig) {
    ROS_INFO("[rmtt_circle_trajectory] exit...");
    ros::shutdown();
}

// 初始化参数
void initParams(ros::NodeHandle& nh) {
    nh.param<float>("desired_yaw", desired_yaw, 0.0f);
    nh.param<float>("circle_radius", circle_radius, 1.0f);
    nh.param<float>("linear_vel", linear_vel, 0.3f);
    nh.param<int>("agent_type", agent_type, 1); // 默认为1 (RMTT)

    if (circle_radius != 0) {
        omega = linear_vel / circle_radius;  // 匀速圆周运动的角速度
    } else {
        omega = 0.0;
    }
}

// 发布到初始位置的命令
void publishInitialPosition() {
    sunray_msgs::agent_cmd cmd;
    cmd.agent_id = 1; // ID
    cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
    cmd.desired_pos.x = circle_radius; // 圆周轨迹起始位置
    cmd.desired_pos.y = 0.0;
    cmd.desired_pos.z = agent_type == sunray_msgs::agent_state::RMTT || agent_type == sunray_msgs::agent_state::SIKONG ? 1.0 : 0.0; // 高度设定
    cmd.desired_yaw = desired_yaw;
    cmd_pub.publish(cmd);
    cout << GREEN << "Moving to start position: x=" << circle_radius << " y=0" << " z=" << cmd.desired_pos.z << " yaw=" << desired_yaw << TAIL << endl;
}

// 发布圆形轨迹的控制命令
void publishCircleCommand() {
    float angle = time_trajectory * omega;
    sunray_msgs::agent_cmd cmd;
    cmd.agent_id = 1;  // RMTT的ID
    cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
    cmd.desired_pos.x = circle_radius * cos(angle);
    cmd.desired_pos.y = circle_radius * sin(angle);
    cmd.desired_pos.z = agent_type == sunray_msgs::agent_state::RMTT || agent_type == sunray_msgs::agent_state::SIKONG ? 1.0 : 0.0; // 高度设定
    cmd.desired_yaw = desired_yaw;
    cmd_pub.publish(cmd);
    cout << BLUE << "Moving in circle: x=" << cmd.desired_pos.x << " y=" << cmd.desired_pos.y << " z=" << cmd.desired_pos.z << " yaw=" << desired_yaw << TAIL << endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "agent_circle_control");
    ros::NodeHandle nh;

    signal(SIGINT, mySigintHandler);
    initParams(nh);


    string agent_prefix;
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

    cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm/" + agent_prefix + "1/agent_cmd", 10);

    // cout << BLUE << "Params -> Yaw: " << desired_yaw << ", Radius: " << circle_radius << ", Linear Velocity: " << linear_vel << ", Agent Prefix: " << agent_prefix << TAIL << endl;


    cout << GREEN << "Enter 1 to move to start position..." << TAIL << endl;
    cin >> start_cmd;
    if (start_cmd == 1) {
        publishInitialPosition();
        ros::Duration(2.0).sleep();
    }

    cout << GREEN << "Enter total trajectory time (seconds): " << TAIL;
    cin >> trajectory_total_time;

    while (ros::ok() && time_trajectory < trajectory_total_time) {
        publishCircleCommand();
        time_trajectory += 0.1;
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }

    return 0;
}