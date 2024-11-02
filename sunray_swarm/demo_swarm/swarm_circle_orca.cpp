#include <ros/ros.h>
#include <signal.h>
#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"

#define MAX_AGENT_NUM 100

using namespace std;
int agent_type;                                  // 智能体类型
int agent_num;                                   // 智能体数量
float desired_yaw;                               // 期望偏航角
float circle_radius;                             // 圆的半径
Eigen::Vector3f circle_center;                   // 圆心坐标
float omega;                                     // 角速度
bool received_start_cmd = false;                 // 标记是否接收到开始命令
sunray_msgs::orca_cmd orca_cmd;                         // ORCA指令

ros::Publisher orca_goal_pub[MAX_AGENT_NUM];     // 目标点发布者
ros::Subscriber orca_state_sub[MAX_AGENT_NUM];   // ORCA状态订阅者
ros::Subscriber swarm_circle_cmd_sub;            // 画圈命令的订阅者
sunray_msgs::orca_state orca_state[MAX_AGENT_NUM];      // ORCA状态
ros::Publisher orca_cmd_pub;                        // 发布ORCA指令

// 信号处理函数
void mySigintHandler(int sig) {
    ROS_INFO("[circle_trajectory] exit...");
    ros::shutdown();
}

// 处理ORCA状态回调
void rmtt_orca_state_cb(const sunray_msgs::orca_stateConstPtr& msg, int i) {
    // 更新指定智能体的ORCA状态
    orca_state[i] = *msg;
}

// 处理画圈命令的回调函数
void swarm_circle_cb(const std_msgs::Bool::ConstPtr &msg) {
    received_start_cmd = msg->data; // 设置接收到的开始命令
    // 设置ORCA命令为HOME
    orca_cmd.orca_cmd = sunray_msgs::orca_cmd::SET_HOME;
    // 发布命令
    orca_cmd_pub.publish(orca_cmd);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "circle_trajectory");
    ros::NodeHandle nh("~");
    ros::Rate rate(100.0);

    nh.param<int>("agent_num", agent_num, 8);
    nh.param<float>("circle_center_x", circle_center[0], 0.0f);
    nh.param<float>("circle_center_y", circle_center[1], 0.0f);
    nh.param<float>("agent_height", circle_center[2], 1.0f);
    nh.param<float>("circle_radius", circle_radius, 1.0f);

    // 订阅ORCA状态，发布目标点
    string agent_prefix = "rmtt_";
    // 【发布】ORCA指令 本节点 ->  ORCA
    orca_cmd_pub = nh.advertise<sunray_msgs::orca_cmd>("/sunray_swarm/" + agent_prefix + "/orca_cmd", 1);
    string agent_name;
    for (int i = 0; i < agent_num; i++) {
        agent_name = "/" + agent_prefix + std::to_string(i + 1);
        orca_goal_pub[i] = nh.advertise<geometry_msgs::Point>("/sunray_swarm" + agent_name + "/goal_point", 1);
        orca_state_sub[i] = nh.subscribe<sunray_msgs::orca_state>("/sunray_swarm" + agent_name + "/agent_orca_state", 1, boost::bind(&rmtt_orca_state_cb, _1, i));
    }
    swarm_circle_cmd_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/demo/swarm_circle", 1, swarm_circle_cb);

    while (ros::ok()) {
        if (received_start_cmd) {
            for (int i = 0; i < agent_num; i++) {
                float angle = i * 2 * M_PI / agent_num;
                geometry_msgs::Point goal_point;
                goal_point.x = circle_center[0] + circle_radius * cos(angle);
                goal_point.y = circle_center[1] + circle_radius * sin(angle);
                goal_point.z = circle_center[2];
                orca_goal_pub[i].publish(goal_point);
            }
            received_start_cmd = false;
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}