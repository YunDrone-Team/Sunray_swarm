#ifndef AGENT_SIM_H
#define AGENT_SIM_H

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <vector>

#include "printf_utils.h"
#include "ros_msg_utils.h"

using namespace std;

class AGENT_SIM
{
    public:
        // 构造函数
        AGENT_SIM(){};
        // 初始化函数
        void init(ros::NodeHandle& nh);
        // 主循环函数
        bool mainloop();

    private:
        int agent_type;
        string agent_prefix;
        string node_name;               // 节点名称
        int agent_id;                     // 无人机编号
        float agent_height;
        std_msgs::Float32 battery;


        sunray_msgs::agent_cmd current_agent_cmd;

        geometry_msgs::Twist cmd_vel;
        geometry_msgs::PoseStamped agent_pos;
        double agent_yaw;

        // 订阅话题
        ros::Subscriber agent_cmd_sub;
        ros::Subscriber agent_cmd_vel_sub;
        ros::Subscriber ugv_cmd_sub;

        // 发布话题
        ros::Publisher mocap_pos_pub;
        ros::Publisher battery_pub;

        ros::Timer debug_timer;
        void agent_cmd_cb(const sunray_msgs::agent_cmd::ConstPtr& msg);
        void agent_cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg);
        geometry_msgs::Quaternion ros_quaternion_from_rpy(double roll, double pitch, double yaw);
};
#endif