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
        // 节点名称
        string node_name;   
        // 智能体类型
        int agent_type;
        // 智能体编号 - 通过参数配置
        int agent_id;         
        // 智能体名称 = 智能体类型+ID号
        string agent_name;
        // 智能体的固定高度 - 通过参数配置
        float agent_height;
        // 电池电量
        std_msgs::Float32 battery;

        // 智能体当前控制指令
        sunray_msgs::agent_cmd current_agent_cmd;

        // 智能体当前底层控制指令
        geometry_msgs::Twist cmd_vel;

        // 智能体位置+姿态
        geometry_msgs::PoseStamped agent_pos;
        double agent_yaw;

        // 订阅话题
        ros::Subscriber agent_cmd_sub;
        ros::Subscriber agent_cmd_vel_sub;

        // 发布话题
        ros::Publisher mocap_pos_pub;
        ros::Publisher battery_pub;

        ros::Timer debug_timer;
        void agent_cmd_cb(const sunray_msgs::agent_cmd::ConstPtr& msg);
        void agent_cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg);
        geometry_msgs::Quaternion ros_quaternion_from_rpy(double roll, double pitch, double yaw);
};
#endif