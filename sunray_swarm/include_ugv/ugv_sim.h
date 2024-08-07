#ifndef UGV_SIM_H
#define UGV_SIM_H

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <vector>

#include "printf_utils.h"
#include "ros_msg_utils.h"

using namespace std;

class UGV_SIM
{
    public:
        // 构造函数
        UGV_SIM(){};
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

        // 执行状态
        enum CONTROL_STATE
        {
            INIT = 0,               // 初始模式     
            HOLD = 1,               // 原地停止
            POS_CONTROL = 2,        // 位置控制
            VEL_CONTROL = 3,        // 速度控制
            ORCA_MODE = 4,          // ORCA控制模式
            TRACK_MODE = 5,         // 追踪控制模式
            RETURN_HOME = 6,        // 返回起始点
        };
        CONTROL_STATE control_state;

        sunray_msgs::ugv_cmd current_ugv_cmd;

        geometry_msgs::Twist cmd_vel;
        sunray_msgs::orca_state agent_orca_state;
        geometry_msgs::PoseStamped agent_pos;
        double agent_yaw;

        // 订阅话题
        ros::Subscriber ugv_cmd_sub;
        ros::Subscriber ugv_cmd_vel_sub;
        ros::Subscriber orca_state_sub;

        // 发布话题
        ros::Publisher mocap_pos_pub;
        ros::Publisher battery_pub;

        ros::Timer debug_timer;
        void ugv_cmd_cb(const sunray_msgs::ugv_cmd::ConstPtr& msg);
        void ugv_cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg);
        void agent_orca_state_cb(const sunray_msgs::orca_state::ConstPtr& msg);
        geometry_msgs::Quaternion ros_quaternion_from_rpy(double roll, double pitch, double yaw);
};
#endif