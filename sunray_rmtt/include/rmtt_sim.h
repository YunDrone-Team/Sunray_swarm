#ifndef RMTT_SIM_H
#define RMTT_SIM_H

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <vector>

#include "printf_utils.h"
#include "ros_msg_utils.h"

using namespace std;

class RMTT_SIM
{
    public:
        // 构造函数
        RMTT_SIM(){};
        // 初始化函数
        void init(ros::NodeHandle& nh);
        // 主循环函数
        bool mainloop();

    private:
        string node_name;               // 节点名称
        string uav_name{""};            // 无人机名称
        int uav_id;                     // 无人机编号
        float rmtt_height;
        int rmtt_num;
        std_msgs::Float32 battery;
        // 执行状态
        // 执行状态
        enum MISSION_STATE
        {
            INIT = 0,               // 初始模式     
            TAKEOFF = 1,     // 
            LAND = 2,    // 降落
            HOLD = 3,        // 悬停
            ORCA_SETUP = 4,
            ORCA_RUN = 5,
            RETURN_HOME = 6,
        };
        MISSION_STATE mission_state;

        sunray_msgs::station_cmd current_station_cmd;

        geometry_msgs::Twist cmd_vel;
        sunray_msgs::rmtt_state rmtt_state;
        sunray_msgs::rmtt_orca rmtt_orca_state;
        geometry_msgs::PoseStamped rmtt_pos;

        // 订阅话题
        ros::Subscriber rmtt_cmd_sub;
        ros::Subscriber rmtt_orca_state_sub;
        ros::Subscriber station_cmd_sub;
        ros::Subscriber station_cmd_sub2;

        // 发布话题
        ros::Publisher rmtt_state_pub;
        ros::Publisher mocap_pos_pub;
        ros::Publisher mocap_vel_pub;
        ros::Publisher rmtt_goal_pub;
        ros::Publisher battery_pub;

        ros::Timer debug_timer;
        void station_cmd_cb(const sunray_msgs::station_cmd::ConstPtr& msg);
        void station_cmd_cb2(const sunray_msgs::station_cmd::ConstPtr& msg);
        void rmtt_cmd_cb(const geometry_msgs::Twist::ConstPtr& msg);
        void rmtt_state_cb(const sunray_msgs::rmtt_state::ConstPtr& msg);
        void rmtt_orca_state_cb(const sunray_msgs::rmtt_orca::ConstPtr& msg);
        void debugCb(const ros::TimerEvent &e);
};
#endif