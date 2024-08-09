#ifndef UGV_CONTROL_H
#define UGV_CONTROL_H

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <vector>

#include "printf_utils.h"
#include "ros_msg_utils.h"

using namespace std;

#define TRA_WINDOW 50        
#define MOCAP_TIMEOUT 0.35                 

class UGV_CONTROL
{
    public:
        // 构造函数
        UGV_CONTROL(){};
        // 初始化函数
        void init(ros::NodeHandle& nh);
        // 主循环函数
        void mainloop();

    private:
        int agent_type;
        string node_name;               // 节点名称
        string agent_prefix{""};           
        string agent_ip;
        int agent_id;                     // 无人机编号
        bool flag_printf;
        bool sim_mode;
        ros::Time get_mocap_time{0};
        bool get_target_pos{false};
        string target_name;
        float agent_height;

        sunray_msgs::agent_state agent_state;           
        sunray_msgs::agent_state agent_state_last;      
        geometry_msgs::Point desired_position;
        double desired_yaw;

        geometry_msgs::Twist desired_vel;

        std_msgs::ColorRGBA led_color;
        std_msgs::String text_info;
        vector<geometry_msgs::PoseStamped> pos_vector;    // 无人机轨迹容器,用于rviz显示

        // 地理围栏
        struct geo_fence
        {
            float max_x;
            float min_x;
            float max_y;
            float min_y;
            float max_z;
            float min_z;
        };
        geo_fence ugv_geo_fence;

        // 悬停控制参数
        struct control_param
        {
            float Kp_xy;
            float Kp_z;
            float Kp_yaw;
            float max_vel_xy;
            float max_vel_z;
            float max_vel_yaw;
        };
        control_param ugv_control_param;

        bool all_odom_valid{false};
        bool all_connected{false};

        sunray_msgs::agent_cmd current_agent_cmd;

        // 订阅话题
        ros::Subscriber mocap_pos_sub;
        ros::Subscriber mocap_vel_sub;
        ros::Subscriber ugv_cmd_sub;
        ros::Subscriber battery_sub;

        // 发布话题
        ros::Publisher agent_cmd_vel_pub;
        ros::Publisher led_pub;
        ros::Publisher agent_state_pub;
        ros::Publisher ugv_mesh_pub;
        ros::Publisher ugv_trajectory_pub;
        ros::Publisher text_info_pub;
        ros::Publisher goal_point_pub;

        ros::Timer timer_state_pub;
        ros::Timer timer_rivz;
        ros::Timer timer_debug;

        void mocap_pos_cb(const geometry_msgs::PoseStampedConstPtr& msg);
        void mocap_vel_cb(const geometry_msgs::TwistStampedConstPtr& msg);
        void agnet_cmd_cb(const sunray_msgs::agent_cmd::ConstPtr& msg);
        void battery_cb(const std_msgs::Float32ConstPtr& msg);
        void timercb_state(const ros::TimerEvent &e);
        void timercb_rviz(const ros::TimerEvent &e);
        void timercb_debug(const ros::TimerEvent &e);
        void printf_param();
        void set_desired_position();
        void check_geo_fence();
        geometry_msgs::Twist enu_to_body(geometry_msgs::Twist enu_cmd);
        void pos_control(geometry_msgs::Point pos_ref, double yaw_ref);
        float constrain_function(float data, float Max, float Min);
        Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q);
        double get_yaw_error(double yaw_ref, double yaw_now);

        void rotation_yaw(double yaw_angle, float body_frame[2], float enu_frame[2]);
        void setup_led();
        void setup_color();
};
#endif