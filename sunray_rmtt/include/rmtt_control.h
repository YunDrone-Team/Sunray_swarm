#ifndef RMTT_CONTROL_H
#define RMTT_CONTROL_H

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <vector>

#include "printf_utils.h"
#include "ros_msg_utils.h"

using namespace std;

#define TRA_WINDOW 50        
#define MOCAP_TIMEOUT 0.35                 

class RMTT_CONTROL
{
    public:
        // 构造函数
        RMTT_CONTROL(){};
        // 初始化函数
        void init(ros::NodeHandle& nh);
        // 主循环函数
        void mainloop();

    private:
        string node_name;               // 节点名称
        string uav_name{""};            // 无人机名称
        string uav_ip;
        int uav_id;                     // 无人机编号
        bool flag_printf;
        bool sim_mode;
        ros::Time get_mocap_time{0};
        double desired_yaw;

        float rmtt_height;
        sunray_msgs::rmtt_state rmtt_state;           // RMTT状态
        sunray_msgs::rmtt_state rmtt_state_last;      // RMTT状态
        geometry_msgs::Point hold_position;

        geometry_msgs::Twist cmd_vel_hold;
        geometry_msgs::Twist cmd_vel_orca_body;
        geometry_msgs::Twist cmd_vel_orca_enu;
        sunray_msgs::rmtt_orca rmtt_orca_state;

        std_msgs::Empty takeoff;
        std_msgs::Empty land;

        std_msgs::ColorRGBA led_color;
        std_msgs::String mled_text;
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
        geo_fence rmtt_geo_fence;

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
        control_param rmtt_control_param;

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

        bool arrived_goal;         // 是否到达目标点
        bool arrived_all_goal = false;      // 是否到达目标点
        sunray_msgs::rmtt_orca rmtt_orca;
        bool all_odom_valid{false};
        bool all_connected{false};

        sunray_msgs::station_cmd current_station_cmd;

        geometry_msgs::PoseStamped target_pos;
        double target_yaw{0};
        bool track_mode;
        
        // 订阅话题
        ros::Subscriber mocap_pos_sub;
        ros::Subscriber mocap_vel_sub;
        ros::Subscriber station_cmd_sub;
        ros::Subscriber station_cmd_sub2;
        ros::Subscriber battery_sub;
        ros::Subscriber orca_cmd_sub;
        ros::Subscriber rmtt_orca_state_sub;
        ros::Subscriber mocap_target_pos_sub;

        // 发布话题
        ros::Publisher rmtt_cmd_pub;
        ros::Publisher takeoff_pub;
        ros::Publisher land_pub;
        ros::Publisher led_pub;
        ros::Publisher mled_pub;
        ros::Publisher rmtt_state_pub;
        ros::Publisher rmtt_mesh_pub;
        ros::Publisher rmtt_trajectory_pub;
        ros::Publisher text_info_pub;
        ros::Publisher rmtt_orca_state_pub;
        ros::Publisher goal_point_pub;
        // 服务
        ros::ServiceClient set_downvision;

        ros::Timer timer_state_pub;
        ros::Timer timer_trajectory_pub;
        ros::Timer timer_debug;

        void mocap_pos_cb(const geometry_msgs::PoseStampedConstPtr& msg);
        void mocap_vel_cb(const geometry_msgs::TwistStampedConstPtr& msg);
        void station_cmd_cb(const sunray_msgs::station_cmd::ConstPtr& msg);
        void station_cmd_cb2(const sunray_msgs::station_cmd::ConstPtr& msg);
        void battery_cb(const std_msgs::Float32ConstPtr& msg);
        void orca_cmd_vel_cb(const geometry_msgs::TwistConstPtr& msg);
        void rmtt_orca_state_cb(const sunray_msgs::rmtt_orcaConstPtr& msg);
        void timercb_state(const ros::TimerEvent &e);
        void timercb_trajectory(const ros::TimerEvent &e);
        void timercb_debug(const ros::TimerEvent &e);
        void printf_param();
        void set_hold_position();
        void check_geo_fence();
        void orca_control();
        void hold_control(geometry_msgs::Point desired_pos);
        float constrain_function(float data, float Max, float Min);
        Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q);
        double get_yaw_error(double desired_yaw, double yaw_now);
        void mocap_target_pos_cb(const geometry_msgs::PoseStampedConstPtr& msg);

        void rotation_yaw(double yaw_angle, float body_frame[2], float enu_frame[2]);
        void setup_led();
        void setup_mled();
        void setup_color();
};
#endif