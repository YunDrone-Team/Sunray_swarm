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
#define ODOM_TIMEOUT 0.35                 

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
        // 节点名称
        string node_name;     
        // 智能体类型 - RMTT
        int agent_type;
        // 智能体编号 - 通过参数配置
        int agent_id;                     
        // 智能体名称 = 智能体类型+ID号
        string agent_name;
        // 智能体IP - 通过参数配置
        string agent_ip;
        // 智能体的固定高度 - 通过参数配置
        float agent_height;
        // 智能体位置来源（1：代表动捕、2代表viobot odom） - 通过参数配置
        int pose_source;
        // 是否打印 - 通过参数配置
        bool flag_printf;
        // 悬停控制参数 - 通过参数配置
        struct control_param
        {
            float Kp_xy;
            float Kp_yaw;
            float max_vel_xy;
            float max_vel_yaw;
        };
        control_param ugv_control_param;
        // 地理围栏 - 通过参数配置
        struct geo_fence
        {
            float max_x;
            float min_x;
            float max_y;
            float min_y;
        };
        geo_fence ugv_geo_fence;

        // 智能体当前状态
        sunray_msgs::agent_state agent_state;   
        // 智能体上一时刻状态        
        sunray_msgs::agent_state agent_state_last; 
        // 智能体获得上一帧定位数据的时间（用于检查定位数据获取是否超时）
        ros::Time get_odom_time{0};
        // 智能体获得上一帧电量数据的时间（用于检查driver数据获取是否超时）
        ros::Time get_battery_time{0};
        // 智能体当前控制指令
        sunray_msgs::agent_cmd current_agent_cmd;
        // 智能体当前期望位置+偏航角（来自外部控制指令赋值）    
        geometry_msgs::Point desired_position;
        double desired_yaw{0.0};
        // 智能体当前期望速度
        geometry_msgs::Twist desired_vel;

        // 以下为辅助变量
        std_msgs::ColorRGBA led_color;
        std_msgs::String text_info;
        // 轨迹容器,用于rviz显示
        vector<geometry_msgs::PoseStamped> pos_vector;    

        // 订阅话题
        ros::Subscriber mocap_pos_sub;
        ros::Subscriber mocap_vel_sub;
        ros::Subscriber viobot_odom_sub;
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
        ros::Publisher vel_rviz_pub;
        
        // 定时器
        ros::Timer timer_state_pub;
        ros::Timer timer_rivz;
        ros::Timer timer_debug;
        ros::Timer timer_rivz2;

        void mocap_pos_cb(const geometry_msgs::PoseStampedConstPtr& msg);
        void mocap_vel_cb(const geometry_msgs::TwistStampedConstPtr& msg);
        void odom_cb(const nav_msgs::OdometryConstPtr& msg);
        void agnet_cmd_cb(const sunray_msgs::agent_cmd::ConstPtr& msg);
        void battery_cb(const std_msgs::Float32ConstPtr& msg);
        void timercb_state(const ros::TimerEvent &e);
        void timercb_rviz(const ros::TimerEvent &e);
        void timercb_debug(const ros::TimerEvent &e);
        void printf_param();
        void set_desired_position();
        bool check_geo_fence();
        geometry_msgs::Twist enu_to_body(geometry_msgs::Twist enu_cmd);
        void pos_control(geometry_msgs::Point pos_ref, double yaw_ref);
        float constrain_function(float data, float Max, float Min);
        Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q);
        double get_yaw_error(double yaw_ref, double yaw_now);

        void rotation_yaw(double yaw_angle, float body_frame[2], float enu_frame[2]);
        void setup_led();
        void setup_rviz_color();
};
#endif