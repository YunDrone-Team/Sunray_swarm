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
#define ODOM_TIMEOUT 0.35   

struct PIDController 
{
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
};

class RMTT_CONTROL
{
    public:
        // 构造函数
        RMTT_CONTROL():tf_buffer(),tf_listener(tf_buffer){};
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
        // 是否为仿真
        bool is_simulation;
        // 智能体固定的飞行高度 - 通过参数配置
        float agent_height;
        // 智能体位置来源（1：代表动捕、2代表地图） - 通过参数配置
        int pose_source;
        // RMTT上方LED灯颜色(同样也是RVIZ仿真中的MESH颜色)
        std_msgs::ColorRGBA led_color;
        // RMTT上方mled字符 - 通过参数配置
        std_msgs::String mled_text;
        // 是否打印 - 通过参数配置
        bool flag_printf;

        // 地面站接管，true代表地面站接管
        bool gs_control{false};
        // 定位地图
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener;

        // 悬停控制参数 - 通过参数配置
        struct control_param
        {
            float max_vel_xy;
            float max_vel_z;
            float max_vel_yaw;
            float deadzone_vel_xy;
            float deadzone_vel_z;
            float deadzone_vel_yaw;
            PIDController pid_xy;
            PIDController pid_z;
            PIDController pid_yaw;

        };
        control_param rmtt_control_param;

        // 地理围栏 - 通过参数配置
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

        // 智能体当前状态
        sunray_swarm_msgs::agent_state agent_state;   
        // 智能体上一时刻状态        
        sunray_swarm_msgs::agent_state agent_state_last; 
        // 智能体获得上一帧定位数据的时间（用于检查定位数据获取是否超时）
        ros::Time get_odom_time{0};
        // 智能体获得上一帧电量数据的时间（用于检查driver数据获取是否超时）
        ros::Time get_battery_time{0};

        // 智能体当前控制指令
        sunray_swarm_msgs::agent_cmd current_agent_cmd;
        // 智能体当前期望位置+偏航角（来自外部控制指令赋值）    
        geometry_msgs::Point desired_position;
        double desired_yaw{0.0};
        // 智能体当前期望速度
        geometry_msgs::Twist desired_vel;

        // 以下为辅助变量
        std_msgs::Empty takeoff;
        std_msgs::Empty land;
        std_msgs::String text_info;

        // 智能体当前期望位置+偏航角（来自外部控制指令赋值）    
        geometry_msgs::Point home_position;
        double home_yaw{0.0};
        // 轨迹容器,用于rviz显示
        vector<geometry_msgs::PoseStamped> pos_vector;    
        
        // 订阅话题
        ros::Subscriber mocap_pos_sub;
        ros::Subscriber mocap_vel_sub;
        ros::Subscriber odom_sub;
        ros::Subscriber agent_cmd_sub;
        ros::Subscriber agent_gs_cmd_sub;
        ros::Subscriber battery_sub;

        // 发布话题
        ros::Publisher agent_cmd_vel_pub;
        ros::Publisher takeoff_pub;
        ros::Publisher land_pub;
        ros::Publisher led_pub;
        ros::Publisher mled_pub;
        ros::Publisher agent_state_pub;
        ros::Publisher rmtt_mesh_pub;
        ros::Publisher rmtt_trajectory_pub;
        ros::Publisher text_info_pub;
        ros::Publisher goal_point_pub;
        ros::Publisher vel_rviz_pub;

        // 服务
        ros::ServiceClient set_downvision;

        // 定时器
        ros::Timer timer_state_pub;
        ros::Timer timer_rivz;
        ros::Timer timer_debug;
        ros::Timer timer_get_map_pose;

        // 内部函数
        void mocap_pos_cb(const geometry_msgs::PoseStampedConstPtr& msg);
        void mocap_vel_cb(const geometry_msgs::TwistStampedConstPtr& msg);
        void odom_cb(const nav_msgs::OdometryConstPtr& msg);
        void agent_cmd_cb(const sunray_swarm_msgs::agent_cmd::ConstPtr& msg);
        void agent_gs_cmd_cb(const sunray_swarm_msgs::agent_cmd::ConstPtr& msg);
        void battery_cb(const std_msgs::Float32ConstPtr& msg);
        void orca_cmd_vel_cb(const geometry_msgs::TwistConstPtr& msg);
        void rmtt_orca_state_cb(const sunray_swarm_msgs::orca_stateConstPtr& msg);
        void timercb_state(const ros::TimerEvent &e);
        void timercb_rviz(const ros::TimerEvent &e);
        void timercb_debug(const ros::TimerEvent &e);
        void timercb_get_map_pose(const ros::TimerEvent &e);
        void printf_param();
        void set_desired_position();
        bool check_geo_fence();
        void orca_control();
        geometry_msgs::Twist pos_control(geometry_msgs::Point pos_ref, double yaw_ref);
        float constrain_function(float data, float Max, float Min);
        Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q);
        double get_yaw_error(double desired_yaw, double yaw_now);
        float pid_control(PIDController& pid, float setpoint, float current_value, float dt);
        geometry_msgs::Twist enu_to_body(geometry_msgs::Twist enu_cmd);
        void rotation_yaw(double yaw_angle, float body_frame[2], float enu_frame[2]);
        void setup_led();
        void setup_mled();
        void setup_rviz_color();
        void handle_cmd(const sunray_swarm_msgs::agent_cmd msg);
};
#endif