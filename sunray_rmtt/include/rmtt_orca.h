#ifndef RMTT_ORCA_H
#define RMTT_ORCA_H

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <vector>

#include "printf_utils.h"
#include "ros_msg_utils.h"
#include "RVO.h"

using namespace std;

#define MAX_NUM 10

class RMTT_ORCA
{
    public:
        // 构造函数
        RMTT_ORCA(){};
        // 初始化函数
        void init(ros::NodeHandle& nh);
        // 主循环函数
        bool orca_run();
        bool start_flag{false};

    private:
        string node_name;               // 节点名称
        string uav_name{""};            // 无人机名称
        bool track_mode;
        int rmtt_num;
        float rmtt_height;
        RVO::RVOSimulator *sim = new RVO::RVOSimulator();;        //算法类
        geometry_msgs::Twist cmd_vel[MAX_NUM];
        std::vector<RVO::Vector2> goals;    // goal
        geometry_msgs::Point home_point[MAX_NUM];

        // 增加达到目标点打印
        std::vector<bool> goal_reached_printed;

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

        // ORCA算法参数
        struct orca_param
        {
            float neighborDist;
            float maxNeighbors;
            float timeHorizon;
            float timeHorizonObst;
            float radius;
            float maxSpeed;
            float time_step;
        };
        orca_param rmtt_orca_param;

        sunray_msgs::rmtt_state rmtt_state[MAX_NUM];
        geometry_msgs::Point rmtt_goal[MAX_NUM];
        bool arrived_goal[MAX_NUM];         // 是否到达目标点
        bool arrived_all_goal = false;      // 是否到达目标点
        sunray_msgs::rmtt_orca rmtt_orca_state[MAX_NUM];
        std_msgs::String text_info;
        sunray_msgs::station_cmd current_station_cmd;
        
        // 订阅话题
        ros::Subscriber rmtt_state_sub[MAX_NUM];
        ros::Subscriber rmtt_goal_sub[MAX_NUM];
        ros::Subscriber station_cmd_sub;
        ros::Subscriber mocap_target_pos_sub[MAX_NUM];

        // 发布话题
        ros::Publisher orca_cmd_pub[MAX_NUM];
        ros::Publisher rmtt_orca_state_pub[MAX_NUM];
        ros::Publisher text_info_pub;

        ros::Timer debug_timer;
        void station_cmd_cb(const sunray_msgs::station_cmd::ConstPtr& msg);

        void debugCb(const ros::TimerEvent &e);
        void rmtt_state_cb(const sunray_msgs::rmtt_state::ConstPtr& msg, int i);
        void rmtt_goal_cb(const geometry_msgs::Point::ConstPtr& msg, int i);
        void mocap_target_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg, int i);
        void setup_obstacles();
        void setup_agents();
        void setup_goals(int index);
        void setup_init_goals();
        void setup_scenario_1();
        void setup_scenario_2();
        void setup_scenario_3();
        void setup_scenario_4();
        void setup_scenario_5();
        bool reachedGoal(int i);
        void printf_param();
};
#endif