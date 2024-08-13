#ifndef agent_ORCA_H
#define agent_ORCA_H

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Eigen>
#include <vector>

#include "printf_utils.h"
#include "ros_msg_utils.h"
#include "RVO.h"

using namespace std;

#define MAX_NUM 10

class ORCA
{
    public:
        // 构造函数
        ORCA(){};
        // 初始化函数
        void init(ros::NodeHandle& nh);
        // 主循环函数
        bool orca_run();
        bool start_flag{false};

    private:
        int agent_type;
        string node_name;               // 节点名称
        string agent_prefix;
        int agent_num;
        float agent_height;
        RVO::RVOSimulator *sim = new RVO::RVOSimulator();;        //算法类
        sunray_msgs::agent_cmd agent_cmd[MAX_NUM];
        std::vector<RVO::Vector2> goals;    // goal
        geometry_msgs::Point home_point[MAX_NUM];
        std_msgs::ColorRGBA led_color[MAX_NUM];

        // 增加达到目标点打印
        std::vector<bool> goal_reached_printed;

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
        orca_param orca_params;

        sunray_msgs::agent_state agent_state[MAX_NUM];
        geometry_msgs::Point agent_goal[MAX_NUM];
        bool arrived_goal[MAX_NUM];         // 是否到达目标点
        bool arrived_all_goal = false;      // 是否到达目标点
        sunray_msgs::orca_state agent_orca_state[MAX_NUM];
        std_msgs::String text_info;
        
        // 订阅话题
        ros::Subscriber agent_state_sub[MAX_NUM];
        ros::Subscriber agent_goal_sub[MAX_NUM];
        ros::Subscriber orca_cmd_sub;

        // 发布话题
        ros::Publisher agent_cmd_pub[MAX_NUM];
        ros::Publisher agent_orca_state_pub[MAX_NUM];
        ros::Publisher goal_point_pub[MAX_NUM];
        ros::Publisher text_info_pub;

        ros::Timer debug_timer;
        void orca_cmd_cb(const sunray_msgs::orca_cmd::ConstPtr& msg);
        void debugCb(const ros::TimerEvent &e);
        void agent_state_cb(const sunray_msgs::agent_state::ConstPtr& msg, int i);
        void agent_goal_cb(const geometry_msgs::Point::ConstPtr& msg, int i);
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
        void setup_color();
};
#endif