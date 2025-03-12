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

#define MAX_NUM 20

class ORCA
{
    public:
        // 构造函数
        ORCA(){};
        // 初始化函数
        void init(ros::NodeHandle& nh);
        // 主循环函数
        bool orca_run();
        // 算法是否启动
        bool start_flag{false};

    private:
        // 节点名称
        string node_name;  
        // 智能体类型 - 通过参数配置
        int agent_type;
        // 智能体数量 - 通过参数配置
        int agent_num;
        // 智能体的固定高度 - 通过参数配置
        float agent_height;
        // ORCA算法参数 - 通过参数配置
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

        struct agent_pose
        {
            float x;
            float y;
            float yaw;
        };

        // 智能体的home点
        agent_pose home_pose[MAX_NUM];

        // 智能体的目标点
        agent_pose goal_pose[MAX_NUM];

        // 智能体当前状态
        sunray_msgs::agent_state agent_state[MAX_NUM];
        // 智能体控制指令 - 待发布
        sunray_msgs::agent_cmd agent_cmd[MAX_NUM];
        // 智能体的home点（home_point.x和home_point.y代表位置，home_point.z代表偏航角）
        geometry_msgs::Point home_point[MAX_NUM];

        // ORCA算法类
        RVO::RVOSimulator *sim = new RVO::RVOSimulator();
        // 智能体ORCA算法状态
        sunray_msgs::orca_state agent_orca_state[MAX_NUM];
        // ORCA目标点        
        std::vector<RVO::Vector2> goals;  
        // 收到的外部目标点（agent_goal.x和agent_goal.y代表目标点位置，agent_goal.z代表期望偏航角）
        geometry_msgs::Point agent_goal[MAX_NUM];  



        // ORCA算法中每个智能体是否到达目标点
        bool arrived_goal[MAX_NUM];        
        // ORCA算法中所有智能体是否到达目标点
        bool arrived_all_goal = false;    
        // ORCA算法目标点颜色（用于RVIZ中区分）
        std_msgs::ColorRGBA goal_point_color[MAX_NUM];
        // 达到目标点打印状态量
        std::vector<bool> goal_reached_printed;

        // 以下为辅助变量
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
        // 定时器
        ros::Timer timer_debug;

        void orca_cmd_cb(const sunray_msgs::orca_cmd::ConstPtr& msg);
        void timercb_debug(const ros::TimerEvent &e);
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