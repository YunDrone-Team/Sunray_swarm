#include <ros/ros.h>
#include <signal.h>
#include <vector>
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "sunray_msgs/agent_cmd.h"
#include "math_utils.h"

#define MAX_AGENT_NUM 10   

using namespace std;

int agent_num;
float agent_height;
ros::Publisher agent_cmd_pub[MAX_AGENT_NUM];
ros::Publisher text_info_pub;
std::vector<geometry_msgs::Point> triangle_formation;
std::vector<geometry_msgs::Point> line_formation;

int agent_type; // 代理类型，用于区分无人机和无人车


enum FORMATION_STATE
{
    TRIANGLE = 0,
    LINE = 1,
};
FORMATION_STATE formation_state;

void mySigintHandler(int sig)
{
    ROS_INFO("[formation_control] exit...");
    ros::shutdown();
}

void printf_params()
{
    cout << "Agent number  : " << agent_num << endl;
    cout << "Agent height  : " << agent_height << endl;
}

void setup_formations()
{
    // 设置三角形队形
    triangle_formation.resize(agent_num);
    triangle_formation[0].x = 0.0; triangle_formation[0].y = 0.0;
    triangle_formation[1].x = 1.0; triangle_formation[1].y = 1.0;
    triangle_formation[2].x = -1.0; triangle_formation[2].y = 1.0;
    // 更多无人机可以根据需要设置

    // 设置一字型队形
    line_formation.resize(agent_num);
    for(int i = 0; i < agent_num; i++) 
    {
        line_formation[i].x = i * 1.0;
        line_formation[i].y = 0.0;
    }
}

void switch_formation(FORMATION_STATE state)
{
    sunray_msgs::agent_cmd agent_cmd[MAX_AGENT_NUM];
    std::vector<geometry_msgs::Point> formation;

    if (state == FORMATION_STATE::TRIANGLE)
    {
        formation = triangle_formation;
        ROS_INFO("Switching to TRIANGLE formation");
    }
    else if (state == FORMATION_STATE::LINE)
    {
        formation = line_formation;
        ROS_INFO("Switching to LINE formation");
    }

    for(int i = 0; i < agent_num; i++) 
    {
        agent_cmd[i].agent_id = i+1;
        agent_cmd[i].control_state = sunray_msgs::agent_cmd::POS_CONTROL;
        agent_cmd[i].desired_pos.x = formation[i].x;
        agent_cmd[i].desired_pos.y = formation[i].y;
        agent_cmd[i].desired_pos.z = agent_height;
        agent_cmd_pub[i].publish(agent_cmd[i]);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "formation_control");
    ros::NodeHandle nh("~");

    nh.param<int>("agent_num", agent_num, 3);  // 默认3台无人机/车
    nh.param<float>("agent_height", agent_height, 1.0f);  // 默认飞行高度1米

    printf_params();
    setup_formations();

    // 定义一个字符串变量，用于存储代理前缀
    string agent_prefix;
    switch(agent_type) {
        case sunray_msgs::agent_state::RMTT:
            agent_prefix = "rmtt_";
            break;
        case sunray_msgs::agent_state::TIANBOT:
            agent_prefix = "tianbot_";
            break;
        case sunray_msgs::agent_state::WHEELTEC:
            agent_prefix = "wheeltec_";
            break;
        case sunray_msgs::agent_state::SIKONG:
            agent_prefix = "sikong_";
            break;
        default:
            agent_prefix = "unknown_";
            break;
    }


    string agent_name;
    for(int i = 0; i < agent_num; i++) 
    {
        agent_name = "/" + agent_prefix + std::to_string(i+1);
        // 【发布】无人车控制指令
        agent_cmd_pub[i] = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 1);
    }
    // [订阅]触发条件
    // agent_cmd_pub = nh.advertise<std_msgs::Bool>("/sunray_swarm/formation_control", 1， start_cmd_cb);
    ros::Rate rate(10.0);
    formation_state = FORMATION_STATE::TRIANGLE;

    while (ros::ok())
    {
        // 在三角形和一字型队形之间切换
        switch_formation(formation_state);

        // 模拟在两种队形之间切换
        formation_state = (formation_state == FORMATION_STATE::TRIANGLE) ? FORMATION_STATE::LINE : FORMATION_STATE::TRIANGLE;

        ros::Duration(8.0).sleep();  // 每8秒切换一次
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}