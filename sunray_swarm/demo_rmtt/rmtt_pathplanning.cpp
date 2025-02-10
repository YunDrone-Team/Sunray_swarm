#include <ros/ros.h>
#include <signal.h>

#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"

#define MAX_AGENT_NUM 10
using namespace std;

int agent_type;                                    // 智能体类型
int agent_num;                                     // 智能体数量
float agent_height;                                // 智能体高度
string node_name;                                  // 节点名称
sunray_msgs::orca_cmd orca_cmd;                    // ORCA指令
sunray_msgs::agent_cmd agent_cmd[MAX_AGENT_NUM];   // 智能体指令
sunray_msgs::orca_state orca_state[MAX_AGENT_NUM]; // ORCA状态
geometry_msgs::Point goal_N[MAX_AGENT_NUM];        // N形目标点
geometry_msgs::Point goal_O[MAX_AGENT_NUM];        // O形目标点
geometry_msgs::Point goal_K[MAX_AGENT_NUM];        // K形目标点
geometry_msgs::Point goal_V[MAX_AGENT_NUM];        // V形目标点

// 执行状态
enum FORMATION_STATE
{
    INIT = 0,        // 初始模式
    N = 1,           // N形队形
    O = 2,           // O形队形
    K = 3,           // K形队形
    O2 = 4,          // 第二次O形队形
    V = 5,           // V形队形
    RETURN_HOME = 6, // 返回初始点
};
FORMATION_STATE formation_state; // 当前队形状态

ros::Publisher text_info_pub;                  // 文字提示消息发布器
ros::Publisher orca_cmd_pub;                   // ORCA指令发布器
ros::Subscriber start_cmd_sub;                 // 启动指令订阅器
ros::Subscriber orca_state_sub[MAX_AGENT_NUM]; // ORCA状态订阅器
ros::Publisher orca_goal_pub[MAX_AGENT_NUM];   // 智能体目标点发布器

// 信号处理函数
void mySigintHandler(int sig)
{
    ROS_INFO("[swarm_with_obstacles] exit...");
    ros::shutdown();
}
// ORCA状态回调函数
void rmtt_orca_state_cb(const sunray_msgs::orca_stateConstPtr &msg, int i)
{
    orca_state[i] = *msg; // 更新指定智能体的ORCA状态
}
// 定时器回调函数
void timercb_show(const ros::TimerEvent &e)
{
}
// 启动命令回调函数
void start_cmd_cb(const std_msgs::BoolConstPtr &msg)
{
    formation_state = FORMATION_STATE::N; // 更新队形状态为N形
}
// 目标点设置函数声明
void setup_show_goals();

// 打印参数
void printf_params()
{
    cout << GREEN << "agent_type    : " << agent_type << "" << TAIL << endl;
    cout << GREEN << "agent_num     : " << agent_num << "" << TAIL << endl;
    cout << GREEN << "agent_height   : " << agent_height << "" << TAIL << endl;
}

// 设置障碍物
void setup_obstacles()
{
    orca_cmd.cmd_source = "demo";
    orca_cmd.orca_cmd = sunray_msgs::orca_cmd::SETUP_OBS;
    // 障碍物示例：中心在原点，边长为2的正方体

    geometry_msgs::Point Point1, Point2, Point3, Point4;
    Point1.x = 0.7;
    Point1.y = 0.7;
    Point2.x = 0.7;
    Point2.y = 0.3;
    Point3.x = 0.3;
    Point3.y = 0.3;
    Point4.x = 0.3;
    Point4.y = 0.7;

    // 将顶点添加到障碍物指令中
    orca_cmd.obs_point.push_back(Point1);
    orca_cmd.obs_point.push_back(Point2);
    orca_cmd.obs_point.push_back(Point3);
    orca_cmd.obs_point.push_back(Point4);

    // 发布设置障碍物指令
    orca_cmd_pub.publish(orca_cmd);
    // 输出设置障碍物完成信息
    cout << GREEN << "setup_obstacles" << TAIL << endl;

    // 发布RVIZ
}
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "swarm_with_obstacles");
    // 创建节点句柄
    ros::NodeHandle nh("~");
    // 设置循环频率为20Hz
    ros::Rate rate(20.0);
    // 获取节点名称
    node_name = ros::this_node::getName();

    // 【参数】智能体类型
    nh.param<int>("agent_type", agent_type, 1);
    // 【参数】智能体编号
    nh.param<int>("agent_num", agent_num, 8);
    // 【参数】agent_height
    nh.param<float>("agent_height", agent_height, 0.0f);
    // 队形转换时间
    float formation_time = 5.0;

    printf_params();     // 打印参数
    string agent_name;   // 智能体名称
    string agent_prefix; // 智能体前缀

    // 根据智能体类型设置前缀
    if (agent_type == sunray_msgs::agent_state::RMTT)
    {
        agent_prefix = "rmtt_";
    }
    else if (agent_type == sunray_msgs::agent_state::UGV)
    {
        agent_prefix = "ugv_";
    }
    else if (agent_type == sunray_msgs::agent_state::SIKONG)
    {
        agent_prefix = "sikong_";
    }
    else
    {
        agent_prefix = "unkonown_";
    }

    // 【发布】ORCA指令 本节点 ->  ORCA
    orca_cmd_pub = nh.advertise<sunray_msgs::orca_cmd>("/sunray_swarm/" + agent_prefix + "/orca_cmd", 1);
    // 【发布】文字提示消息（回传至地面站显示）
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);
    // 【订阅】程序触发指令
    start_cmd_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/demo/rmtt_pathPlanning", 1, start_cmd_cb);

    // 为每个智能体设置发布和订阅
    for (int i = 0; i < agent_num; i++)
    {
        // 创建智能体名称
        agent_name = "/" + agent_prefix + std::to_string(i + 1);
        // 【发布】无人机的目标点
        orca_goal_pub[i] = nh.advertise<geometry_msgs::Point>("/sunray_swarm" + agent_name + "/goal_point", 1);
        // 【订阅】无人机orca状态
        orca_state_sub[i] = nh.subscribe<sunray_msgs::orca_state>("/sunray_swarm" + agent_name + "/agent_orca_state", 1, boost::bind(&rmtt_orca_state_cb, _1, i));
    }

    // 创建定时器
    ros::Timer timer_show = nh.createTimer(ros::Duration(3.0), timercb_show);

    sleep(1.0);         // 暂停1秒
    setup_obstacles();  // 设置障碍物
    setup_show_goals(); // 设置目标点

    // 初始化队形状态
    formation_state = FORMATION_STATE::INIT;
    // 发布目标点的标志
    bool pub_goal_once = false;

    // 主循环
    while (ros::ok())
    {
        // 回调函数
        ros::spinOnce();

        switch (formation_state)
        {
        case FORMATION_STATE::INIT:
            // 初始状态，等待启动命令
            break;

        case FORMATION_STATE::N:
            // N形队形
            if (!pub_goal_once)
            {
                cout << BLUE << node_name << " ORCA: SET_HOME" << TAIL << endl;

                orca_cmd.orca_cmd = sunray_msgs::orca_cmd::SET_HOME; // 设置HOME指令
                orca_cmd_pub.publish(orca_cmd);                      // 发布HOME指令
                sleep(0.5);                                          // 暂停0.5秒

                cout << BLUE << node_name << " Formation: N" << TAIL << endl;
                for (int i = 0; i < agent_num; i++)
                {
                    orca_goal_pub[i].publish(goal_N[i]); // 发布N形目标点
                    sleep(0.1);                          // 暂停0.1秒
                }
                pub_goal_once = true;                   // 设置目标点已发布
                orca_state[0].arrived_all_goal = false; // 初始化到达标志
                sleep(1.0);                             // 暂停1秒
            }

            if (pub_goal_once && orca_state[0].arrived_all_goal)
            {
                sleep(formation_time);
                formation_state = FORMATION_STATE::O; // 切换到O形队形
                pub_goal_once = false;                // 重置发布标志
            }
            break;
        case FORMATION_STATE::O:
            // O形队形
            if (!pub_goal_once)
            {
                cout << BLUE << node_name << " Formation: O" << TAIL << endl;
                for (int i = 0; i < agent_num; i++)
                {
                    orca_goal_pub[i].publish(goal_O[i]); // 发布O形目标点
                    sleep(0.1);                          // 暂停0.1秒
                }
                pub_goal_once = true;                   // 设置目标点已发布
                orca_state[0].arrived_all_goal = false; // 初始化到达标志
                sleep(1.0);                             // 暂停1秒
            }

            if (pub_goal_once && orca_state[0].arrived_all_goal)
            {
                sleep(formation_time);
                formation_state = FORMATION_STATE::K; // 切换到K形队形
                pub_goal_once = false;                // 重置发布标志
            }
            break;
        case FORMATION_STATE::K:
            // K形队形
            if (!pub_goal_once)
            {
                cout << BLUE << node_name << " Formation: K" << TAIL << endl;
                for (int i = 0; i < agent_num; i++)
                {
                    orca_goal_pub[i].publish(goal_K[i]); // 发布K形目标点
                    sleep(0.1);                          // 暂停0.1秒
                }
                pub_goal_once = true;
                orca_state[0].arrived_all_goal = false;
                sleep(1.0);
            }

            if (pub_goal_once && orca_state[0].arrived_all_goal)
            {
                sleep(formation_time);
                formation_state = FORMATION_STATE::O2; // 切换到第二次O形队形
                pub_goal_once = false;                 // 重置发布标志
            }
            break;
        case FORMATION_STATE::O2:
            // O形队形
            if (!pub_goal_once)
            {
                cout << BLUE << node_name << " Formation: O" << TAIL << endl;
                for (int i = 0; i < agent_num; i++)
                {
                    orca_goal_pub[i].publish(goal_O[i]); // 发布O形目标点
                    sleep(0.1);                          // 暂停0.1秒
                }
                pub_goal_once = true;                   // 设置目标点已发布
                orca_state[0].arrived_all_goal = false; // 初始化到达标志
                sleep(1.0);                             // 暂停1秒
            }

            if (pub_goal_once && orca_state[0].arrived_all_goal)
            {
                sleep(formation_time);
                formation_state = FORMATION_STATE::V; // 切换到V形队形
                pub_goal_once = false;                // 重置发布标志
            }
            break;
        case FORMATION_STATE::V:
            // V形队形
            if (!pub_goal_once)
            {
                cout << BLUE << node_name << " Formation: V" << TAIL << endl;
                for (int i = 0; i < agent_num; i++)
                {
                    orca_goal_pub[i].publish(goal_V[i]); // 发布V形目标点
                    sleep(0.1);                          // 暂停0.1秒
                }
                pub_goal_once = true;                   // 设置目标点已发布
                orca_state[0].arrived_all_goal = false; // 初始化到达标志
                sleep(1.0);                             // 暂停1秒
            }

            if (pub_goal_once && orca_state[0].arrived_all_goal)
            {
                sleep(formation_time);
                formation_state = FORMATION_STATE::RETURN_HOME; // 切换到返回家
                pub_goal_once = false;                          // 重置发布标志
            }
            break;
        case FORMATION_STATE::RETURN_HOME:
            // Return
            if (!pub_goal_once)
            {
                cout << BLUE << node_name << " ORCA: RETURN_HOME" << TAIL << endl;

                orca_cmd.orca_cmd = sunray_msgs::orca_cmd::RETURN_HOME;     // 设置ORCA命令为返回起点
                    orca_cmd_pub.publish(orca_cmd);                         // 发布命令
                    pub_goal_once = true;                                   // 标志设置为已发布
                    orca_state[0].arrived_all_goal = false;                 // 重置到达目标状态
                    sleep(1.0);                                             // 延迟
            }
            // 检查所有智能体是否到达目标点
            if (pub_goal_once && orca_state[0].arrived_all_goal)
            {
                return 0;
            }
            break;
        }

        // sleep
        rate.sleep();
    }

    return 0;
}

void setup_show_goals()
{

    // 字母 N
    goal_N[0].x = -1.0;
    goal_N[0].y = 1.5; // 顶部左端


    // 字母 O
    goal_O[0].x = 0.1;
    goal_O[0].y = 1.5; // 上部中点

    // 字母 K
    goal_K[0].x = 0.1;
    goal_K[0].y = 0.9; // 上部交点

    // 字母 V
    goal_V[0].x = 0.1;
    goal_V[0].y = 0.6; // 左上点


    cout << GREEN << "setup_show_goals" << TAIL << endl;
}