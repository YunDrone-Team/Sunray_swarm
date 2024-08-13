#include <ros/ros.h>
#include <signal.h>

#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"

#define MAX_AGENT_NUM 10   
using namespace std;

int agent_type;
int agent_num;
float agent_height;
string node_name;   
sunray_msgs::orca_cmd orca_cmd;
sunray_msgs::agent_cmd agent_cmd[MAX_AGENT_NUM];
sunray_msgs::orca_state orca_state[MAX_AGENT_NUM];
geometry_msgs::Point goal_N[MAX_AGENT_NUM];
geometry_msgs::Point goal_O[MAX_AGENT_NUM];
geometry_msgs::Point goal_K[MAX_AGENT_NUM];
geometry_msgs::Point goal_V[MAX_AGENT_NUM];

// 执行状态
enum FORMATION_STATE
{
    INIT = 0,               // 初始模式
    N = 1,              
    O = 2,     
    K = 3,    
    O2 = 4,        
    V = 5,
    RETURN_HOME = 6,
};
FORMATION_STATE formation_state;

ros::Publisher text_info_pub;
ros::Publisher orca_cmd_pub;
ros::Subscriber start_cmd_sub;
ros::Subscriber orca_state_sub[MAX_AGENT_NUM];
ros::Publisher orca_goal_pub[MAX_AGENT_NUM];

void mySigintHandler(int sig)
{
    ROS_INFO("[formation_nokov] exit...");
    ros::shutdown();
}
void rmtt_orca_state_cb(const sunray_msgs::orca_stateConstPtr& msg, int i)
{
    orca_state[i] = *msg;
}
void timercb_show(const ros::TimerEvent &e)
{
    
}
void start_cmd_cb(const std_msgs::BoolConstPtr& msg)
{
    formation_state = FORMATION_STATE::N;
}
void setup_show_goals();

void printf_params()
{
    cout << GREEN << "agent_type    : " << agent_type << "" << TAIL << endl;
    cout << GREEN << "agent_num     : " << agent_num << "" << TAIL << endl;
    cout << GREEN << "agent_height   : " << agent_height << "" << TAIL << endl;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "formation_nokov");
    ros::NodeHandle nh("~");
    ros::Rate rate(20.0);
    node_name = ros::this_node::getName();

    // 【参数】智能体类型
    nh.param<int>("agent_type", agent_type, 1);
    // 【参数】智能体编号
    nh.param<int>("agent_num", agent_num, 8);
    // 【参数】agent_height
    nh.param<float>("agent_height", agent_height, 0.0f);
    float formation_time = 5.0;

    printf_params();

    // 【发布】ORCA指令 本节点 ->  ORCA
    orca_cmd_pub = nh.advertise<sunray_msgs::orca_cmd>("/sunray_swarm/orca_cmd", 1);
    // 【发布】文字提示消息（回传至地面站显示）
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);
    // 【订阅】程序触发指令
    start_cmd_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/formation_nokov", 1, start_cmd_cb);

    string agent_name;

    for(int i = 0; i < agent_num; i++) 
    {
        agent_name = "/rmtt_" + std::to_string(i+1);
        // 【发布】无人机的目标点
        orca_goal_pub[i] = nh.advertise<geometry_msgs::Point>("/sunray_swarm" + agent_name + "/goal_point", 1);
        // 【订阅】无人机orca状态
		orca_state_sub[i] = nh.subscribe<sunray_msgs::orca_state>("/sunray_swarm" + agent_name + "/agent_orca_state", 1, boost::bind(&rmtt_orca_state_cb,_1,i));
    }

    ros::Timer timer_show = nh.createTimer(ros::Duration(3.0), timercb_show);

    setup_show_goals();

    formation_state = FORMATION_STATE::INIT;

    bool pub_goal_once = false;

    // 主循环
    while (ros::ok())
    {
        // 回调函数
        ros::spinOnce();

        switch(formation_state)
        {
            case FORMATION_STATE::INIT:

            break;

            case FORMATION_STATE::N:
                // N形队形
                if(!pub_goal_once)
                {
                    cout << BLUE << node_name << " ORCA: SET_HOME" << TAIL << endl;

                    orca_cmd.orca_cmd = sunray_msgs::orca_cmd::SET_HOME;
                    orca_cmd_pub.publish(orca_cmd);
                    sleep(0.5);

                    cout << BLUE << node_name << " Formation: N" << TAIL << endl;
                    for(int i = 0; i < agent_num; i++) 
                    {
                        orca_goal_pub[i].publish(goal_N[i]);
                        sleep(0.1);
                    }
                    pub_goal_once = true;
                    orca_state[0].arrived_all_goal = false;
                    sleep(1.0);
                }

                if(pub_goal_once && orca_state[0].arrived_all_goal)
                {
                    sleep(formation_time);
                    formation_state = FORMATION_STATE::O;
                    // set flag
                    pub_goal_once = false;
                }
                break;
            case FORMATION_STATE::O:
                // O形队形
                if(!pub_goal_once)
                {
                    cout << BLUE << node_name << " Formation: O" << TAIL << endl;
                    for(int i = 0; i < agent_num; i++) 
                    {
                        orca_goal_pub[i].publish(goal_O[i]);
                        sleep(0.1);
                    }
                    pub_goal_once = true;
                    orca_state[0].arrived_all_goal = false;
                    sleep(1.0);
                }

                if(pub_goal_once && orca_state[0].arrived_all_goal)
                {
                    sleep(formation_time);
                    formation_state = FORMATION_STATE::K;
                    // set flag
                    pub_goal_once = false;
                }
                break;
            case FORMATION_STATE::K:
                // K形队形
                if(!pub_goal_once)
                {
                    cout << BLUE << node_name << " Formation: K" << TAIL << endl;
                    for(int i = 0; i < agent_num; i++) 
                    {
                        orca_goal_pub[i].publish(goal_K[i]);
                        sleep(0.1);
                    }
                    pub_goal_once = true;
                    orca_state[0].arrived_all_goal = false;
                    sleep(1.0);
                }

                if(pub_goal_once && orca_state[0].arrived_all_goal)
                {
                    sleep(formation_time);
                    formation_state = FORMATION_STATE::O2;
                    // set flag
                    pub_goal_once = false;
                }
                break;
            case FORMATION_STATE::O2:
                // O形队形
                if(!pub_goal_once)
                {
                    cout << BLUE << node_name << " Formation: O" << TAIL << endl;
                    for(int i = 0; i < agent_num; i++) 
                    {
                        orca_goal_pub[i].publish(goal_O[i]);
                        sleep(0.1);
                    }
                    pub_goal_once = true;
                    orca_state[0].arrived_all_goal = false;
                    sleep(1.0);
                }

                if(pub_goal_once && orca_state[0].arrived_all_goal)
                {
                    sleep(formation_time);
                    formation_state = FORMATION_STATE::V;
                    // set flag
                    pub_goal_once = false;
                }
                break;
            case FORMATION_STATE::V:
                // V形队形
                if(!pub_goal_once)
                {
                    cout << BLUE << node_name << " Formation: V" << TAIL << endl;
                    for(int i = 0; i < agent_num; i++) 
                    {
                        orca_goal_pub[i].publish(goal_V[i]);
                        sleep(0.1);
                    }
                    pub_goal_once = true;
                    orca_state[0].arrived_all_goal = false;
                    sleep(1.0);
                }

                if(pub_goal_once && orca_state[0].arrived_all_goal)
                {
                    sleep(formation_time);
                    formation_state = FORMATION_STATE::RETURN_HOME;
                    // set flag
                    pub_goal_once = false;
                }
                break;
            case FORMATION_STATE::RETURN_HOME:
                // Return
                if(!pub_goal_once)
                {
                    cout << BLUE << node_name << " ORCA: RETURN_HOME" << TAIL << endl;

                    orca_cmd.orca_cmd = sunray_msgs::orca_cmd::RETURN_HOME;
                    orca_cmd_pub.publish(orca_cmd);
                    pub_goal_once = true;
                    orca_state[0].arrived_all_goal = false;
                    sleep(1.0);
                }

                if(pub_goal_once && orca_state[0].arrived_all_goal)
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
    goal_N[0].x = -1.0; goal_N[0].y = 1.5;   // 顶部左端
    goal_N[1].x = 1.2; goal_N[1].y = 1.5;  // 底部左端
    goal_N[2].x = 0.1; goal_N[2].y = 0.15;   // 中部交点
    goal_N[3].x = 1.2; goal_N[3].y = -1.3;  // 顶部右端
    goal_N[4].x = -1.0; goal_N[4].y = -1.3; // 底部右端
    goal_N[5].x = 3.0; goal_N[5].y = 4.15;   // 中部交点
    goal_N[6].x = 3.2; goal_N[6].y = -4.3;  // 顶部右端
    goal_N[7].x = -2.0; goal_N[7].y = -4.3; // 底部右端

    // 字母 O
    goal_O[0].x = 0.1;  goal_O[0].y = 1.5;  // 上部中点
    goal_O[1].x = 1.4;  goal_O[1].y = 0.5; // 下部中点
    goal_O[2].x = -0.8; goal_O[2].y = 0.0;  // 上部边点
    goal_O[3].x = 1.4; goal_O[3].y = -0.5; // 下部边点
    goal_O[4].x = 0.1;  goal_O[4].y = -1.5;  // 中心点
    goal_O[5].x = 3.0; goal_O[5].y = 4.15;   // 中部交点
    goal_O[6].x = 3.2; goal_O[6].y = -4.3;  // 顶部右端
    goal_O[7].x = -2.0; goal_O[7].y = -4.3; // 底部右端
    // 字母 K
    goal_K[0].x = 0.1; goal_K[0].y = 0.9;  // 上部交点
    goal_K[1].x = 1.2; goal_K[1].y = 0.9; // 下部交点
    goal_K[2].x = -0.8; goal_K[2].y = 0.9;  // 中部竖线
    goal_K[3].x = 1.2; goal_K[3].y = -0.9;  // 斜线上端
    goal_K[4].x = -0.8; goal_K[4].y = -0.9; // 斜线下端
    goal_K[5].x = 3.0; goal_K[5].y = 4.15;   // 中部交点
    goal_K[6].x = 3.2; goal_K[6].y = -4.3;  // 顶部右端
    goal_K[7].x = -2.0; goal_K[7].y = -4.3; // 底部右端
    // 字母 V
    goal_V[0].x = 0.1;  goal_V[0].y = 0.6;  // 左上点
    goal_V[1].x = 1.1;  goal_V[1].y = 1.1; // 右上点
    goal_V[2].x = -0.7;  goal_V[2].y = 0.0;  // 底部点
    goal_V[3].x = 1.1;  goal_V[3].y = -1.1;  // 左下点
    goal_V[4].x = 0.1;  goal_V[4].y = -0.6; // 右下点
    goal_V[5].x = 3.0; goal_V[5].y = 4.15;   // 中部交点
    goal_V[6].x = 3.2; goal_V[6].y = -4.3;  // 顶部右端
    goal_V[7].x = -2.0; goal_V[7].y = -4.3; // 底部右端
}