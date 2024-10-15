#include <ros/ros.h>
#include <signal.h>

#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"

#define MAX_AGENT_NUM 10                                // 定义最大智能体数量
using namespace std;

int agent_type;                                         // 智能体类型
int agent_num;                                          // 智能体数量
float agent_height;                                     // 智能体高度
string node_name;                                       // 节点名称
sunray_msgs::orca_cmd orca_cmd;                         // ORCA指令
sunray_msgs::agent_cmd agent_cmd[MAX_AGENT_NUM];        // 智能体控制指令
sunray_msgs::orca_state orca_state[MAX_AGENT_NUM];      // ORCA状态
geometry_msgs::Point goal_1[MAX_AGENT_NUM];             // 1形队形目标点
geometry_msgs::Point goal_2[MAX_AGENT_NUM];             // 2形队形目标点
geometry_msgs::Point goal_3[MAX_AGENT_NUM];             // 3形队形目标点
geometry_msgs::Point goal_4[MAX_AGENT_NUM];             // 4形队形目标点
geometry_msgs::Point goal_5[MAX_AGENT_NUM];             // 5形队形目标点
geometry_msgs::Point goal_6[MAX_AGENT_NUM];             // 6形队形目标点

float thres = 0.1;                                      //定义距离阈值
bool all_agents_reached = true;

// 执行状态
enum FORMATION_STATE
{
    INIT = 0,               // 初始模式
    GOAL_1 = 1,                  // 1形队形
    GOAL_2 = 2,                  // 2形队形
    GOAL_3 = 3,                  // 3形队形
    GOAL_4 = 4,                 // 4形队形
    GOAL_5 = 5,                 // 5形队形
    GOAL_6 = 6,                 // 6形队形
    RETURN_HOME = 6,        // 返回起点
};
FORMATION_STATE formation_state;        // 当前队形状态

ros::Publisher text_info_pub;                       // 发布文字提示消息
ros::Publisher orca_cmd_pub;                        // 发布ORCA指令
ros::Subscriber start_cmd_sub;                      // 订阅启动指令
ros::Subscriber orca_state_sub[MAX_AGENT_NUM];      // 订阅ORCA状态
ros::Publisher orca_goal_pub[MAX_AGENT_NUM];        // 发布ORCA目标点
ros::Publisher agent_cmd_pub[MAX_AGENT_NUM];  // 新增发布起飞和降落命令的发布者
bool agent_reached_goal[MAX_AGENT_NUM] = {false};  // 每个无人机的到达状态

// 信号处理函数
void mySigintHandler(int sig)
{
    ROS_INFO("[formation_nokov] exit...");          // 打印退出信息
    ros::shutdown();                                // 关闭ROS
}
// 处理ORCA状态回调
void rmtt_orca_state_cb(const sunray_msgs::orca_stateConstPtr& msg, int i)
{
    // 更新指定智能体的ORCA状态
    orca_state[i] = *msg;
}
// 定时器回调（用于显示目标点）
void timercb_show(const ros::TimerEvent &e)
{
    
}
// 启动指令回调
void start_cmd_cb(const std_msgs::BoolConstPtr& msg)
{
    formation_state = FORMATION_STATE::GOAL_1;// 设置队形状态为goal_1
    // rmtt_state = *msg;
}
// 函数声明，用于设置目标点
void setup_show_goals();
// 打印参数函数
void printf_params()
{
    cout << GREEN << "agent_type    : " << agent_type << "" << TAIL << endl;
    cout << GREEN << "agent_num     : " << agent_num << "" << TAIL << endl;
    cout << GREEN << "agent_height   : " << agent_height << "" << TAIL << endl;
}

// 起飞和降落的控制函数
void send_takeoff_and_land_cmd(int agent_id, bool is_takeoff)
{
    sunray_msgs::agent_cmd cmd;
    cmd.agent_id = agent_id;
    if (is_takeoff) 
    {
        cmd.control_state = sunray_msgs::agent_cmd::TAKEOFF;  // 起飞命令
        ROS_INFO("Agent %d takeoff command sent.", agent_id);
    } 
    else 
    {
        cmd.control_state = sunray_msgs::agent_cmd::LAND;  // 降落命令
        ROS_INFO("Agent %d landing command sent.", agent_id);
    }
    agent_cmd_pub[agent_id - 1].publish(cmd);  // 发布起飞/降落命令
}
// 主函数
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "formation_nokov");
    // 创建节点句柄
    ros::NodeHandle nh("~");
    // 设置循环频率为20Hz
    ros::Rate rate(20.0);
    // 获取节点名称
    node_name = ros::this_node::getName();

    // 【参数】智能体类型 获取智能体类型参数
    nh.param<int>("agent_type", agent_type, 0);
    // 【参数】智能体编号 获取智能体数量参数
    nh.param<int>("agent_num", agent_num, 1);
    // 【参数】agent_height 获取智能体高度参数
    nh.param<float>("agent_height", agent_height, 1.0f);
    // 定义队形持续时间
    float formation_time = 5.0;

    printf_params();                             // 打印参数信息
    string agent_name;                           // 存储智能体名称
    string agent_prefix;                         // 存储智能体前缀

    // 根据智能体类型设置前缀
    if(agent_type == sunray_msgs::agent_state::RMTT)
    {
        agent_prefix = "rmtt_";
    }else if(agent_type == sunray_msgs::agent_state::UGV)
    {
        agent_prefix = "ugv_";
    }else if(agent_type == sunray_msgs::agent_state::SIKONG)
    {
        agent_prefix = "sikong_";
    }else
    {
        agent_prefix = "unkonown_";
    }

    // 【发布】ORCA指令 本节点 ->  ORCA
    orca_cmd_pub = nh.advertise<sunray_msgs::orca_cmd>("/sunray_swarm/" + agent_prefix + "/orca_cmd", 1);
    // 【发布】文字提示消息（回传至地面站显示）
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);
    // 【订阅】程序触发指令
    start_cmd_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/demo/swarm_formation_nokov", 1, start_cmd_cb);

    // 【订阅】无人机状态数据
//     rmtt_state_sub = nh.subscribe<sunray_msgs::agent_state>("/sunray_swarm" + agent_name + "/agent_state", 1, rmtt_state_cb);

    // 为每个智能体创建发布和订阅
    for(int i = 0; i < agent_num; i++) 
    {
        agent_name = "/" + agent_prefix + std::to_string(i+1);
        // 【发布】无人机的目标点
        orca_goal_pub[i] = nh.advertise<geometry_msgs::Point>("/sunray_swarm" + agent_name + "/goal_point", 1);
        // 【订阅】无人机orca状态
		orca_state_sub[i] = nh.subscribe<sunray_msgs::orca_state>("/sunray_swarm" + agent_name + "/agent_orca_state", 1, boost::bind(&rmtt_orca_state_cb,_1,i));
        // 【发布】起飞和降落命令
        agent_cmd_pub[i] = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 1);
    }
    // 创建定时器
    ros::Timer timer_show = nh.createTimer(ros::Duration(3.0), timercb_show);
    // 设置目标点
    setup_show_goals();
    // 初始化队形状态
    formation_state = FORMATION_STATE::INIT;
    // 发布目标点的标志
    bool pub_goal_once = false;

    // 主循环
    while (ros::ok())
    {
        // 回调函数
        ros::spinOnce();

        switch(formation_state)
        {
            case FORMATION_STATE::INIT:
            // 初始状态逻辑
            break;

            case FORMATION_STATE::GOAL_1:
                // N形队形
                if(!pub_goal_once)
                {
                    cout << BLUE << node_name << " V形队形ORCA: SET_HOME" << TAIL << endl;
                    // 设置ORCA命令为HOME
                    orca_cmd.orca_cmd = sunray_msgs::orca_cmd::SET_HOME;
                    // 发布命令
                    orca_cmd_pub.publish(orca_cmd);
                    // 设置延迟
                    sleep(0.5);

                    cout << BLUE << node_name << " Formation: N" << TAIL << endl;
                    for(int i = 0; i < agent_num; i++) 
                    {
                        // 先让无人机降落
                        send_takeoff_and_land_cmd(i + 1, false);
                        sleep(1.0);  // 模拟降落的过程

                        // 起飞并发布目标点
                        send_takeoff_and_land_cmd(i + 1, true);
                        sleep(1.0);  // 模拟起飞过程

                        orca_goal_pub[i].publish(goal_1[i]);    // 发布N形队形目标点
                        sleep(0.1);                             // 延迟
                    }
                    pub_goal_once = true;                       // 标志设置为已发布
                    // for (int i = 0; i < agent_num; i++)
                    // {
                    //     orca_state[0].arrived_all_goal = false;     // 重置到达目标状态
                    // }
                    
                    sleep(1.0);                                 // 延迟
                }
                // 逐个检测每个无人机是否到达目标点
                for (int i = 0; i < agent_num; i++)
                {
                    if (!agent_reached_goal[i])
                    {
                        // 如果无人机的当前位置接近目标点，则标记其已到达
                        if (fabs(orca_state[i].goal[0] - goal_1[i].x) < thres && fabs(orca_state[i].goal[1] - goal_1[i].y) < thres)
                        {
                            ROS_INFO("Agent %d reached the goal, performing landing and next steps...", i + 1);

                            // 无人机到达目标点后，执行后续的起飞和降落操作
                            send_takeoff_and_land_cmd(i + 1, false);  // 先降落
                            sleep(1.0);  // 模拟降落的过程

                            send_takeoff_and_land_cmd(i + 1, true);  // 再次起飞
                            sleep(1.0);  // 模拟起飞过程

                            agent_reached_goal[i] = true;  // 标志该无人机已到达并执行完流程
                        }
                    }
                }

                // 检查是否所有无人机都完成了任务
                for (int i = 0; i < agent_num; i++)
                {
                    if (!agent_reached_goal[i])
                    {
                        all_agents_reached = false;
                        break;  // 如果有无人机未完成任务，继续等待
                    }
                }

                // 如果所有无人机都完成了当前目标任务，切换到下一个状态
                if (all_agents_reached)
                {
                    cout<<"goal2 pub"<<endl;
                    sleep(formation_time);                      // 等待队形时间
                    formation_state = FORMATION_STATE::GOAL_2;  // 转换到O形队形
                    pub_goal_once = false;                      // 重置标志
                }
                break;
            case FORMATION_STATE::GOAL_2:
                // O形队形
                if(!pub_goal_once)
                {
                    cout << BLUE << node_name << " Formation: O" << TAIL << endl;
                    for(int i = 0; i < agent_num; i++) 
                    {
                        // 先让无人机降落
                        send_takeoff_and_land_cmd(i + 1, false);
                        sleep(1.0);  // 模拟降落的过程

                        // 起飞并发布目标点
                        send_takeoff_and_land_cmd(i + 1, true);
                        sleep(1.0);  // 模拟起飞过程
                        orca_goal_pub[i].publish(goal_2[i]);    // 发布O形队形目标点
                        sleep(0.1);                             // 延迟
                    }
                    pub_goal_once = true;                       // 标志设置为已发布
                    // orca_state[0].arrived_all_goal = false;     // 重置到达目标状态
                    sleep(1.0);                                 // 延迟
                }
                // 逐个检测每个无人机是否到达目标点并执行流程
                for (int i = 0; i < agent_num; i++)
                {
                    if (!agent_reached_goal[i])
                    {
                        if (fabs(orca_state[i].goal[0] - goal_2[i].x) < thres && fabs(orca_state[i].goal[1] - goal_2[i].y) < thres)
                        {
                            ROS_INFO("Agent %d reached the goal, performing landing and next steps...", i + 1);

                            send_takeoff_and_land_cmd(i + 1, false);
                            sleep(1.0);

                            send_takeoff_and_land_cmd(i + 1, true);
                            sleep(1.0);

                            agent_reached_goal[i] = true;
                        }
                    }
                }

                // 检查是否所有无人机都完成了任务
                all_agents_reached = true;
                for (int i = 0; i < agent_num; i++)
                {
                    if (!agent_reached_goal[i])
                    {
                        all_agents_reached = false;
                        break;
                    }
                }

                if (all_agents_reached)
                {
                    sleep(formation_time);
                    formation_state = FORMATION_STATE::GOAL_3;  // 转换到K形队形
                    pub_goal_once = false;
                }
                break;
            case FORMATION_STATE::GOAL_3:
                // K形队形
                if(!pub_goal_once)
                {
                    cout << BLUE << node_name << " Formation: K" << TAIL << endl;
                    for(int i = 0; i < agent_num; i++) 
                    {
                        // 先让无人机降落
                        send_takeoff_and_land_cmd(i + 1, false);
                        sleep(1.0);  // 模拟降落的过程

                        // 起飞并发布目标点
                        send_takeoff_and_land_cmd(i + 1, true);
                        sleep(1.0);  // 模拟起飞过程
                        orca_goal_pub[i].publish(goal_3[i]);    // 发布K形队形目标点
                        sleep(0.1);                             // 延迟
                    }
                    pub_goal_once = true;                       // 标志设置为已发布
                    // orca_state[0].arrived_all_goal = false;     // 重置到达目标状态
                    sleep(1.0);                                 // 延迟
                }
                // 检查所有智能体是否到达目标
                for (int i = 0; i < agent_num; i++)
                {
                    if (!agent_reached_goal[i])
                    {
                        if (fabs(orca_state[i].goal[0] - goal_3[i].x) < thres && fabs(orca_state[i].goal[1] - goal_3[i].y) < thres)
                        {
                            ROS_INFO("Agent %d reached the goal, performing landing and next steps...", i + 1);

                            send_takeoff_and_land_cmd(i + 1, false);
                            sleep(1.0);

                            send_takeoff_and_land_cmd(i + 1, true);
                            sleep(1.0);

                            agent_reached_goal[i] = true;
                        }
                    }
                }

                // 检查是否所有无人机都完成了任务
                all_agents_reached = true;
                for (int i = 0; i < agent_num; i++)
                {
                    if (!agent_reached_goal[i])
                    {
                        all_agents_reached = false;
                        break;
                    }
                }

                if (all_agents_reached)
                {
                    sleep(formation_time);                      // 等待队形时间
                    formation_state = FORMATION_STATE::GOAL_4;      // 转换到O2形队形
                    pub_goal_once = false;                      // 重置标志
                }
                break;

            case FORMATION_STATE::GOAL_4:
                // O形队形
                if(!pub_goal_once)
                {
                    cout << BLUE << node_name << " Formation: O" << TAIL << endl;
                    for(int i = 0; i < agent_num; i++) 
                    {
                        // 先让无人机降落
                        send_takeoff_and_land_cmd(i + 1, false);
                        sleep(1.0);  // 模拟降落的过程

                        // 起飞并发布目标点
                        send_takeoff_and_land_cmd(i + 1, true);
                        sleep(1.0);  // 模拟起飞过程
                        orca_goal_pub[i].publish(goal_2[i]);    // 发布O2形队形目标点
                        sleep(0.1);                             // 延迟
                    }
                    pub_goal_once = true;                       // 标志设置为已发布
                    // orca_state[0].arrived_all_goal = false;     // 重置到达目标状态
                    sleep(1.0);                                 // 延迟
                }
                // 检查所有智能体是否到达目标
                for (int i = 0; i < agent_num; i++)
                {
                    if (!agent_reached_goal[i])
                    {
                        if (fabs(orca_state[i].goal[0] - goal_2[i].x) < thres && fabs(orca_state[i].goal[1] - goal_2[i].y) < thres)
                        {
                            ROS_INFO("Agent %d reached the goal, performing landing and next steps...", i + 1);

                            send_takeoff_and_land_cmd(i + 1, false);
                            sleep(1.0);

                            send_takeoff_and_land_cmd(i + 1, true);
                            sleep(1.0);

                            agent_reached_goal[i] = true;
                        }
                    }
                }

                // 检查是否所有无人机都完成了任务
                all_agents_reached = true;
                for (int i = 0; i < agent_num; i++)
                {
                    if (!agent_reached_goal[i])
                    {
                        all_agents_reached = false;
                        break;
                    }
                }

                if (all_agents_reached)
                {
                    sleep(formation_time);                      // 等待队形时间
                    formation_state = FORMATION_STATE::GOAL_5;       // 转换到V形队形
                    pub_goal_once = false;                      // 重置标志
                }
                break;
            case FORMATION_STATE::GOAL_5:
                // V形队形
                if(!pub_goal_once)
                {
                    cout << BLUE << node_name << " Formation: V" << TAIL << endl;
                    for(int i = 0; i < agent_num; i++) 
                    {
                        // 先让无人机降落
                        send_takeoff_and_land_cmd(i + 1, false);
                        sleep(1.0);  // 模拟降落的过程

                        // 起飞并发布目标点
                        send_takeoff_and_land_cmd(i + 1, true);
                        sleep(1.0);  // 模拟起飞过程
                        orca_goal_pub[i].publish(goal_5[i]);    // 发布V形队形目标点
                        sleep(0.1);                             // 延迟
                    }
                    pub_goal_once = true;                       // 标志设置为已发布
                    // orca_state[0].arrived_all_goal = false;     // 重置到达目标状态
                    sleep(1.0);                                 // 延迟
                }   
                // 检查所有智能体是否到达目标

                for (int i = 0; i < agent_num; i++)
                {
                    if (!agent_reached_goal[i])
                    {
                        if (fabs(orca_state[i].goal[0] - goal_5[i].x) < thres && fabs(orca_state[i].goal[1] - goal_5[i].y) < thres)
                        {
                            ROS_INFO("Agent %d reached the goal, performing landing and next steps...", i + 1);

                            send_takeoff_and_land_cmd(i + 1, false);
                            sleep(1.0);

                            send_takeoff_and_land_cmd(i + 1, true);
                            sleep(1.0);

                            agent_reached_goal[i] = true;
                        }
                    }
                }

                // 检查是否所有无人机都完成了任务
                all_agents_reached = true;
                for (int i = 0; i < agent_num; i++)
                {
                    if (!agent_reached_goal[i])
                    {
                        all_agents_reached = false;
                        break;
                    }
                }

                if (all_agents_reached)
                {
                    sleep(formation_time);                      // 等待队形时间
                    formation_state = FORMATION_STATE::RETURN_HOME; // 转换到返回起点状态
                    pub_goal_once = false;                      // 重置标志
                }
                break;
            case FORMATION_STATE::RETURN_HOME:
                // Return
                if(!pub_goal_once)
                {
                    cout << BLUE << node_name << " ORCA: RETURN_HOME" << TAIL << endl;

                    orca_cmd.orca_cmd = sunray_msgs::orca_cmd::RETURN_HOME; // 设置ORCA命令为返回起点
                    orca_cmd_pub.publish(orca_cmd);                         // 发布命令
                    pub_goal_once = true;                                   // 标志设置为已发布
                    orca_state[0].arrived_all_goal = false;                 // 重置到达目标状态
                    sleep(1.0);                                             // 延迟
                }
                // 检查所有智能体是否到达目标
                if(pub_goal_once && orca_state[0].arrived_all_goal)
                {
                    formation_state = FORMATION_STATE::INIT;
                    pub_goal_once = false;
                    for(int i = 0; i < agent_num; i++) 
                    {
                        // 先让无人机降落
                        send_takeoff_and_land_cmd(i + 1, false);
                        sleep(1.0);  // 模拟降落的过程
                    }

                    // return 0;
                }
                break;
        }
        ros::spinOnce();
        // sleep
        rate.sleep();
    }

    return 0;

}

void setup_show_goals()
{
    // 字母 N
    goal_1[0].x = -1.0; goal_1[0].y = 1.5;   // 顶部左端
    goal_1[1].x = 1.2; goal_1[1].y = 1.5;  // 底部左端
    goal_1[2].x = 0.1; goal_1[2].y = 0.15;   // 中部交点
    goal_1[3].x = 1.2; goal_1[3].y = -1.3;  // 顶部右端
    goal_1[4].x = -1.0; goal_1[4].y = -1.3; // 底部右端
    goal_1[5].x = 3.0; goal_1[5].y = 4.15;   // 中部交点
    goal_1[6].x = 3.2; goal_1[6].y = -4.3;  // 顶部右端
    goal_1[7].x = -2.0; goal_1[7].y = -4.3; // 底部右端

    // 字母 O
    goal_2[0].x = 0.1;  goal_2[0].y = 1.5;  // 上部中点
    goal_2[1].x = 1.4;  goal_2[1].y = 0.5; // 下部中点
    goal_2[2].x = -0.8; goal_2[2].y = 0.0;  // 上部边点
    goal_2[3].x = 1.4; goal_2[3].y = -0.5; // 下部边点
    goal_2[4].x = 0.1;  goal_2[4].y = -1.5;  // 中心点
    goal_2[5].x = 3.0; goal_2[5].y = 4.15;   // 中部交点
    goal_2[6].x = 3.2; goal_2[6].y = -4.3;  // 顶部右端
    goal_2[7].x = -2.0; goal_2[7].y = -4.3; // 底部右端
    // 字母 K
    goal_3[0].x = 0.1; goal_3[0].y = 0.9;  // 上部交点
    goal_3[1].x = 1.2; goal_3[1].y = 0.9; // 下部交点
    goal_3[2].x = -0.8; goal_3[2].y = 0.9;  // 中部竖线
    goal_3[3].x = 1.2; goal_3[3].y = -0.9;  // 斜线上端
    goal_3[4].x = -0.8; goal_3[4].y = -0.9; // 斜线下端
    goal_3[5].x = 3.0; goal_3[5].y = 4.15;   // 中部交点
    goal_3[6].x = 3.2; goal_3[6].y = -4.3;  // 顶部右端
    goal_3[7].x = -2.0; goal_3[7].y = -4.3; // 底部右端
    // 字母 V
    goal_5[0].x = 0.1;  goal_5[0].y = 0.6;  // 左上点
    goal_5[1].x = 1.1;  goal_5[1].y = 1.1; // 右上点
    goal_5[2].x = -0.7;  goal_5[2].y = 0.0;  // 底部点
    goal_5[3].x = 1.1;  goal_5[3].y = -1.1;  // 左下点
    goal_5[4].x = 0.1;  goal_5[4].y = -0.6; // 右下点
    goal_5[5].x = 3.0; goal_5[5].y = 4.15;   // 中部交点
    goal_5[6].x = 3.2; goal_5[6].y = -4.3;  // 顶部右端
    goal_5[7].x = -2.0; goal_5[7].y = -4.3; // 底部右端
}
