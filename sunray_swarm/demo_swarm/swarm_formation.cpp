/***********************************************************************************
 *  文件名: swarm_formation.cpp                                                          
 *  作者: Yun Drone                                                                 
 *  描述: 集群demo：集群固定阵型变换 agent_num：1-6（其他数量需要重新设定阵型）
 *     1、读取阵型文件（阵型为提前预设）
 *     2、如果是无人机，请使用地面站一键起飞所有无人机（无人车集群忽略这一步）
 *     3、等待demo启动指令
 *     4、启动后根据阵型变化指令并发送到控制节点执行（POS_CONTROL模式）
 *     5、可通过demo_start_flag暂停或恢复阵型变换移动
 *     6、本程序没有设置自动降落，可以停止移动后，通过地面站降落
 ***********************************************************************************/

#include <ros/ros.h>
#include <signal.h>
#include "printf_utils.h"
#include "ros_msg_utils.h"

#define MAX_AGENT_NUM 10                                // 定义最大智能体数量
using namespace std;

int agent_type;                                         // 智能体类型
int agent_num;                                          // 智能体数量
float agent_height;                                     // 智能体高度
bool demo_start_flag = false;                           // 是否接收到开始命令
std_msgs::String text_info;                             // 打印消息

sunray_msgs::orca_cmd agent_orca_cmd;                   // ORCA指令
sunray_msgs::orca_state orca_state[MAX_AGENT_NUM];      // ORCA状态

geometry_msgs::Point fomation_line[MAX_AGENT_NUM];            
geometry_msgs::Point fomation_triangle[MAX_AGENT_NUM];           
geometry_msgs::Point fomation_square[MAX_AGENT_NUM];      
float formation_keep_time;                              // 阵型保持时间
// 执行状态
enum FORMATION_STATE
{
    INIT = 0,               // 初始模式
    LINE = 1,                  
    TRIANGLE = 2,                
    SQAURE = 3,               
    RETURN_HOME = 10,        // 返回起点
};
FORMATION_STATE formation_state;        // 当前队形状态

ros::Publisher orca_cmd_pub;                        // 发布ORCA指令
ros::Subscriber demo_start_flag_sub;                // 订阅开始命令
ros::Subscriber orca_state_sub[MAX_AGENT_NUM];      // 订阅ORCA状态
ros::Publisher orca_goal_pub[MAX_AGENT_NUM];        // 发布ORCA目标点
ros::Publisher text_info_pub;                       // 发布文字提示消息

void mySigintHandler(int sig)
{
    ROS_INFO("[swarm_formation] exit...");          // 打印退出信息
    ros::shutdown();                                // 关闭ROS
}
// 处理ORCA状态回调
void rmtt_orca_state_cb(const sunray_msgs::orca_stateConstPtr& msg, int i)
{
    // 更新指定智能体的ORCA状态
    orca_state[i] = *msg;
}

void demo_start_flag_cb(const std_msgs::Bool::ConstPtr &msg)
{
    demo_start_flag = msg->data;    

    if(demo_start_flag)
    {
        text_info.data = "Get demo start cmd";
        cout << GREEN << text_info.data << TAIL << endl;
        text_info_pub.publish(text_info);
    }else
    {
        text_info.data = "Get demo stop cmd";
        cout << GREEN << text_info.data << TAIL << endl;
        text_info_pub.publish(text_info);
    }
}
void setup_formation();

// 主函数
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "swarm_formation");
    // 创建节点句柄
    ros::NodeHandle nh("~");
    // 设置循环频率为10Hz
    ros::Rate rate(10.0);

    // 【参数】智能体类型 
    nh.param<int>("agent_type", agent_type, 1);
    // 【参数】智能体编号 
    nh.param<int>("agent_num", agent_num, 6);
    // 【参数】阵型保持时间
    nh.param<float>("formation_keep_time", formation_keep_time, 5.0);

    cout << GREEN << ros::this_node::getName() << " start." << TAIL << endl;
    cout << GREEN << "agent_num           : " << agent_num << "" << TAIL << endl;
    cout << GREEN << "formation_keep_time : " << formation_keep_time << "" << TAIL << endl;

    // 根据智能体类型设置名称前缀
    string agent_prefix;
    if (agent_type == sunray_msgs::agent_state::RMTT)
    {
        agent_prefix = "rmtt";
        agent_height = 1.0;
        cout << GREEN << "agent_type    : rmtt" << TAIL << endl;
    }
    else if (agent_type == sunray_msgs::agent_state::UGV)
    {
        agent_prefix = "ugv";
        agent_height = 0.1;
        cout << GREEN << "agent_type    : ugv" << TAIL << endl;
    }

    //【订阅】触发指令 外部 -> 本节点 
    demo_start_flag_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/demo/swarm_formation", 1, demo_start_flag_cb);
    //【发布】ORCA算法指令 本节点 -> ORCA算法节点
    orca_cmd_pub = nh.advertise<sunray_msgs::orca_cmd>("/sunray_swarm/" + agent_prefix + "/orca_cmd", 1);
    //【发布】文字提示消息  本节点 -> 地面站
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);

    for (int i = 0; i < agent_num; i++) 
    {
        string agent_name = agent_prefix + "_" + std::to_string(i + 1);
        //【发布】每个智能体的ORCA算法目标点 本节点 -> ORCA算法节点
        orca_goal_pub[i] = nh.advertise<geometry_msgs::Point>("/sunray_swarm/" + agent_name + "/goal_point", 1);
        //【订阅】每个智能体的ORCA算法状态 本节点 -> ORCA算法节点
        orca_state_sub[i] = nh.subscribe<sunray_msgs::orca_state>("/sunray_swarm/" + agent_name + "/agent_orca_state", 1, boost::bind(&rmtt_orca_state_cb, _1, i));
    }

    // 设置阵型
    setup_formation();
    // 初始化队形状态
    formation_state = FORMATION_STATE::INIT;
    // 发布目标点的标志
    bool pub_formation = false;

    sleep(5.0);
    // 设置ORCA算法HOME点，并启动ORCA算法
    agent_orca_cmd.header.stamp = ros::Time::now();
    agent_orca_cmd.header.frame_id = "world";
    agent_orca_cmd.cmd_source = ros::this_node::getName();
    agent_orca_cmd.orca_cmd = sunray_msgs::orca_cmd::SET_HOME;
    orca_cmd_pub.publish(agent_orca_cmd);

    cout << GREEN << "start orca..." << TAIL << endl;
    sleep(3.0);

    // 主循环
    while (ros::ok())
    {
        // 等待demo启动
        if(!demo_start_flag)
        {
            // 处理一次回调函数
            ros::spinOnce();
            // sleep
            rate.sleep();
            continue;
        }
   
        switch(formation_state)
        {
            case FORMATION_STATE::INIT:
                formation_state = FORMATION_STATE::LINE; 
            break;

            case FORMATION_STATE::LINE:
                if(!pub_formation)
                {
                    cout << BLUE <<  " Formation: fomation_line" << TAIL << endl;
                    for(int i = 0; i < agent_num; i++) 
                    {
                        // 通过ORCA目标点发布队形目标点
                        orca_goal_pub[i].publish(fomation_line[i]); 
                        // 延迟   
                        sleep(0.1);                             
                    }
                    // pub_formation设置为true
                    pub_formation = true;  
                    // 重置到达目标状态                     
                    orca_state[0].arrived_all_goal = false;     
                    sleep(1.0);                            
                }
                // 检查所有智能体是否到达目标
                if(pub_formation && orca_state[0].arrived_all_goal)
                {
                    // 等待队形时间
                    sleep(formation_keep_time);  
                    // 切换到下一个队形                    
                    formation_state = FORMATION_STATE::TRIANGLE;  
                    // 重置标志    
                    pub_formation = false;                      
                }
                break;

            case FORMATION_STATE::TRIANGLE:
                if(!pub_formation)
                {
                    cout << BLUE <<  " Formation: fomation_triangle" << TAIL << endl;
                    for(int i = 0; i < agent_num; i++) 
                    {
                        // 通过ORCA目标点发布队形目标点
                        orca_goal_pub[i].publish(fomation_triangle[i]); 
                        // 延迟   
                        sleep(0.1);                             
                    }
                    // pub_formation设置为true
                    pub_formation = true;  
                    // 重置到达目标状态                     
                    orca_state[0].arrived_all_goal = false;     
                    sleep(1.0);                            
                }
                // 检查所有智能体是否到达目标
                if(pub_formation && orca_state[0].arrived_all_goal)
                {
                    // 等待队形时间
                    sleep(formation_keep_time);  
                    // 切换到下一个队形                    
                    formation_state = FORMATION_STATE::SQAURE;  
                    // 重置标志    
                    pub_formation = false;                      
                }
                break;

            case FORMATION_STATE::SQAURE:
                if(!pub_formation)
                {
                    cout << BLUE <<  " Formation: fomation_square" << TAIL << endl;
                    for(int i = 0; i < agent_num; i++) 
                    {
                        // 通过ORCA目标点发布队形目标点
                        orca_goal_pub[i].publish(fomation_square[i]); 
                        // 延迟   
                        sleep(0.1);                             
                    }
                    // pub_formation设置为true
                    pub_formation = true;  
                    // 重置到达目标状态                     
                    orca_state[0].arrived_all_goal = false;     
                    sleep(1.0);                            
                }
                // 检查所有智能体是否到达目标
                if(pub_formation && orca_state[0].arrived_all_goal)
                {
                    // 等待队形时间
                    sleep(formation_keep_time);  
                    // 切换到下一个队形                    
                    formation_state = FORMATION_STATE::LINE;  
                    // 重置标志    
                    pub_formation = false;                      
                }
                break;

            case FORMATION_STATE::RETURN_HOME:
                if(!pub_formation)
                {
                    cout << BLUE <<  " ORCA: RETURN_HOME" << TAIL << endl;
                    agent_orca_cmd.orca_cmd = sunray_msgs::orca_cmd::RETURN_HOME; // 设置ORCA命令为返回起点
                    orca_cmd_pub.publish(agent_orca_cmd);                         // 发布命令
                    pub_formation = true;                                   // 标志设置为已发布
                    orca_state[0].arrived_all_goal = false;                 // 重置到达目标状态
                    sleep(1.0);                                             // 延迟
                }
                // 检查所有智能体是否到达目标
                if(pub_formation && orca_state[0].arrived_all_goal)
                {
                    return 0;
                }
                break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;

}

void setup_formation()
{
    // 一字阵型
    // 1号智能体
    fomation_line[0].x = 0.0; 
    fomation_line[0].y = -0.5;
    fomation_line[0].z = -0.0;
    // 2号智能体
    fomation_line[1].x = 0.0; 
    fomation_line[1].y = 0.5;
    fomation_line[1].z = -0.0;
    // 3号智能体
    fomation_line[2].x = 0.0; 
    fomation_line[2].y = -1.5;
    fomation_line[2].z = -0.0;
    // 4号智能体
    fomation_line[3].x = 0.0; 
    fomation_line[3].y = 1.5;
    fomation_line[3].z = -0.0;
    // 5号智能体
    fomation_line[4].x = 0.0; 
    fomation_line[4].y = -2.5;
    fomation_line[4].z = -0.0;
    // 6号智能体
    fomation_line[5].x = 0.0; 
    fomation_line[5].y = 2.5;
    fomation_line[5].z = -0.0;

    // 三角阵型
    // 1号智能体
    fomation_triangle[0].x = 1.0; 
    fomation_triangle[0].y = -0.5;
    fomation_triangle[0].z = -0.0;
    // 2号智能体
    fomation_triangle[1].x = 1.0; 
    fomation_triangle[1].y = 0.5;
    fomation_triangle[1].z = -0.0;
    // 3号智能体
    fomation_triangle[2].x = 0.0; 
    fomation_triangle[2].y = -1.5;
    fomation_triangle[2].z = -0.0;
    // 4号智能体
    fomation_triangle[3].x = 0.0; 
    fomation_triangle[3].y = 1.5;
    fomation_triangle[3].z = -0.0;
    // 5号智能体
    fomation_triangle[4].x = -1.0; 
    fomation_triangle[4].y = -2.5;
    fomation_triangle[4].z = -0.0;
    // 6号智能体
    fomation_triangle[5].x = -1.0; 
    fomation_triangle[5].y = 2.5;
    fomation_triangle[5].z = -0.0;

    // 方形阵型
    // 1号智能体
    fomation_square[0].x = 1.0; 
    fomation_square[0].y = 1.5;
    fomation_square[0].z = 0.0;
    // 2号智能体
    fomation_square[1].x = 1.0; 
    fomation_square[1].y = 0.0;
    fomation_square[1].z = 0.0;
    // 3号智能体
    fomation_square[2].x = 1.0; 
    fomation_square[2].y = -1.5;
    fomation_square[2].z = 0.0;
    // 4号智能体
    fomation_square[3].x = -1.0; 
    fomation_square[3].y = 1.5;
    fomation_square[3].z = 0.0;
    // 5号智能体
    fomation_square[4].x = -1.0; 
    fomation_square[4].y = -0.0;
    fomation_square[4].z = 0.0;
    // 6号智能体
    fomation_square[5].x = -1.0; 
    fomation_square[5].y = -1.5;
    fomation_square[5].z = 0.0;
}
