/***********************************************************************************
 *  文件名: swarm_circle.cpp                                                          
 *  作者: Yun Drone                                                                 
 *  描述: 集群demo：集群圆形轨迹移动（ORCA算法参与避障计算）agent_num：1-10均可
 *     1、从参数列表里面获取圆形轨迹参数
 *     2、如果是无人机，请使用地面站一键起飞所有无人机（无人车集群忽略这一步）
 *     3、等待demo启动指令
 *     4、启动后根据时间计算圆形轨迹位置并发送到控制节点执行（POS_CONTROL模式）
 *     5、可通过demo_start_flag暂停或恢复圆形轨迹
 *     6、本程序没有设置自动降落，可以停止圆形轨迹移动后，通过地面站降落
 ***********************************************************************************/

#include <ros/ros.h>
#include <signal.h>
#include "printf_utils.h"
#include "ros_msg_utils.h"

#define MAX_AGENT_NUM 100
using namespace std;

int agent_type;                        // 智能体类型
int agent_num;                         // 智能体数量
int agent_height;                      // 智能体固定的高度
bool demo_start_flag = false;          // 是否接收到开始命令

Eigen::Vector3f circle_center;         // 圆参数：圆心坐标
float circle_radius;                   // 圆参数：圆周轨迹的半径
float linear_vel;                      // 圆参数：线速度
float omega;                           // 圆参数：角速度
float direction;                       // 圆参数：方向，1或-1
float time_trajectory = 0.0;           // 圆参数：轨迹时间计数器

sunray_msgs::orca_cmd agent_orca_cmd;  // ORCA指令
std_msgs::String text_info;            // 打印消息
sunray_msgs::orca_state orca_state[MAX_AGENT_NUM];      // ORCA状态

ros::Subscriber demo_start_flag_sub;             // 订阅开始命令
ros::Subscriber orca_state_sub[MAX_AGENT_NUM];   // ORCA算法状态订阅
ros::Publisher orca_cmd_pub;                     // 发布ORCA指令
ros::Publisher orca_goal_pub[MAX_AGENT_NUM];     // ORCA算法目标点发布
ros::Publisher text_info_pub;                   // 发布信息到地面站

void mySigintHandler(int sig) 
{
    ROS_INFO("[swarm_circle] exit...");
    ros::shutdown();
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

// 主函数
int main(int argc, char **argv) 
{
    // 初始化ROS节点
    ros::init(argc, argv, "swarm_circle");
    // 创建节点句柄
    ros::NodeHandle nh("~");
    // 设置循环频率为10Hz
    ros::Rate rate(10.0);

    // 【参数】智能体类型 0代表RMTT，1代表UGV
    nh.param<int>("agent_type", agent_type, 0);
    // 【参数】智能体编号 智能体数量，默认为6
    nh.param<int>("agent_num", agent_num, 6);
    // 【参数】圆形轨迹参数：圆心X坐标
    nh.param<float>("circle_center_x", circle_center[0], 0.0f);
    // 【参数】圆形轨迹参数：圆心Y坐标
    nh.param<float>("circle_center_y", circle_center[1], 0.0f);
    // 【参数】圆形轨迹参数：半径
    nh.param<float>("circle_radius", circle_radius, 1.0f);
    // 【参数】圆形轨迹参数：线速度
    nh.param<float>("linear_vel", linear_vel, 0.1f);
    // 【参数】圆形轨迹参数：圆的方向 1或-1
    nh.param<float>("direction", direction, 1.0f);
    // 计算角速度
    if (circle_radius != 0)
    {
        omega = direction * fabs(float(linear_vel / circle_radius));
    }
    else
    {
        omega = 0.0;
    }
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

    cout << GREEN << ros::this_node::getName() << " start." << TAIL << endl;
    cout << GREEN << "agent_num     : " << agent_num << "" << TAIL << endl;
    cout << GREEN << "circle_center_x : " << circle_center[0] << "" << TAIL << endl;
    cout << GREEN << "circle_center_y : " << circle_center[1] << TAIL << endl;
    cout << GREEN << "circle_radius : " << circle_radius << "" << TAIL << endl;
    cout << GREEN << "linear_vel    : " << linear_vel << "" << TAIL << endl;
    cout << GREEN << "direction     : " << direction << "" << TAIL << endl;
    cout << GREEN << "omega         : " << omega << "" << TAIL << endl;

    //【订阅】触发指令 外部 -> 本节点 
    demo_start_flag_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/demo/swarm_circle", 1, demo_start_flag_cb);
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

    sleep(5.0);
    // 设置ORCA算法HOME点，并启动ORCA算法
    agent_orca_cmd.header.stamp = ros::Time::now();
    agent_orca_cmd.header.frame_id = "world";
    agent_orca_cmd.cmd_source = ros::this_node::getName();
    agent_orca_cmd.orca_cmd = sunray_msgs::orca_cmd::SET_HOME;
    orca_cmd_pub.publish(agent_orca_cmd);

    cout << GREEN << "start orca..." << TAIL << endl;
    sleep(3.0);

    // 将智能体移动到初始位置
    for (int i = 0; i < agent_num; i++) 
    {
        float angle = i * 2 * M_PI / agent_num;
        geometry_msgs::Point goal_point;
        goal_point.x = circle_center[0] + circle_radius * cos(angle);
        goal_point.y = circle_center[1] + circle_radius * sin(angle);
        goal_point.z = circle_center[2];
        orca_goal_pub[i].publish(goal_point);
    }
    sleep(10.0);

    time_trajectory = 0.0;
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

        // 执行圆周运动，当demo_start_flag为false时退出
        while(ros::ok() && demo_start_flag)
        {
            for (int i = 0; i < agent_num; i++) 
            {
                float angle = omega * time_trajectory + i * 2 * M_PI / agent_num;
                geometry_msgs::Point goal_point;
                goal_point.x = circle_center[0] + circle_radius * cos(angle);
                goal_point.y = circle_center[1] + circle_radius * sin(angle);
                goal_point.z = circle_center[2];
                orca_goal_pub[i].publish(goal_point);
            }
            // 更新时间计数器，由于循环频率为10Hz，因此设置为0.1秒
            time_trajectory += 0.1;
            ros::spinOnce();
            rate.sleep();
        }
    }

    return 0;
}

