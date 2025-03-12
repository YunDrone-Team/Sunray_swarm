/***********************************************************************************
 *  文件名: swarm_formation.cpp                                                          
 *  作者: Yun Drone                                                                 
 *  描述: 集群demo：领从编队阵型 agent_num：1-6（其他数量需要重新设定阵型偏移量）
 *     1、读取阵型偏移量（阵型偏移量为提前预设）
 *     2、如果是无人机，请使用地面站一键起飞所有无人机（无人车集群忽略这一步）
 *     3、等待demo启动指令
 *     4、启动后智能体依据阵型偏移量围绕领机展开，领机可自定义移动轨迹，从机跟随（POS_CONTROL模式）
 *     5、可通过demo_start_flag暂停或恢复阵型变换移动
 *     6、本程序没有设置自动降落，可以停止移动后，通过地面站降落
 ***********************************************************************************/

#include <ros/ros.h>
#include <signal.h>
#include "printf_utils.h"
#include "ros_msg_utils.h"

#define MAX_AGENT_NUM 10
using namespace std;

int agent_type;                                         // 智能体类型
int agent_num;                                          // 智能体数量
float agent_height;                                     // 智能体高度
bool demo_start_flag = false;                           // 是否接收到开始命令
std_msgs::String text_info;                             // 打印消息

sunray_msgs::orca_cmd agent_orca_cmd;                   // ORCA指令
sunray_msgs::orca_state orca_state[MAX_AGENT_NUM];      // ORCA状态

geometry_msgs::Point leader_pos;           
geometry_msgs::Point formation_offset[MAX_AGENT_NUM];           

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

// 主机轨迹生成函数（可以根据需要修改）
void generate_reference_trajectory(geometry_msgs::Point &point, double time)
{
    point.x = 1.0 * sin(0.1 * time);            // 根据时间生成参考点的X坐标
    point.y = 1.0 * cos(0.1 * time);            // 根据时间生成参考点的Y坐标
    point.z = 0.0;                              // 根据时间生成参考点的YAW
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "swarm_leader_follower");
    // 创建节点句柄
    ros::NodeHandle nh("~");
    // 设置节点的执行频率为10Hz
    ros::Rate rate(10);

    // 【参数】智能体类型 
    nh.param<int>("agent_type", agent_type, 1);
    // 【参数】智能体编号 
    nh.param<int>("agent_num", agent_num, 6);
    cout << GREEN << ros::this_node::getName() << " start." << TAIL << endl;
    cout << GREEN << "agent_num           : " << agent_num << "" << TAIL << endl;

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
    demo_start_flag_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/demo/swarm_leader_follower", 1, demo_start_flag_cb);
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

    // 设置阵型便宜量
    setup_formation();

    sleep(5.0);
    // 设置ORCA算法HOME点，并启动ORCA算法
    agent_orca_cmd.header.stamp = ros::Time::now();
    agent_orca_cmd.header.frame_id = "world";
    agent_orca_cmd.cmd_source = ros::this_node::getName();
    agent_orca_cmd.orca_cmd = sunray_msgs::orca_cmd::SET_HOME;
    orca_cmd_pub.publish(agent_orca_cmd);

    cout << GREEN << "start orca..." << TAIL << endl;
    sleep(3.0);

    float time = 0.0;
    // 主程序
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

        // 生成领机的位置
        generate_reference_trajectory(leader_pos, time);
        time = time + 0.1;

        // 发布1号机及其跟随无人机的目标点
        for (int i = 0; i < agent_num; i++)
        {
            geometry_msgs::Point goal_point;
            goal_point.x = leader_pos.x + formation_offset[i].x; // 设置目标位置X坐标
            goal_point.y = leader_pos.y + formation_offset[i].y; // 设置目标位置Y坐标
            goal_point.z = leader_pos.z + formation_offset[i].z; // 设置目标YAW
            orca_goal_pub[i].publish(goal_point); // 发布目标点
            sleep(0.1);  
        }

        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}

void setup_formation()
{
    // 设置阵型偏移量
    // 1号智能体
    formation_offset[0].x = 0.0;
    formation_offset[0].y = 0.0;
    formation_offset[0].z = 0.0;
    // 2号智能体
    formation_offset[1].x = 1.0;
    formation_offset[1].y = 0.5;
    formation_offset[1].z = 0.0;
    // 3号智能体
    formation_offset[2].x = 1.0;
    formation_offset[2].y = -0.5;
    formation_offset[2].z = 0.0;
    // 4号智能体
    formation_offset[3].x = -1.0;
    formation_offset[3].y = 1.0;
    formation_offset[3].z = 0.0;
    // 5号智能体
    formation_offset[4].x = -1.0;
    formation_offset[4].y = 0.0;
    formation_offset[4].z = 0.0;
    // 6号智能体
    formation_offset[5].x = -1.0;
    formation_offset[5].y = -1.0;
    formation_offset[5].z = 0.0;
}