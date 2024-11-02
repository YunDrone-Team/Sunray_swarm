#include <ros/ros.h>
#include "std_msgs/String.h"
#include "math_utils.h"

#define MAX_AGENT_NUM 3

using namespace std;
int agent_type;                                 // 代理类型，用于区分无人机和无人车
ros::Subscriber swarm_leader_cmd_sub;           // 触发条件

int agent_num = MAX_AGENT_NUM;                  // 初始化智能体数量为最大数量
float agent_height;                             // 智能体的飞行高度
int start_cmd;                                  // 启动命令
ros::Publisher text_info_pub;                   // 文字提示消息的发布者
geometry_msgs::Point reference_point;           // 存储参考轨迹的点
geometry_msgs::Point offset[MAX_AGENT_NUM];     // 存储各个智能体的偏移量
bool received_start_cmd = false;                // 标记是否接收到开始命令

ros::Subscriber orca_state_sub[MAX_AGENT_NUM]; // ORCA状态订阅器
ros::Publisher orca_goal_pub[MAX_AGENT_NUM];   // 智能体目标点发布器
sunray_msgs::orca_state orca_state[MAX_AGENT_NUM];      // ORCA状态
sunray_msgs::orca_cmd orca_cmd;                         // ORCA指令
ros::Publisher orca_cmd_pub;                        // 发布ORCA指令

// ORCA状态回调函数
void rmtt_orca_state_cb(const sunray_msgs::orca_stateConstPtr &msg, int i)
{
    orca_state[i] = *msg; // 更新指定智能体的ORCA状态
}
// 处理触发命令的回调函数
void swarm_leader_cmd_cb(const std_msgs::Bool::ConstPtr &msg)
{
    received_start_cmd = msg->data;             // 设置输入信息
    start_cmd = 1;
    // 设置ORCA命令为HOME
    orca_cmd.orca_cmd = sunray_msgs::orca_cmd::SET_HOME;
    // 发布命令
    orca_cmd_pub.publish(orca_cmd);
    // 设置延迟
    sleep(0.5);
}
// 主机轨迹生成函数（可以根据需要修改）
void generate_reference_trajectory(geometry_msgs::Point &point, double time)
{
    point.x = 1.0 * sin(0.1 * time);            // 根据时间生成参考点的X坐标
    point.y = 1.0 * cos(0.1 * time);            // 根据时间生成参考点的Y坐标
    point.z = agent_height;                     // 设置参考点的高度为智能体的高度
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "leader_follower_control");
    // 创建节点句柄
    ros::NodeHandle nh("~");
    // 设置节点的执行频率为10Hz
    ros::Rate rate(10);
    // 【参数】从参数服务器获取智能体高度
    nh.param<float>("agent_height", agent_height, 1.0f); 
    // 【参数】从参数服务器获取智能体类型
    nh.param<int>("agent_type", agent_type, 0);        

    // 设置2号和3号无人机的偏移量
    offset[1].x = 0.4; // 设置2号机在X轴上的偏移量
    offset[1].y = 0.7; // 设置2号机在Y轴上的偏移量
    offset[1].z = 0.0; // 设置2号机的Z坐标为0
    offset[2].x = -0.4; // 设置3号机在X轴上的偏移量
    offset[2].y = 0.7; // 设置3号机在Y轴上的偏移量
    offset[2].z = 0.0; // 设置3号机的Z坐标为0

    string agent_prefix;
    switch (agent_type)
    {
    case sunray_msgs::agent_state::RMTT:
        agent_prefix = "rmtt_";
        break;
    case sunray_msgs::agent_state::UGV:
        agent_prefix = "ugv_";
        break;
    case sunray_msgs::agent_state::SIKONG:
        agent_prefix = "sikong_";
        break;
    default:
        agent_prefix = "unknown_";
        break;
    }
    string agent_name;
    // 【发布】ORCA指令 本节点 ->  ORCA
    orca_cmd_pub = nh.advertise<sunray_msgs::orca_cmd>("/sunray_swarm/" + agent_prefix + "/orca_cmd", 1);
    for (int i = 0; i < agent_num; i++)
    {
        agent_name = "/" + agent_prefix + std::to_string(i + 1);
        // 【订阅】无人机ORCA状态
        orca_state_sub[i] = nh.subscribe<sunray_msgs::orca_state>("/sunray_swarm" + agent_name + "/agent_orca_state", 1, boost::bind(&rmtt_orca_state_cb, _1, i));
        // 【发布】无人机的目标点
        orca_goal_pub[i] = nh.advertise<geometry_msgs::Point>("/sunray_swarm" + agent_name + "/goal_point", 1);
    }
    // [订阅]地面站 ->  本节点  触发条件
    swarm_leader_cmd_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/demo/swarm_leader_follower", 1, swarm_leader_cmd_cb);
    // 定义开始时间
    ros::Time start_time;
    // 如果启动命令已接收
    if (start_cmd == 1)
    {
        // 记录当前时间为开始时间
        ros::Time start_time = ros::Time::now();
        start_cmd == 0;
    }
    sleep(5.0);
    // 主程序
    while (ros::ok())
    {
        // 如果接收到开始命令
        if (received_start_cmd)
        {
            // 获取当前时间
            ros::Time current_time = ros::Time::now();
            // 计算自开始以来经过的时间
            double elapsed_time = (current_time - start_time).toSec();
    
            // 生成1号机的参考轨迹
            generate_reference_trajectory(reference_point, elapsed_time);
    
            // 发布1号机及其跟随无人机的目标点
            for (int i = 0; i < agent_num; i++)
            {
                geometry_msgs::Point goal_point;
                goal_point.x = reference_point.x + offset[i].x; // 设置目标位置X坐标
                goal_point.y = reference_point.y + offset[i].y; // 设置目标位置Y坐标
                goal_point.z = reference_point.z + offset[i].z; // 设置目标位置Z坐标
                orca_goal_pub[i].publish(goal_point); // 发布目标点
            }
        }
        ros::spinOnce();
        // 休眠0.1秒
        ros::Duration(0.1).sleep();
    }
    
    return 0;
}