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
ros::Publisher agent_cmd_pub[MAX_AGENT_NUM];    // 存储每个智能体的控制指令发布者
ros::Publisher text_info_pub;                   // 文字提示消息的发布者
geometry_msgs::Point reference_point;           // 存储参考轨迹的点
geometry_msgs::Point offset[MAX_AGENT_NUM];     // 存储各个智能体的偏移量
bool received_start_cmd = false;                // 标记是否接收到开始命令
// 处理触发命令的回调函数
void swarm_leader_cmd_cb(const std_msgs::Bool::ConstPtr &msg)
{
    received_start_cmd = msg->data;             // 设置输入信息
    start_cmd = 1;
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

    // 定义一个字符串变量，用于存储代理前缀
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
    // 存储智能体名称
    string agent_name;
    for (int i = 0; i < agent_num; i++)
    {
        // 生成智能体名称
        agent_name = "/" + agent_prefix + std::to_string(i + 1);
        // 【发布】无人车控制指令
        agent_cmd_pub[i] = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 1);
    }
    // [订阅]地面站 ->  本节点  触发条件
    swarm_leader_cmd_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/demo/leader_follower", 1, swarm_leader_cmd_cb);
    // 定义开始时间
    ros::Time start_time;
    // 如果启动命令已接收
    if (start_cmd == 1)
    {
        // 记录当前时间为开始时间
        ros::Time start_time = ros::Time::now();
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

            // 发布1号机的控制指令
            sunray_msgs::agent_cmd agent_cmd[MAX_AGENT_NUM]; // 定义控制指令数组
            agent_cmd[0].agent_id = 1; // 设置1号机的ID
            agent_cmd[0].control_state = sunray_msgs::agent_cmd::POS_CONTROL; // 设置控制状态为位置控制
            agent_cmd[0].desired_pos = reference_point; // 设置目标位置为参考点
            agent_cmd[0].desired_yaw = 0.0; // 固定yaw
            agent_cmd_pub[0].publish(agent_cmd[0]); // 发布1号机的控制指令

            // 2号和3号机跟随1号机
            for (int i = 1; i < agent_num; i++)
            {
                agent_cmd[i].agent_id = i + 1; // 设置智能体ID
                agent_cmd[i].control_state = sunray_msgs::agent_cmd::POS_CONTROL; // 设置控制状态为位置控制
                agent_cmd[i].desired_pos.x = reference_point.x + offset[i].x; // 设置目标位置X坐标
                agent_cmd[i].desired_pos.y = reference_point.y + offset[i].y; // 设置目标位置Y坐标
                agent_cmd[i].desired_pos.z = reference_point.z + offset[i].z; // 设置目标位置Z坐标
                agent_cmd[i].desired_yaw = 0.0; // 固定yaw
                agent_cmd_pub[i].publish(agent_cmd[i]); // 发布跟随机的控制指令
            }
        }
        ros::spinOnce();
        // 休眠0.1秒
        ros::Duration(0.1).sleep();
    }

    return 0;
}