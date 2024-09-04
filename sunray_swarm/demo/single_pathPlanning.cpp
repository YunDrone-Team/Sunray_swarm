#include <ros/ros.h>
#include <signal.h>
#include <vector>
#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"

using namespace std;

ros::Publisher cmd_pub;    // 发布无人机控制命令
ros::Publisher marker_pub; // 发布RVIZ标记用于显示障碍物
int agent_type;            // 代理类型变量，用于区分不同类型（无人机或无人车）
float agent_height;        // 设置无人机高度变量
int agent_num;              //设置智能体数量

// 规划路径到达目标点
void planAndDriveToTarget(const geometry_msgs::Point &target)
{
    sunray_msgs::agent_cmd cmd;
    // 依据代理类型设置的ID
    cmd.agent_id = 1;
    // 设置控制状态为位置控制模式
    cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
    // orca_cmd.orca_cmd = sunray_msgs::orca_cmd::SET_HOME;

    // 将目标位置赋值给消息的desired_pos字段
    cmd.desired_pos = target;
    // 设置默认的朝向角度为0.0
    cmd.desired_yaw = 0.0;
    // 发布控制命令
    cmd_pub.publish(cmd);
    cout << BLUE << "Agent driving to target: x=" << target.x << " y=" << target.y << " z=" << target.z << endl;
}

// 设置障碍物并发布到RVIZ进行可视化
void setupObstacles()
{
    // 创建一个geometry_msgs::Point对象，用于存储障碍物的位置
    geometry_msgs::Point obstacle;
    // 设置障碍物的坐标
    obstacle.x = 2.0;
    obstacle.y = 2.0;
    obstacle.z = 0.0;

    // 创建一个visualization_msgs::Marker对象，在RVIZ中显示障碍物
    visualization_msgs::Marker marker;
    // 设置标记的参考坐标系为“world”
    marker.header.frame_id = "world";
    // 设置标记的时间戳为当前时间
    marker.header.stamp = ros::Time::now();
    // 设置命名空间为“obstacles”
    marker.ns = "obstacles";
    // 设置标记的ID
    marker.id = 1;
    // 设置标记类型为立方体
    marker.type = visualization_msgs::Marker::CUBE;
    // 设置标记操作为“添加”
    marker.action = visualization_msgs::Marker::ADD;
    // 将障碍物位置赋值给标记的pose.position字段
    marker.pose.position = obstacle;
    // 设置标记的方向缩放比例
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 2.0;
    // 设置标记的红色通道值为1
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    // 设置标记的透明度为1.0
    marker.color.a = 1.0;
    // 发布标记以显示障碍物
    marker_pub.publish(marker);
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "drone_path_planning");
    // 创建节点句柄
    ros::NodeHandle nh;
    // 从参数服务器获取代理类型参数，默认为0
    nh.param<int>("agent_type", agent_type, 0);
    // 从参数服务器设置高度,默认为1
    nh.param<float>("agent_height", agent_height, 1);
    //从参数服务器设置数量，默认为1
    nh.param<int>("agent_num", agent_num, 1);

    // 声明一个字符串变量存储代理前缀
    string agent_prefix;
    switch (agent_type)
    {
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
    // 初始化发布者
    for (int i = 0; i < agent_num; i++)
    {
        agent_name = "/" + agent_prefix + std::to_string(i + 1);
        cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 1);
    }

    // cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm/" + agent_prefix + "1/agent_cmd", 10);
    // 初始化marker_pub发布者，发布RVIZ标记
    marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    // 调用setupObstacles函数设置障碍物并发布到RVIZ
    setupObstacles();
    // Point对象，用于存储目标位置
    geometry_msgs::Point target;
    // 定义并初始化z轴位置变量
    float z = 0.0;
    while (ros::ok())
    {
        cout << GREEN << "Enter target position (x y" << ((agent_type == sunray_msgs::agent_state::RMTT || agent_type == sunray_msgs::agent_state::SIKONG) ? " z" : "") << "): ";
        cin >> target.x >> target.y;
        if (agent_type == sunray_msgs::agent_state::RMTT || agent_type == sunray_msgs::agent_state::SIKONG)
        {
            // cin >> target.z;
            // 从参数服务器获取z
            target.z = agent_height;
        }
        else
        {
            // 对于无人车，z轴设置为0
            target.z = 0.0; // ugv设置
        }
        // 调用planAndDriveToTarget函数规划路径并驱动到目标点
        planAndDriveToTarget(target);
        // 处理一次回调函数
        ros::spinOnce();
        // 等待无人机行驶到目标点
        ros::Duration(2.0).sleep();
    }
    return 0;
}