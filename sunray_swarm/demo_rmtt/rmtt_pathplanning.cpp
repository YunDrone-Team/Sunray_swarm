#include <ros/ros.h>
#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"

using namespace std;

ros::Publisher agent_cmd_pub;            // 发布无人机控制命令
ros::Publisher marker_pub;               // 发布RVIZ标记，用于显示障碍物
ros::Publisher text_info_pub;            // 发布文字提示消息
ros::Subscriber single_pathPlanning_sub; // 订阅单个路径规划触发信号

bool received_start_cmd = false; // 标记是否接收到开始命令
geometry_msgs::Point target;     // 存储目标位置
int agent_id;                    // 设置智能体编号
float agent_height;              // 设置无人机飞行高度
string node_name;                // 节点名称

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
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    // 设置标记的透明度为1.0
    marker.color.a = 1.0;
    // 发布标记以显示障碍物
    marker_pub.publish(marker);
}

// 触发路径规划的回调函数
void single_pathPlanning_cb(const std_msgs::Bool::ConstPtr &msg)
{
    received_start_cmd = msg->data; // 当收到触发信号为true时，更新状态
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "rmtt_pathPlanning");
    // 创建节点句柄
    ros::NodeHandle nh;
    // 设置循环频率为10Hz
    ros::Rate rate(10);
    // 获取当前节点名称
    node_name = ros::this_node::getName();

    // 【参数】从参数服务器设置高度,默认为1
    nh.param<float>("agent_height", agent_height, 1.0f);
    // 【参数】从参数服务器设置数量，默认为1
    nh.param<int>("agent_id", agent_id, 1);
    // 【参数】从参数服务器获取目标位置——TODO
    nh.param<double>("target_x", target.x, 0.0);
    nh.param<double>("target_y", target.y, 0.0);
    // 【参数】使用智能体飞行高度作为z值
    nh.param<float>("target_z", agent_height);

    // 声明一个字符串变量存储代理前缀
    string agent_name;
    // 构造用于发布控制命令话题名称
    agent_name = "/rmtt_" + std::to_string(agent_id);
    // 【订阅】触发指令 外部 -> 本节点
    single_pathPlanning_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/demo/single_pathPlanning", 1, single_pathPlanning_cb);
    // 【发布】控制指令 本节点 -> 控制节点
    agent_cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 10);
    // 【发布】文字提示消息  本节点 -> 地面站
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);
    // 【发布】 初始化marker_pub发布者，发布RVIZ标记
    marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    // 调用setupObstacles函数设置障碍物并发布到RVIZ
    setupObstacles();
    // 定义并初始化z轴位置变量
    float z = 0;
    while (ros::ok())
    {
        if (received_start_cmd)
        {
            // 发送开始画圆信息
            std_msgs::String start_info;
            start_info.data = "Start Moving";
            // 终端打印信息
            cout << GREEN << "Start Moving" << TAIL << endl;
            // 发布信息
            text_info_pub.publish(start_info);
            // 创建控制命令对象
            sunray_msgs::agent_cmd cmd;
            // 从参数服务器获取z
            target.z = agent_height;
            // 依据代理类型设置的ID
            cmd.agent_id = 1;
            // 设置控制状态为位置控制模式
            cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
            // 设置指令来源
            cmd.cmd_source = "ugv_pathplaning";
            // 将目标位置赋值给消息的desired_pos字段
            cmd.desired_pos = target;
            // 设置默认的朝向角度为0.0
            cmd.desired_yaw = 0.0;
            // 发布控制命令
            agent_cmd_pub.publish(cmd);
            // 打印目标信息
            std_msgs::String end_info;
            end_info.data = "ending Moving";
            // 终端打印信息
            cout << GREEN << "ending Moving" << TAIL << endl;
            // 发布信息
            text_info_pub.publish(end_info);
        }
        // 处理回调函数
        ros::spinOnce();
        // 等待无人机行驶到目标点
        ros::Duration(2.0).sleep();
    }
    return 0;
}