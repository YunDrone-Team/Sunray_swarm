/***********************************************************************************
 *  文件名: ugv_pathplanning.cpp                                                          
 *  作者: Yun Drone                                                                 
 *  描述: 无人车demo：圆形轨迹移动
 *     1、从参数列表里面获取圆形轨迹参数
 *     2、等待demo启动指令
 *     3、启动后根据时间计算圆形轨迹位置并发送到控制节点执行（POS_CONTROL模式）
 *     4、可通过demo_start_flag暂停或恢复圆形轨迹
 ***********************************************************************************/

#include <ros/ros.h>    
#include "printf_utils.h"
#include "ros_msg_utils.h"

using namespace std;

int agent_id;                           // 智能体编号
bool demo_start_flag = false;           // 标记是否接收到开始命令
geometry_msgs::Point target;            // 存储目标位置

sunray_msgs::orca_cmd agent_orca_cmd;   //ORCA算法指令

ros::Publisher orca_cmd_pub;           // ORCA算法指令
ros::Publisher text_info_pub;          // 发布信息到地面站
ros::Subscriber demo_start_flag_sub;   // 订阅开始命令
ros::Publisher marker_pub;             // 发布RVIZ标记，用于显示障碍物

// 设置障碍物并发布到RVIZ进行可视化
void setupObstacles()
{
    cout << GREEN << "setup_obstacles" << TAIL << endl;

    geometry_msgs::Point Point1, Point2, Point3, Point4;
    Point1.x = 0.5;
    Point1.y = 0.5;
    Point2.x = -0.5;
    Point2.y = 0.5;
    Point3.x = -0.5;
    Point3.y = 0.5;
    Point4.x = 0.5;
    Point4.y = -0.5;

    // 在ORCA算法中添加固定障碍物
    agent_orca_cmd.header.stamp = ros::Time::now();
    agent_orca_cmd.header.frame_id = "world";
    agent_orca_cmd.cmd_source = "ugv_pathplanning";
    agent_orca_cmd.orca_cmd = sunray_msgs::orca_cmd::SETUP_OBS;
    // 将顶点添加到障碍物指令中
    agent_orca_cmd.obs_point.push_back(Point1);
    agent_orca_cmd.obs_point.push_back(Point2);
    agent_orca_cmd.obs_point.push_back(Point3);
    agent_orca_cmd.obs_point.push_back(Point4);
    orca_cmd_pub.publish(agent_orca_cmd);

    // 障碍物在RVIZ中显示
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
void demo_start_flag_cb(const std_msgs::Bool::ConstPtr& msg) 
{
    demo_start_flag = msg->data;  
}

// 主函数
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "ugv_pathplanning");
    // 创建节点句柄
    ros::NodeHandle nh("~");
    // 设置循环频率为10Hz
    ros::Rate rate(10);

    // 【参数】智能体编号
    nh.param<int>("agent_id", agent_id, 1);
    // 【参数】从参数服务器获取目标位置——TODO
    nh.param<double>("target_x", target.x, 0.0);
    nh.param<double>("target_y", target.y, 0.0);

    cout << GREEN << ros::this_node::getName() << " start." << TAIL << endl;
    cout << GREEN << "agent_id      : " << agent_id << TAIL << endl;


    string agent_name = "/ugv_" + std::to_string(agent_id);
    // 【订阅】触发指令 外部 -> 本节点 
    demo_start_flag_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/demo/ugv_pathplanning", 1, demo_start_flag_cb);
    // 【发布】ORCA算法指令 本节点 -> ORCA算法节点
    orca_cmd_pub = nh.advertise<sunray_msgs::orca_cmd>("/sunray_swarm/ugv/orca_cmd", 10);
    // 【发布】文字提示消息  本节点 -> 地面站
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);
    // 【发布】 初始化marker_pub发布者，发布RVIZ标记
    marker_pub = nh.advertise<visualization_msgs::Marker>("/sunray_swarm/demo/obs_marker", 10);

    // 设置障碍物：1、ORCA算法指令设置障碍物 2、发布障碍物RVIZ显示效果
    setupObstacles();

    // 设置ORCA算法HOME点，并启动ORCA算法
    agent_orca_cmd.header.stamp = ros::Time::now();
    agent_orca_cmd.header.frame_id = "world";
    agent_orca_cmd.cmd_source = "ugv_pathplanning";
    agent_orca_cmd.orca_cmd = sunray_msgs::orca_cmd::SET_HOME;
    orca_cmd_pub.publish(agent_orca_cmd);

    cout << GREEN << "start orca..." << TAIL << endl;


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
        
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}