/***********************************************************************************
 *  文件名: path_planning.cpp                                                          
 *  作者: Yun Drone                                                                 
 *  描述: ORCA demo：固定障碍物中的路径规划（基于ORCA算法）
 *     1、初始化障碍物（本demo中障碍物是写死的）
 *     2、通过ORCA算法指令配置障碍物
 *     3、通过ORCA目标点话题来测试是否能够成功避障
 ***********************************************************************************/

#include <ros/ros.h>    
#include "printf_utils.h"
#include "ros_msg_utils.h"

using namespace std;

int agent_type;                           // 智能体编号

sunray_msgs::orca_cmd agent_orca_cmd;   //ORCA算法指令

ros::Publisher orca_cmd_pub;           // ORCA算法指令
ros::Publisher agent_goal_pub;         // ORCA算法目标点
ros::Publisher marker_pub;             // 发布RVIZ标记，用于显示障碍物

void setupObstacles(geometry_msgs::Point obs_center, int marker_id);

// 主函数
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "path_planning");
    // 创建节点句柄
    ros::NodeHandle nh("~");
    // 设置循环频率为10Hz
    ros::Rate rate(10);

    // 【参数】智能体类型
    nh.param<int>("agent_type", agent_type, 1);
    cout << GREEN << ros::this_node::getName() << " start." << TAIL << endl;
    string agent_prefix;
    if(agent_type == sunray_msgs::agent_state::RMTT)
    {
        agent_prefix = "rmtt";
        cout << GREEN << " agent_type: rmtt" << TAIL << endl;
    }
    else if(agent_type == sunray_msgs::agent_state::UGV)
    {
        agent_prefix = "ugv";
        cout << GREEN << " agent_type: ugv" << TAIL << endl;
    }

    // 【发布】ORCA算法指令 本节点 -> ORCA算法节点
    orca_cmd_pub = nh.advertise<sunray_msgs::orca_cmd>("/sunray_swarm/"+agent_prefix+"/orca_cmd", 10);
    // 【发布】ORCA算法目标点 本节点 -> ORCA算法节点
    agent_goal_pub = nh.advertise<geometry_msgs::Point>("/sunray_swarm/" + agent_prefix + "_1/goal_point", 10);
    // 【发布】 初始化marker_pub发布者，发布RVIZ标记
    marker_pub = nh.advertise<visualization_msgs::Marker>("/sunray_swarm/demo/obs_marker", 10);

    sleep(5.0);
    // 设置ORCA算法HOME点，并启动ORCA算法
    agent_orca_cmd.header.stamp = ros::Time::now();
    agent_orca_cmd.header.frame_id = "world";
    agent_orca_cmd.cmd_source = ros::this_node::getName();
    agent_orca_cmd.orca_cmd = sunray_msgs::orca_cmd::SET_HOME;
    orca_cmd_pub.publish(agent_orca_cmd);
    cout << GREEN << "start orca..." << TAIL << endl;

    sleep(1.0);
    // 设置障碍物：1、ORCA算法指令设置障碍物 2、发布障碍物RVIZ显示效果
    // obs_center为障碍物中心点，障碍物为一个边长为0.5米、高度为2米的长方体
    // 共设置2个障碍物
    geometry_msgs::Point obs_center;
    obs_center.x = 0.0;
    obs_center.y = 0.0;
    setupObstacles(obs_center, 1);
    sleep(1.0);
    // obs_center.x = -2.0;
    // obs_center.y = 2.0;
    // setupObstacles(obs_center, 2);

    geometry_msgs::Point goal;
    // 主循环
    while (ros::ok())
    {    
		cout << GREEN << "Please input goal point..." << TAIL << endl;
        cout << GREEN << "goal: --- x [m] "  << TAIL << endl;
        cin >> goal.x;
        cout << GREEN << "goal: --- y [m]"  << TAIL << endl;
        cin >> goal.y;
        cout << GREEN << "goal: --- yaw [rad]"  << TAIL << endl;
        cin >> goal.z;

        agent_goal_pub.publish(goal);
        
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

// 设置障碍物并发布到RVIZ进行可视化
// obs_center为障碍物中心点，障碍物为一个边长为1米、高度为2米的长方体
void setupObstacles(geometry_msgs::Point obs_center, int marker_id)
{
    cout << GREEN << "setup_obstacles at: " << obs_center.x << "," << obs_center.y << TAIL << endl;
    // 在ORCA算法中设置障碍物
    // ----> 定义障碍物顶点
    geometry_msgs::Point Point1, Point2, Point3, Point4;
    Point1.x = obs_center.x + 0.25 + 0.0*rand() / RAND_MAX;
    Point1.y = obs_center.y + 0.25 + 0.0*rand() / RAND_MAX;
    Point2.x = obs_center.x - 0.25+ 0.0*rand() / RAND_MAX;
    Point2.y = obs_center.y + 0.25+ 0.0*rand() / RAND_MAX;
    Point3.x = obs_center.x - 0.25+ 0.0*rand() / RAND_MAX;
    Point3.y = obs_center.y + 0.25+ 0.0*rand() / RAND_MAX;
    Point4.x = obs_center.x + 0.25+ 0.0*rand() / RAND_MAX;
    Point4.y = obs_center.y - 0.25+ 0.0*rand() / RAND_MAX;

    // ----> 通过SETUP_OBS指令在ORCA算法中设置障碍物
    agent_orca_cmd.header.stamp = ros::Time::now();
    agent_orca_cmd.header.frame_id = "world";
    agent_orca_cmd.cmd_source = ros::this_node::getName();
    agent_orca_cmd.orca_cmd = sunray_msgs::orca_cmd::SETUP_OBS;
    agent_orca_cmd.obs_point.push_back(Point1);
    agent_orca_cmd.obs_point.push_back(Point2);
    agent_orca_cmd.obs_point.push_back(Point3);
    agent_orca_cmd.obs_point.push_back(Point4);
    orca_cmd_pub.publish(agent_orca_cmd);

    // 障碍物在RVIZ中显示
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "obstacles";
    marker.id = marker_id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = obs_center.x;
    marker.pose.position.y = obs_center.y;
    marker.pose.position.z = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 2.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker_pub.publish(marker);
}