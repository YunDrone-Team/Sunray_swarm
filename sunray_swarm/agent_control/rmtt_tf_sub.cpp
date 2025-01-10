#include <ros/ros.h>
#include <iostream>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
using namespace std;

// 定义回调函数，用于处理接收到的位姿数据
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ROS_INFO("Received Pose: ");
    ROS_INFO("Position: x = %f, y = %f, z = %f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    ROS_INFO("Orientation: x = %f, y = %f, z = %f, w = %f", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
}

int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "pose_subscriber");

    // 创建一个 ROS 节点句柄
    ros::NodeHandle nh;

     int agent_id;
        // 【参数】智能体编号
        nh.param<int>("agent_id", agent_id, 1);
        string agent_name = "rmtt_"+std::to_string(agent_id);
        

    // 订阅话题 /program_pose，消息类型是 geometry_msgs::PoseStamped
    ros::Subscriber map_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/sunray_swarm/"+ agent_name +"/map_pose", 1,poseCallback);

    // 使用 ros::spin() 让回调函数不断被调用
    ros::spin();

    return 0;
}