#include <ros/ros.h>
#include <iostream>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include "printf_utils.h"

using namespace std;

int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "map_tf_listener");
    // 创建句柄
    ros::NodeHandle nh("~");

    // 【参数】智能体编号
    int agent_id;
    nh.param<int>("agent_id", agent_id, 1);
    string agent_name = "rmtt_" + std::to_string(agent_id);

    // 【发布】定位数据  本节点 -> rmtt_driver
    ros::Publisher map_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "/sunray_swarm/" + agent_name + "/map_pose", 1);

    // 初始化 TF 监听器
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // 设定地图框架和程序框架的名称
    string map_frame = "sunray_swarm/" + agent_name + "/map";        // 地图框架
    string program_frame = "sunray_swarm/" + agent_name + "/base_link"; // 程序框架

    // 标志变量，用于控制错误信息的打印
    bool has_printed_error = false;

    // 主循环
    ros::Rate rate(5); // 10 Hz
    while (ros::ok())
    {
        try
        {
            // 获取从地图到程序框架的变换
            geometry_msgs::TransformStamped transform_stamped =
                tf_buffer.lookupTransform(map_frame, program_frame, ros::Time(0));

            // 获取位移和平移
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = map_frame;

            pose.pose.position.x = transform_stamped.transform.translation.x;
            pose.pose.position.y = transform_stamped.transform.translation.y;
            pose.pose.position.z = transform_stamped.transform.translation.z;

            // 获取旋转（四元数）
            pose.pose.orientation = transform_stamped.transform.rotation;

            // 将 pose 发布到 /map_pose 话题
            map_pose_pub.publish(pose);

            // 如果之前打印过错误信息，则重置标志变量
            if (has_printed_error)
            {
                cout << GREEN << ">>>>>>>>>>>>>>>>>>> RMTT_MAP  Parameters <<<<<<<<<<<<<<<<" << TAIL << endl;
                cout << GREEN << ">>>>>>>>>>>>>>>>>>> TF transform is now available. <<<<<<<<<<<<<<<<" << TAIL << endl;
                has_printed_error = false;
            }
        }
        catch (const tf2::TransformException& ex)
        {
            // 只在第一次出错时打印错误信息
            if (!has_printed_error)
            {
                cout << RED << ">>>>>>>>>>>>>>>>>>> RMTT_MAP Parameters <<<<<<<<<<<<<<<<" << TAIL << endl;
                cout << RED << "TF Error: \"" << map_frame << "\"" << TAIL << endl;
                has_printed_error = true;
            }
        }

        rate.sleep();
    }

    return 0;
}