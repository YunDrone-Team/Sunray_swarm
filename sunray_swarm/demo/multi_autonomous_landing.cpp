#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include "sunray_msgs/agent_cmd.h"

ros::Publisher drone_cmd_pub;
ros::Publisher car_cmd_pub;
ros::Subscriber car_position_sub;
ros::Subscriber drone_position_sub;

geometry_msgs::PoseStamped car_position;
geometry_msgs::PoseStamped drone_position;
geometry_msgs::PoseStamped landing_target;

// 状态枚举
enum MISSION_STATE
{
    TAKEOFF,
    FLIGHT,
    LANDING,
    COMPLETED
};
MISSION_STATE mission_state;

// 起飞
void takeoff()
{
    sunray_msgs::agent_cmd cmd;
    cmd.control_state = sunray_msgs::agent_cmd::TAKEOFF;
    drone_cmd_pub.publish(cmd);
    ROS_INFO("Drone takeoff initiated.");
    mission_state = FLIGHT;
}

// 航线飞行
void flight()
{
    sunray_msgs::agent_cmd cmd;
    cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
    // 设置目标航点
    cmd.desired_pos.x = 10.0;
    cmd.desired_pos.y = 10.0;
    cmd.desired_pos.z = 5.0; // 保持高度
    drone_cmd_pub.publish(cmd);
    ROS_INFO("Drone is flying along the path.");
    // 检查是否接近降落点
    if (fabs(drone_position.pose.position.x - car_position.pose.position.x) < 0.5 &&
        fabs(drone_position.pose.position.y - car_position.pose.position.y) < 0.5)
    {
        mission_state = LANDING;
    }
}

// 自主降落
void landing()
{
    sunray_msgs::agent_cmd cmd;
    cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
    // 设置降落目标点（无人车位置）
    cmd.desired_pos.x = car_position.pose.position.x;
    cmd.desired_pos.y = car_position.pose.position.y;
    cmd.desired_pos.z = car_position.pose.position.z + 0.2; // 高度略高于车身
    drone_cmd_pub.publish(cmd);

    ROS_INFO("Drone is preparing to land.");

    // 当高度接近0时完成降落
    if (fabs(drone_position.pose.position.z - car_position.pose.position.z) < 0.1)
    {
        mission_state = COMPLETED;
        ROS_INFO("Drone has successfully landed on the vehicle.");
    }
}

// 车位置回调函数
void carPositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    car_position = *msg;
}

// 无人机位置回调函数
void dronePositionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    drone_position = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "autonomous_landing_demo");
    ros::NodeHandle nh;

    // 初始化发布者和订阅者
    drone_cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("drone_cmd", 10);
    car_cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("car_cmd", 10);
    car_position_sub = nh.subscribe("car_position", 10, carPositionCallback);
    drone_position_sub = nh.subscribe("drone_position", 10, dronePositionCallback);

    ros::Rate rate(10.0);
    mission_state = TAKEOFF;

    while (ros::ok())
    {
        switch(mission_state)
        {
            case TAKEOFF:
                takeoff();
                break;

            case FLIGHT:
                flight();
                break;

            case LANDING:
                landing();
                break;

            case COMPLETED:
                ROS_INFO("Mission completed.");
                break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}