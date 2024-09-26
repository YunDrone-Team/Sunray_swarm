#include <ros/ros.h>
#include "printf_utils.h"
#include "math_utils.h"

using namespace std;

ros::Publisher cmd_pub;                // 发布控制命令
ros::Subscriber cmd_sub;               // 订阅开始命令
ros::Publisher text_info_pub;          // 发布信息到地面站

float desired_yaw;                     // 期望的偏航角
float circle_radius;                   // 圆周轨迹的半径
float linear_vel;                      // 线速度
float time_trajectory = 0.0;           // 轨迹时间计数器
float omega;                           // 角速度
bool received_start_cmd = false;       // 是否接收到开始命令
int agent_id;                          // 智能体编号
float agent_height;                    // 设置无人机高度
string node_name;                      // 节点名称

void startCmdCallback(const std_msgs::Bool::ConstPtr &msg)
{
    received_start_cmd = msg->data;    // 设置接收到的开始命令
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "single_circle");
    ros::NodeHandle nh("~");
    node_name = ros::this_node::getName();

    // 从参数服务器获取参数
    nh.param<float>("desired_yaw", desired_yaw, 0.0f);
    nh.param<float>("circle_radius", circle_radius, 1.0f);
    nh.param<float>("linear_vel", linear_vel, 0.3f);
    nh.param<float>("agent_height", agent_height, 1.0f);
    nh.param<int>("agent_id", agent_id, 1);

    string agent_name = "/rmtt_" + std::to_string(agent_id);
    cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 10);
    cmd_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/demo/single_circle", 1, startCmdCallback);
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);

    ros::Rate rate(10);
    while (ros::ok())
    {
        if (received_start_cmd)
        {
            sunray_msgs::agent_cmd cmd;
            // 移动到起始位置
            cmd.agent_id = agent_id;
            cmd.cmd_source = node_name;
            cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
            cmd.desired_pos.x = circle_radius;   // 圆心+x
            cmd.desired_pos.y = 0.0;             // 圆心+y
            cmd.desired_pos.z = agent_height;    // 高度
            cmd.desired_yaw = desired_yaw;       // 偏航角

            cmd_pub.publish(cmd);
            cout << GREEN << "Moving to start position: x=" << circle_radius << " y=0 z=" << agent_height << " yaw=" << desired_yaw << TAIL << endl;
            ros::Duration(2.0).sleep();  // 等待2秒以确保到达起始位置

            // 执行圆周运动
            time_trajectory = 0.0;  // 重置时间计数器
            while (time_trajectory < 20.0)
            {
                float angle = time_trajectory * omega;
                cmd.agent_id = agent_id;
                cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
                cmd.desired_pos.x = circle_radius * cos(angle);
                cmd.desired_pos.y = circle_radius * sin(angle);
                cmd.desired_pos.z = agent_height;
                cmd.desired_yaw = desired_yaw + angle;  // 使无人机朝向运动方向

                cmd_pub.publish(cmd);
                cout << BLUE << "Moving in circle: x=" << cmd.desired_pos.x << " y=" << cmd.desired_pos.y << " z=" << agent_height << " yaw=" << cmd.desired_yaw << TAIL << endl;
                time_trajectory += 0.1;
                rate.sleep();
            }

            received_start_cmd = false;  // 完成后重置命令
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}