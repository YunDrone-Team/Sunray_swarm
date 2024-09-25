#include <ros/ros.h>
#include "printf_utils.h"
#include "math_utils.h"

ros::Publisher agent_cmd_pub; // 发布控制命令
float desired_yaw;            // 期望的偏航角
float circle_radius;          // 圆周轨迹的半径
float linear_vel;             // 线速度
float time_trajectory = 0.0;  // 轨迹时间计数器
float omega;                  // 角速度
float agent_height;
int agent_id;
int start_cmd = 0; // 开始命令标志
string node_name;
ros::Publisher text_info_pub; // 发布文字提示消息

bool received_start_cmd = false; // 设置开始点
ros::Subscriber single_pathplanning_sub;
float trajectory_total_time = 20;

// 回调函数
void single_circle_cb(const std_msgs::Bool::ConstPtr &msg)
{
    received_start_cmd = msg->data; // 设置输入信息
    start_cmd = 1;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "single_circle");
    ros::NodeHandle nh("~");
    ros::Rate rate(10);
    node_name = ros::this_node::getName();
    // 从参数服务器获取参数
    nh.param<float>("desired_yaw", desired_yaw, 0.0f);
    nh.param<float>("circle_radius", circle_radius, 1.0f);
    nh.param<float>("linear_vel", linear_vel, 0.3f);
    nh.param<float>("agent_height", agent_height, 1.0f);
    // 【参数】智能体编号
    nh.param<int>("agent_id", agent_id, 1);

    string agent_name;
    agent_name = "/rmtt_" + std::to_string(agent_id);
    // 【发布】控制指令 本节点 -> 控制节点
    agent_cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 10);
    // 【订阅】触发指令 外部 -> 本节点
    single_pathplanning_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/demo/single_circle", 1, single_circle_cb);
    // 【发布】文字提示消息  本节点 -> 地面站
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);

    sunray_msgs::agent_cmd cmd;

    while (ros::ok())
    {
        if (circle_radius != 0)
        {
            omega = linear_vel / circle_radius;
            

            if (received_start_cmd)
            {
                // 发布初始位置命令
                sunray_msgs::agent_cmd cmd;
                // ID 默认为1
                cmd.agent_id = 1;
                // 设置指令来源
                cmd.cmd_source = "single_circle";
                // 设置控制状态为位置控制模式
                cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
                // 设置起始位置的x坐标为圆周轨迹的半径
                cmd.desired_pos.x = circle_radius;
                // 设置起始位置的y坐标为0
                cmd.desired_pos.y = 0.0;
                // 设置起始位置的偏航角
                cmd.desired_yaw = desired_yaw;
                // 发布初始位置命令
                agent_cmd_pub.publish(cmd);
                cout << GREEN << "Moving to start position: x=" << circle_radius << " y=0" << " z=" << cmd.desired_pos.z << " yaw=" << desired_yaw << TAIL << endl;
                // 等待2秒以确保移动完成
                ros::Duration(5.0).sleep();
                received_start_cmd = false;

                time_trajectory = 0.0; // 重置时间计数器
            }
            else if (start_cmd == 1  && time_trajectory < 20.0)
            {
                float angle = time_trajectory * omega;
                cmd.agent_id = 1;
                cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
                cmd.desired_pos.x = circle_radius * cos(angle);
                cmd.desired_pos.y = circle_radius * sin(angle);
                cmd.desired_pos.z = agent_height;
                cmd.desired_yaw = desired_yaw;
                agent_cmd_pub.publish(cmd);
                cout << BLUE << "Moving in circle: x=" << cmd.desired_pos.x << " y=" << cmd.desired_pos.y << " z=" << cmd.desired_pos.z << " yaw=" << desired_yaw << TAIL << endl;
                time_trajectory += 0.1;
            }
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}