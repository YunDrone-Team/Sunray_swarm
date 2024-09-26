#include <ros/ros.h>
#include "printf_utils.h"
#include "math_utils.h"

using namespace std;

ros::Publisher agent_cmd_pub;                // 发布控制命令
ros::Subscriber rmtt_circle_cmd_sub;               // 订阅开始命令
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

void rmtt_circle_cb(const std_msgs::Bool::ConstPtr &msg)
{
    received_start_cmd = msg->data;    // 设置接收到的开始命令
}

int main(int argc, char **argv)
{
    // 初始化ROS节点，设置节点名称为"single_circle"
    ros::init(argc, argv, "single_circle");
    // 创建节点句柄
    ros::NodeHandle nh("~");
    // 获取并存储当前节点的名称
    node_name = ros::this_node::getName();
    // 设置节点的执行频率为10Hz
    ros::Rate rate(10);

    // 从参数服务器获取参数
    // 从参数服务器获取期望的偏航角，默认为0.0
    nh.param<float>("desired_yaw", desired_yaw, 0.0f);
    // 从参数服务器获取圆周轨迹的半径，默认为1.0米
    nh.param<float>("circle_radius", circle_radius, 1.0f);
    // 从参数服务器获取线速度，默认为0.3米/秒
    nh.param<float>("linear_vel", linear_vel, 0.3f);
    // 从参数服务器获取无人机的飞行高度，默认为1.0米
    nh.param<float>("agent_height", agent_height, 1.0f);
    // 从参数服务器获取智能体编号，默认为1
    nh.param<int>("agent_id", agent_id, 1);

    // 构造用于发布控制命令的完整话题名称
    string agent_name = "/rmtt_" + std::to_string(agent_id);
    // 初始化命令发布者，话题名称根据智能体类型和编号动态生成
    agent_cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 10);
    // 初始化开始命令的订阅者，监听特定的触发信号
    rmtt_circle_cmd_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/demo/single_circle", 1, rmtt_circle_cb);
    // 初始化地面站信息发布者，用于发送文本信息到地面站
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);
    
    while (ros::ok())
    {   
        // 检查是否接收到开始命令
        if (received_start_cmd)
        {
            // 创建控制命令消息对象
            sunray_msgs::agent_cmd cmd;
            // 移动到起始位置
            cmd.agent_id = 1;   // 指定智能体编号
            cmd.cmd_source = "rmtt_circle";  // 指定命令来源为当前节点
            cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;  // 设置控制模式为位置控制
            cmd.desired_pos.x = circle_radius;  // 设置x坐标为圆周半径
            cmd.desired_pos.y = 0.0;            // 设置y坐标为0
            cmd.desired_pos.z = agent_height;   // 设置z坐标为设定的飞行高度
            cmd.desired_yaw = desired_yaw;      // 设置偏航角
            // 发布初始位置命令
            agent_cmd_pub.publish(cmd);
            cout << GREEN << "Moving to start position: x=" << circle_radius << " y=0 z=" << agent_height << " yaw=" << desired_yaw << TAIL << endl;
            ros::Duration(3.0).sleep();  // 等待3秒以确保到达起始位置

            // 重置时间计数器，准备执行圆周运动
            time_trajectory = 0.0;  
            // 执行圆周运动，持续时间20秒
            while (time_trajectory < 20.0)
            {
                // 匀速圆周运动的角速度
                omega = linear_vel / circle_radius;
                float angle = time_trajectory * omega;
                cmd.agent_id = 1;
                cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
                cmd.desired_pos.x = circle_radius * cos(angle);  // 计算当前x坐标
                cmd.desired_pos.y = circle_radius * sin(angle);  // 计算当前y坐标
                cmd.desired_pos.z = agent_height;
                cmd.desired_yaw = desired_yaw;  // 更新偏航角
                // 发布当前位置命令
                agent_cmd_pub.publish(cmd);
                cout << BLUE << "Moving in circle: x=" << cmd.desired_pos.x << " y=" << cmd.desired_pos.y << " z=" << agent_height << " yaw=" << cmd.desired_yaw << TAIL << endl;
                // 更新时间计数器
                time_trajectory += 0.1;
                rate.sleep();
            }
            // 圆周运动完成后，重置开始命令标志
            received_start_cmd = false;  // 完成后重置命令
        }
        // 调用ROS spinOnce函数来处理回调函数
        ros::spinOnce();
        // 根据设定频率休眠，保持稳定的循环频率
        rate.sleep();
    }
    return 0;
}