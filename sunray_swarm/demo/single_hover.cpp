#include <ros/ros.h>
#include <signal.h>
#include "sunray_msgs/agent_cmd.h"
#include "geometry_msgs/Point.h"
#include <iostream>
#include "ros_msg_utils.h"
#include "printf_utils.h"
using namespace std;

ros::Publisher cmd_pub;  // 发布控制命令
int agent_type; // 代理类型，用于区分无人机和无人车
float agent_height;//设置无人机高度
int agent_num;//设置智能体数量
ros::Subscriber trigger_sub; // 订阅触发信号
bool triggered = 0; // 触发状态标志

// 信号处理函数，用于关闭节点
void mySigintHandler(int sig) {
    ROS_INFO("Shutting down agent hover control node...");
    ros::shutdown();
}

// 发布悬停位置的函数
void publishHoverPosition(float x, float y, float z, float yaw) {
    sunray_msgs::agent_cmd cmd;
    // 代理的ID
    cmd.agent_id = 1;  
    // 设置位置控制模式
    cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL; 
     // 设置期望的悬停位置x坐标
    cmd.desired_pos.x = x; 
    cmd.desired_pos.y = y;
    // 设置期望的悬停位置z坐标（高度agent_height）
    cmd.desired_pos.z = agent_height;  
    //设置期望的悬停姿态（偏航角）
    cmd.desired_yaw = yaw; 
    cmd_pub.publish(cmd);
    cout << BLUE << "Agent moving to hover position: x=" << x << " y=" << y << " z=" << z << " yaw=" << yaw << endl;
}
// 触发信号的回调函数
void triggerCallback(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        cout << BLUE << "Trigger received, moving to preset position..." << endl;
        // 预设目标位置
        // float preset_x = 2.0, preset_y = 2.0, preset_z = agent_height, preset_yaw = 0.0;
        // float preset_x = 1.0, preset_y = 2.0, preset_z = agent_height, preset_yaw = 0.0;
        // float preset_x = 3.0, preset_y = 2.0, preset_z = agent_height, preset_yaw = 0.0;
        // publishHoverPosition(preset_x, preset_y, preset_z, preset_yaw);
        triggered = true; // 设置触发标志
    }
}

int main(int argc, char **argv) {
    // 初始化ROS节点，节点名为"agent_hover_control"
    ros::init(argc, argv, "agent_hover_control");
    //创建节点句柄
    ros::NodeHandle nh;
    // 设置信号处理函数，用于关闭节点
    signal(SIGINT, mySigintHandler); 
    // 从参数服务器获取agent_type参数的值，默认为1 (tianbot)
    nh.param<int>("agent_type", agent_type, 1); 
    // 【参数】智能体高度
    nh.param<float>("agent_height", agent_height, 1.0);
    // 定义一个字符串变量，用于存储代理前缀

    // 【参数】智能体编号
    nh.param<int>("agent_num", agent_num, 1);
    string agent_prefix;

    // 根据类型选择适当的前缀
    switch(agent_type) {
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

    // 初始化发布者
    string agent_name;
    for (int i = 0; i < agent_num; i++)
    {
        agent_name = "/" +agent_prefix +std::to_string(i+1);
        cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 10);
    }
    trigger_sub = nh.subscribe<std_msgs::Bool>("/trigger_signal", 10, triggerCallback);

    // [订阅]触发条件
    // agent_cmd_pub = nh.advertise<std_msgs::Bool>("/sunray_swarm/single_hover", 1， start_cmd_cb);

    
    // 定义悬停位置变量，默认z为0，对于无人车适用
    float x, y, yaw, z = 0.0;  // 默认Z为0，对于无人车适用
    if (agent_type == sunray_msgs::agent_state::RMTT || agent_type == sunray_msgs::agent_state::SIKONG) {
        z = agent_height; // 默认无人机高度为1m
    }

    // 从用户输入获取初始悬停位置
    cout << GREEN << "Enter initial hover position (x, y, (z), yaw in degrees, and z if UAV): ";
    if (z != 0.0) {
        cin >> x >> y >> z >> yaw;  // 无人机包含z轴
    } else {
        cin >> x >> y >> yaw;  // 无人车不包含z轴
    }
    yaw = yaw * M_PI / 180.0; // 转换为弧度
    // 发布起始悬停位置
    publishHoverPosition(x, y, z, yaw); 

    // 主循环
    while (ros::ok()) {
        cout << GREEN << "Update hover position (x, y, (z), yaw in degrees, and z if UAV): ";
        if (z != 0.0) {
            cin >> x >> y >> z >> yaw; // 如果是无人机，输入包含z轴
        } else {
            cin >> x >> y >> yaw;  // 如果是无人车，输入不包含z轴
        }
        // 将偏航角从度转换为弧度
        yaw = yaw * M_PI / 180.0; 
        publishHoverPosition(x, y, z, yaw); // 根据输入更新悬停位置
        ros::spinOnce(); // 处理回调函数
    }

    return 0;
}