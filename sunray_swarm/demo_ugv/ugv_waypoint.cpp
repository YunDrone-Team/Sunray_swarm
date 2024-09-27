#include <ros/ros.h>
#include "printf_utils.h"
#include "ros_msg_utils.h"


using namespace std;

ros::Publisher agent_cmd_pub;            // 发布控制命令
vector<geometry_msgs::Point> waypoints;  // 航点列表
int agent_id;                            // 代理编号
float agent_height;                      // 无人机飞行高度
bool received_start_cmd = false;         // 标记是否接收到开始命令
ros::Publisher text_info_pub;            // 发布文字提示消息
ros::Subscriber single_waypoint_sub;     // 订阅路径规划触发信号
string node_name;


// 触发信号的回调函数，处理接收到的位置
void single_waypoint_cb(const std_msgs::Bool::ConstPtr &msg) 
{
    received_start_cmd = msg->data; // 设置输入信息
}

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "ugv_waypoint");
    // 创建节点句柄
    ros::NodeHandle nh;
    // 设置循环频率为10Hz
    ros::Rate rate(10);
    // 【参数】智能体高度
    nh.param<float>("agent_height", agent_height, 0.0f);
    // 【参数】智能体编号
    nh.param<int>("agent_id", agent_id, 1);


    string agent_name;
    agent_name = "/ugv_" + std::to_string(agent_id);
    // 【订阅】触发指令 外部 -> 本节点 
    single_waypoint_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/demo/single_waypoint", 1, single_waypoint_cb);
    // 【发布】控制指令 本节点 -> 控制节点
    agent_cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 10);
    // 【发布】文字提示消息  本节点 -> 地面站
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);

    // 设置航点列表
    // 提取存储的航点数
    int waypoint_count;
    nh.param("waypoint_count", waypoint_count, 1); // 获取航点总数，默认为0
    waypoints.resize(waypoint_count);
    
    for (int i = 0; i < waypoint_count; ++i) {
        // 构造参数名称
        string param_base = "waypoint_" + to_string(i + 1); // 航点编号从1开始
        nh.param(param_base + "_x", waypoints[i].x, 1.0);
        nh.param(param_base + "_y", waypoints[i].y, 1.0);
        waypoints[i].z = agent_height; // 高度使用智能体设置的高度
    }

    sunray_msgs::agent_cmd cmd;
    // 从用户输入获取航点位置
    while (ros::ok()) 
    {
        if(received_start_cmd)
        {
            // 设置为起飞状态
            cmd.control_state = 11;
            // 发送开始导航信息
            std_msgs::String start_info;
            start_info.data = "Start Moving";
            // 终端打印信息
            cout << GREEN << "Start Moving" << TAIL << endl;
            // 发布信息
            text_info_pub.publish(start_info);
            // 向每个航点发送导航命令
            for (auto &waypoint : waypoints) {
                
                cmd.agent_id = agent_id;
                cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
                cmd.cmd_source = "ugv_waypoint";
                cmd.desired_pos = waypoint;
                cmd.desired_yaw = 0.0; // 偏航角通常在导航过程中另行计算

                agent_cmd_pub.publish(cmd); // 发布控制命令

                // 发送提示消息
                std_msgs::String text_info;
                text_info.data = "Navigating to waypoint: x=" + to_string(waypoint.x) + " y=" + to_string(waypoint.y) + " z=" + to_string(waypoint.z);
                text_info_pub.publish(text_info);

                // 等待模拟到达该点，这里使用暂停6秒
                ros::Duration(6.0).sleep();
            }
            // 重置开始命令状态
            received_start_cmd = false; 
            // 打印信息
            std_msgs::String end_info;
            end_info.data = "ending Moving";
            // 终端打印信息
            cout << GREEN << "ending Moving" << TAIL << endl;
            // 发布信息
            text_info_pub.publish(end_info);
        }
        // 处理一次回调函数
        ros::spinOnce();
        // 等待无人机行驶到目标点
        rate.sleep();
        
    }

    return 0;
}