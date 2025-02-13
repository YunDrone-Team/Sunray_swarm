
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sunray_msgs/agent_cmd.h> 

using namespace std;

// 全局变量
ros::Publisher agent_cmd_pub;           // 发布控制命令
vector<geometry_msgs::Point> waypoints; // 航点列表
int agent_id;                           // 代理编号
float agent_height;                     // 无人机飞行高度
bool received_start_cmd = false;        // 标记是否接收到开始命令
ros::Publisher text_info_pub;           // 发布文字提示消息
ros::Subscriber single_waypoint_sub;    // 订阅路径规划触发信号
ros::Publisher takeoff_pub;             // 发布起飞指令
ros::Publisher land_pub;                // 发布降落指令

std_msgs::Empty takeoff; // 起飞命令
std_msgs::Empty land;    // 降落命令

// 触发信号的回调函数
void single_waypoint_cb(const std_msgs::Bool::ConstPtr &msg)
{
    // 设置输入信息
    received_start_cmd = msg->data; 
    ROS_INFO("Received start command: %d", received_start_cmd);
}

// 主函数
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "rmtt_waypoint");
    ros::NodeHandle nh("~");

    // 设置循环频率为10Hz
    ros::Rate rate(10);

    // 从参数服务器获取参数
    nh.param<float>("agent_height", agent_height, 1.0f); // 默认高度为1.0米
    nh.param<int>("agent_id", agent_id, 1);              // 默认无人机编号为1

    // 构造代理名称
    string agent_name = "/rmtt_" + to_string(agent_id);

    // 【订阅】触发指令 外部 -> 本节点
    single_waypoint_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/demo/rmtt_waypoint", 1, single_waypoint_cb);

    // 【发布】控制指令 本节点 -> 控制节点
    agent_cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 10);

    // 【发布】文字提示消息  本节点 -> 地面站
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);

    // 【发布】无人机起飞指令 本节点 -> rmtt_driver
    takeoff_pub = nh.advertise<std_msgs::Empty>("/sunray_swarm/" + agent_name + "/takeoff", 1);

    // 【发布】无人机降落指令 本节点 -> rmtt_driver
    land_pub = nh.advertise<std_msgs::Empty>("/sunray_swarm/" + agent_name + "/land", 1);

    // 初始化 waypoints 的大小
    int waypoint_count = 3; // 设置 3 个航点
    waypoints.resize(waypoint_count); // 初始化 waypoints 的大小

    // 从参数服务器加载航点数据
    // nh.param<double>("waypoint_1_x", waypoints[0].x, 1.0);
    // nh.param<double>("waypoint_1_y", waypoints[0].y, 2.0);
    // waypoints[0].z = agent_height; // 高度使用智能体设置的高度

    // nh.param<double>("waypoint_2_x", waypoints[1].x, 2.0);
    // nh.param<double>("waypoint_2_y", waypoints[1].y, 2.0);
    // waypoints[1].z = agent_height; // 高度使用智能体设置的高度

    // nh.param<double>("waypoint_3_x", waypoints[2].x, 3.0);
    // nh.param<double>("waypoint_3_y", waypoints[2].y, 2.0);
    // waypoints[2].z = agent_height; // 高度使用智能体设置的高度

    for (int i = 0; i < waypoint_count; ++i)
    {
        // 构造参数名称
        // string param_base = "waypoint_" + to_string(i + 1); // 航点编号从1开始
        nh.param<double>("waypoint_" + to_string(i + 1) + "_x", waypoints[i].x, 1.0);
        nh.param<double>("waypoint_" + to_string(i + 1) + "_y", waypoints[i].y, 2.0);
        waypoints[i].z = agent_height; // 高度使用智能体设置的高度
    }

    // 打印航点信息
    // for (int i = 0; i < waypoint_count; ++i)
    // {
    //     ROS_INFO("Waypoint %d: x=%.2f, y=%.2f, z=%.2f", i + 1, waypoints[i].x, waypoints[i].y, waypoints[i].z);
    // }

    // 创建控制命令对象
    sunray_msgs::agent_cmd cmd;
    // 依据代理类型设置的ID
    cmd.agent_id = agent_id;
    // 设置指令来源
    cmd.cmd_source = "rmtt_waypoint";

    // 主循环
    while (ros::ok())
    {
        // 如果接收到开始命令
        if (received_start_cmd)
        {
            // 发送起飞命令
            takeoff_pub.publish(takeoff);
            ROS_INFO("Takeoff command sent.");
            ros::Duration(5.0).sleep(); // 等待无人机起飞

            // 发送开始导航信息
            std_msgs::String start_info;
            start_info.data = "Start Moving";
            // 发布信息
            text_info_pub.publish(start_info);
            // 终端打印信息
            ROS_INFO("Start Moving");

            // 依次飞向每个航点
            for (auto &waypoint : waypoints)
            {
                // 设置目标位置
                cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
                cmd.desired_pos = waypoint;
                cmd.desired_yaw = 0.0; // 偏航角

                // 发布控制命令
                agent_cmd_pub.publish(cmd);
                ROS_INFO("Navigating to waypoint: x=%.2f, y=%.2f, z=%.2f", waypoint.x, waypoint.y, waypoint.z);

                // 发送提示消息
                std_msgs::String text_info;
                text_info.data = "Navigating to waypoint: x=" + to_string(waypoint.x) + " y=" + to_string(waypoint.y) + " z=" + to_string(waypoint.z);
                text_info_pub.publish(text_info);

                // 等待无人机到达目标点
                ros::Duration(6.0).sleep();
            }

            // 发送降落命令
            land_pub.publish(land);
            ROS_INFO("Land command sent.");

            // 重置开始命令状态
            received_start_cmd = false;
        }

        // 处理回调函数
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}