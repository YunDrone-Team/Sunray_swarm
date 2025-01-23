// #include <ros/ros.h>
// #include "printf_utils.h"
// #include "ros_msg_utils.h"

// using namespace std;

// ros::Publisher agent_cmd_pub;           // 发布控制命令
// vector<geometry_msgs::Point> waypoints; // 航点列表
// int agent_id;                           // 代理编号
// float agent_height;                     // 无人机飞行高度
// bool received_start_cmd = false;        // 标记是否接收到开始命令
// ros::Publisher text_info_pub;           // 发布文字提示消息
// ros::Subscriber single_waypoint_sub;    // 订阅路径规划触发信号
// string node_name;
// ros::Publisher takeoff_pub;
// ros::Publisher land_pub;

// std_msgs::Empty takeoff;
// std_msgs::Empty land;

// // 触发信号的回调函数，处理接收到的位置
// void single_waypoint_cb(const std_msgs::Bool::ConstPtr &msg)
// {
//     received_start_cmd = msg->data; // 设置输入信息
// }

// int main(int argc, char **argv)
// {
//     // 初始化ROS节点
//     ros::init(argc, argv, "rmtt_waypoint");
//     // 创建节点句柄
//     ros::NodeHandle nh;
//     // 设置循环频率为10Hz
//     ros::Rate rate(10);
//     // 【参数】智能体高度
//     nh.param<float>("agent_height", agent_height, 1.0f);
//     // 【参数】智能体编号
//     nh.param<int>("agent_id", agent_id, 1);

//     string agent_name;
//     agent_name = "/rmtt_" + std::to_string(agent_id);
//     // 【订阅】触发指令 外部 -> 本节点
//     single_waypoint_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/demo/rmtt_waypoint", 1, single_waypoint_cb);
//     // 【发布】控制指令 本节点 -> 控制节点
//     agent_cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 10);
//     // 【发布】文字提示消息  本节点 -> 地面站
//     text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);
//     // 【发布】无人机起飞指令 本节点 -> rmtt_driver
//     takeoff_pub = nh.advertise<std_msgs::Empty>("/sunray_swarm/" + agent_name + "/takeoff", 1); 
//     // 【发布】无人机降落指令 本节点 -> rmtt_driver
//     land_pub = nh.advertise<std_msgs::Empty>("/sunray_swarm/" + agent_name + "/land", 1); 
    
    

//     // 设置航点列表
//     // 提取存储的航点数
//     int waypoint_count;
//     nh.param<int>("waypoint_count", waypoint_count, 2); // 获取航点总数，默认为0
//     waypoints.resize(waypoint_count);
//     cout<< waypoint_count << endl;

//     for (int i = 0; i < waypoint_count; ++i)
//     {
//         // 构造参数名称
//         string param_base = "waypoint_" + to_string(i + 1); // 航点编号从1开始
//         nh.param<double>("waypoint_" + to_string(i + 1) + "_x", waypoints[i].x, 1.0);
//         nh.param<double>("waypoint_" + to_string(i + 1) + "_y", waypoints[i].y, 2.0);
//         waypoints[i].z = agent_height; // 高度使用智能体设置的高度
//     }

//     // 创建控制命令对象
//     sunray_msgs::agent_cmd cmd;
//     // 依据代理类型设置的ID
//     cmd.agent_id = 1;
//     // 设置为起飞状态
//     sunray_msgs::agent_cmd::TAKEOFF;
//     //发布起飞命令
//     takeoff_pub.publish(takeoff);
//     // 设置指令来源
//     cmd.cmd_source = "ugv_pathplaning";
//     // 发布话题
//     agent_cmd_pub.publish(cmd);
//     // 等待3秒以确保无人机起飞
//     ros::Duration(3.0).sleep(); 

//     // 从用户输入获取航点位置
//     while (ros::ok())
//     {
//         if (received_start_cmd)
//         {
//             // 发送开始导航信息
//             std_msgs::String start_info;
//             start_info.data = "Start Moving";
//             // 终端打印信息
//             cout << GREEN << "Start Moving" << TAIL << endl;
//             // 发布信息
//             text_info_pub.publish(start_info);
//             // 向每个航点发送导航命令
//             for (auto &waypoint : waypoints)
//             {
//                 cmd.desired_pos.z = agent_height; // 设置高度
//                 agent_cmd_pub.publish(cmd); // 发布起飞控制命令
//                 // 等待起飞完成（可以根据具体情况调整时间）
//                 ros::Duration(5.0).sleep();
//                 cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
//                 cmd.cmd_source = "rmtt_waypoint";
//                 cmd.desired_pos = waypoint;
//                 cmd.desired_yaw = 0.0; // 偏航角

//                 agent_cmd_pub.publish(cmd); // 发布控制命令

//                 // 发送提示消息
//                 std_msgs::String text_info;
//                 text_info.data = "Navigating to waypoint: x=" + to_string(waypoint.x) + " y=" + to_string(waypoint.y) + " z=" + to_string(waypoint.z);
//                 text_info_pub.publish(text_info);

//                 // 等待模拟到达该点，这里使用暂停6秒
//                 ros::Duration(6.0).sleep();
//             }
//             // 重置开始命令状态
//             // received_start_cmd = false; 
//         }
//         // 处理一次回调函数
//         ros::spinOnce();
//         // 等待无人机行驶到目标点
//         rate.sleep();
//     }

//     return 0;
// }


#include <ros/ros.h>
#include <signal.h>
#include "printf_utils.h"
#include "ros_msg_utils.h"


using namespace std;

ros::Publisher cmd_pub;                 // 发布控制命令
vector<geometry_msgs::Point> waypoints; // 航点列表
int agent_type;                         // 代理类型变量，用于区分不同类型（无人机或无人车）
float agent_height;                     //设置无人机高度
int agent_num;                          //设置无人机数量

// 信号处理函数，用于在接收到中断信号关闭节点
void mySigintHandler(int sig) {
    ROS_INFO("Shutting down agent waypoint control node...");
    ros::shutdown();
}
// 发布悬停位置的函数
void navigateToWaypoints() {
    sunray_msgs::agent_cmd cmd;
    cmd.agent_id = 1; // 根据代理类型设置的ID
    // 设置控制状态为位置控制模式
    cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL; 
    // 遍历每个航点，发送导航命令
    for (auto& waypoint : waypoints) {
         // 设置当前航点为目标位置
        cmd.desired_pos = waypoint;
        // 通过发布者发布命令消息
        cmd_pub.publish(cmd);
        cout << BLUE << "Agent navigating to waypoint: x=" << waypoint.x << " y=" << waypoint.y << " z=" << waypoint.z << endl;
        // 模拟到达该点的等待时间，暂停6秒
        ros::Duration(6.0).sleep(); // 等待10秒以模拟到达该点
    }
}

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "agent_waypoint_control");
    // 创建节点句柄
    ros::NodeHandle nh;
    // 设置信号处理函数
    signal(SIGINT, mySigintHandler); 
    // 从参数服务器获取agent_type参数的值，默认为0
    nh.param<int>("agent_type", agent_type, 0); 
    // 【参数】智能体高度
    nh.param<float>("agent_height", agent_height, 1.0);
    // 【参数】智能体高度
    nh.param<int>("agent_num", agent_num, 1);
    // 定义一个字符串变量，用于存储代理前缀
    string agent_prefix;
    // 根据代理类型选择适当的前缀
    switch(agent_type) {
        case sunray_msgs::agent_state::RMTT:
            agent_prefix = "rmtt_";
            break;

    }
    string agent_name;
    for (int i = 0; i < agent_num; i++)
    {
        agent_name = "/" + agent_prefix + std::to_string(i+1);
        // 初始化发布者
        cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 1);

    }

        // [订阅]触发条件
    // agent_cmd_pub = nh.advertise<std_msgs::Bool>("/sunray_swarm/single_waypoint", 1， start_cmd_cb);

    // 定义航点位置变量
    float x, y, z;
    // 用户输入选择变量
    char choice;
    // 从用户输入获取航点位置
    while (ros::ok()) {
        cout << GREEN << "Enter waypoint position (x, y"  << "): ";
        if (!(cin >> x >> y)) {
            cin.clear();
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
            cout << RED << "Invalid x or y coordinate input." << endl;
            continue;
        }
        if (agent_type == sunray_msgs::agent_state::RMTT || agent_type == sunray_msgs::agent_state::SIKONG) {
            if (!(cin >> z)) {
                // 清除输入错误状态
                cin.clear();
                // 忽略剩余输入
                cin.ignore(numeric_limits<streamsize>::max(), '\n');
                cout << RED << "Invalid z coordinate, setting z to default 1.0" << endl;
                z = agent_height;  // 默认高度为无人机
            }
        } else {
            z = 0.0;  // 无人车默认高度
        }
        // 将输入的航点位置保存到waypoints列表中
        geometry_msgs::Point point;
        point.x = x;
        point.y = y;
        point.z = z;
        waypoints.push_back(point);
        // 提示用户是否添加更多航点或开始导航
        cout << GREEN << "Press 'n' to add another waypoint, or 's' to start navigating: ";
        cin >> choice;
        if (choice == 's' || choice == 'S') {
            // 开始导航到航点
            navigateToWaypoints();
            // 清空航点列表
            waypoints.clear();
        }
        // 如果用户输入无效
        else if(choice != 'n' && choice != 'N') {
            // 清除输入错误状态
            cin.clear();
            // 忽略剩余输入
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
            cout << RED << "Invalid input, exiting program." << endl;
        }
    }

    return 0;
}