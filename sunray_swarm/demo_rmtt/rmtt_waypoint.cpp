#include <ros/ros.h>
#include <signal.h>
#include <vector>
#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"


using namespace std;

ros::Publisher cmd_pub; // 发布控制命令
vector<geometry_msgs::Point> waypoints; // 航点列表
int agent_type; // 代理类型变量，用于区分不同类型（无人机或无人车）
float agent_height;//设置无人机高度
int agent_num;

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
        cout << GREEN << "Enter waypoint position (x, y" << ((agent_type == sunray_msgs::agent_state::TIANBOT || agent_type == sunray_msgs::agent_state::WHEELTEC) ? "" : ", z") << "): ";
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