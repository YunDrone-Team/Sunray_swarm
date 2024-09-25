#include <ros/ros.h>
#include "printf_utils.h"
#include "ros_msg_utils.h"


using namespace std;

ros::Publisher agent_cmd_pub; // 发布控制命令
vector<geometry_msgs::Point> waypoints; // 航点列表
int agent_type; // 代理类型变量，用于区分不同类型（无人机或无人车）
float agent_height;//设置无人机高度
int agent_id;
bool received_start_cmd = false;     // 接收到开始命令s
string node_name;
ros::Publisher text_info_pub; // 发布文字提示消息
ros::Subscriber single_waypoint_sub;



// 触发信号的回调函数，处理接收到的位置
void single_waypoint_cb(const std_msgs::Bool::ConstPtr &msg) 
{
    received_start_cmd = msg->data; // 设置输入信息
}

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "rmtt_waypoint");
    // 创建节点句柄
    ros::NodeHandle nh;
    // 设置循环频率为10Hz
    ros::Rate rate(10);
    // 【参数】智能体高度
    nh.param<float>("agent_height", agent_height, 1.0f);
    // 【参数】智能体编号
    nh.param<int>("agent_id", agent_id, 1);
    // 【参数】目标点位置——TODO
    // nh.param<int>("agent_goals", agent_goals, 1);

    string agent_name;
    agent_name = "/rmtt_" + std::to_string(agent_id);
    // 【订阅】触发指令 外部 -> 本节点 ——TODO设置为BOOL值变量
    single_waypoint_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/demo/single_waypoint", 1, single_waypoint_cb);
    // 【发布】控制指令 本节点 -> 控制节点
    agent_cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 10);
    // 【发布】文字提示消息  本节点 -> 地面站
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);

    // 定义航点位置变量
    float x, y, z;
    // 用户输入选择变量
    char choice;
    // 从用户输入获取航点位置
    while (ros::ok()) 
    {
        if(received_start_cmd)
        {
            cout << GREEN << "Enter target position (x y): ";
            cin >> x >> y;
            // 从参数服务器获取z
            z = agent_height;
            
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
                sunray_msgs::agent_cmd cmd;
                cmd.agent_id = 1; // 根据代理类型设置的ID
                // 设置控制状态为位置控制模式
                cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL; 
                // 设置指令来源
                cmd.cmd_source = "single_pathplanning";
                // 遍历每个航点，发送导航命令
                for (auto& waypoint : waypoints) {
                    // 设置当前航点为目标位置
                    cmd.desired_pos = waypoint;
                    // 通过发布者发布命令消息
                    agent_cmd_pub.publish(cmd);
                    cout << BLUE << "Agent navigating to waypoint: x=" << waypoint.x << " y=" << waypoint.y << " z=" << waypoint.z << endl;
                    // 模拟到达该点的等待时间，暂停6秒
                    ros::Duration(6.0).sleep(); // 等待10秒以模拟到达该点
                }
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
        // 处理一次回调函数
        ros::spinOnce();
        // 等待无人机行驶到目标点
        rate.sleep();
        
    }

    return 0;
}