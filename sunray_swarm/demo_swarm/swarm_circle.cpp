#include <ros/ros.h>
#include <signal.h>

#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"

#define MAX_AGENT_NUM 100

using namespace std;
int agent_type;                                  // 智能体类型
int agent_num;                                   // 智能体数量
float desired_yaw;                               // 期望偏航角
int start_cmd = 0;                               // 启动命令
float time_trajectory = 0.0;                     // 当前轨迹时间
float trajectory_total_time;                     // 轨迹总时间
sunray_msgs::agent_cmd agent_cmd[MAX_AGENT_NUM]; // 智能体控制命令数组
float angle[MAX_AGENT_NUM];                      // 存储每个智能体的当前角度
float cos_angle;                                 // 当前角度的余弦值
float sin_angle;                                 // 当前角度的正弦值
float circle_radius;                             // 圆的半径
float linear_vel;                                // 线速度
float omega;                                     // 角速度
float direction;                                 // 方向，1或-1
Eigen::Vector3f circle_center;                   // 圆心坐标
int agent_time;                                  // 每次画圆的持续时间
bool received_start_cmd = false;                 // 标记是否接收到开始命令

// 发布者和订阅者
ros::Publisher ugv_cmd_pub[MAX_AGENT_NUM]; // 每个智能体的控制命令发布者
ros::Publisher text_info_pub;              // 文字提示消息发布者
ros::Subscriber swarm_circle_cmd_sub;      // 订阅者，用于接收画圈指令

// 信号处理函数
void mySigintHandler(int sig)
{
    ROS_INFO("[circle_trajectory] exit...");
    ros::shutdown();
}
// 处理画圈命令的回调函数
void swarm_circle_cb(const std_msgs::Bool::ConstPtr &msg)
{
    received_start_cmd = msg->data; // 设置接收到的开始命令
}
// 打印参数值的函数
void printf_params()
{
    cout << GREEN << "agent_type    : " << agent_type << "" << TAIL << endl;
    cout << GREEN << "agent_num     : " << agent_num << "" << TAIL << endl;
    cout << GREEN << "desired_yaw   : " << desired_yaw << "" << TAIL << endl;
    cout << GREEN << "circle_center_x : " << circle_center[0] << "" << TAIL << endl;
    cout << GREEN << "circle_center_y : " << circle_center[1] << TAIL << endl;
    cout << GREEN << "agent_height  : " << circle_center[2] << "" << TAIL << endl;
    cout << GREEN << "circle_radius : " << circle_radius << "" << TAIL << endl;
    cout << GREEN << "linear_vel    : " << linear_vel << "" << TAIL << endl;
    cout << GREEN << "direction     : " << direction << "" << TAIL << endl;
    cout << GREEN << "omega         : " << omega << "" << TAIL << endl;
}
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "circle_trajectory");
    // 创建节点句柄
    ros::NodeHandle nh("~");
    // 设置循环频率为100Hz
    ros::Rate rate(100.0);

    // 【参数】智能体类型 智能体类型，默认为0
    nh.param<int>("agent_type", agent_type, 0);
    // 【参数】智能体编号 智能体数量，默认为8
    nh.param<int>("agent_num", agent_num, 8);
    // 【参数】desired_yaw 期望偏航角，默认为0
    nh.param<float>("desired_yaw", desired_yaw, 0.0f);
    // 【参数】圆心X 圆心X坐标
    nh.param<float>("circle_center_x", circle_center[0], 0.0f);
    // 【参数】圆心Y 圆心Y坐标
    nh.param<float>("circle_center_y", circle_center[1], 0.0f);
    // 【参数】智能体高度
    nh.param<float>("agent_height", circle_center[2], 1.0f);
    // 【参数】半径
    nh.param<float>("circle_radius", circle_radius, 1.0f);
    // 【参数】线速度
    nh.param<float>("linear_vel", linear_vel, 0.3f);
    // 【参数】圆的方向 1或-1
    nh.param<float>("direction", direction, 1.0f);
    // 【参数】从参数服务器获取智能画圆时间，默认为10
    nh.param<int>("agent_time", agent_time, 10);
    // 计算角速度
    if (circle_radius != 0)
    {
        omega = direction * fabs(float(linear_vel / circle_radius));
    }
    else
    {
        // 半径为0时，角速度为0
        omega = 0.0;
    }
    // 打印参数值
    printf_params();
    // 智能体名称前缀
    string agent_prefix;
    // 根据智能体类型设置名称前缀

    if (agent_type == sunray_msgs::agent_state::RMTT)
    {
        agent_prefix = "rmtt_";
    }
    else if (agent_type == sunray_msgs::agent_state::UGV)
    {
        agent_prefix = "ugv_";
    }
    else if (agent_type == sunray_msgs::agent_state::SIKONG)
    {
        agent_prefix = "sikong_";
    }
    else
    {
        agent_prefix = "unkonown_";
    }

    // 【发布】文字提示消息（回传至地面站显示）
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);
    // 【订阅】触发指令 外部-> 本节点  初始化开始命令的订阅者，监听特定的触发信号
    swarm_circle_cmd_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/demo/swarm_circle", 1, swarm_circle_cb);
    // 创建控制命令发布者
    string agent_name;
    for (int i = 0; i < agent_num; i++)
    {
        // 生成智能体名称
        agent_name = "/" + agent_prefix + std::to_string(i + 1);
        // 【发布】无人车控制指令
        ugv_cmd_pub[i] = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm/rmtt/agent_cmd", 1);
    }
    // 主循环
    while (ros::ok())
    {
        // 检查是否接收到启动命令
        if (received_start_cmd)
        {
            // 重置轨迹时间
            time_trajectory = 0.0;
            // 计算第一个智能体的初始角度
            angle[0] = time_trajectory * omega;
            // 初始化各个智能体的角度
            for (int i = 1; i < agent_num; i++)
            {
                // 计算每个智能体的初始角度
                angle[i] = angle[0] - i * 2 * M_PI / agent_num;
            }
            // 发布初始控制命令

            for (int i = 0; i < agent_num; i++)
            {
                cos_angle = cos(angle[i]); // 计算余弦值
                sin_angle = sin(angle[i]); // 计算正弦值
                // 设置智能体ID
                agent_cmd[i].agent_id = i + 1;
                // 设置控制状态为位置控制
                agent_cmd[i].control_state = sunray_msgs::agent_cmd::POS_CONTROL;
                // 设置命令来源
                agent_cmd[i].cmd_source = "circle_trajectory";
                // 计算目标X坐标
                agent_cmd[i].desired_pos.x = circle_radius * cos_angle + circle_center[0];
                // 计算目标Y坐标
                agent_cmd[i].desired_pos.y = circle_radius * sin_angle + circle_center[1];
                // 设置目标Z坐标
                agent_cmd[i].desired_pos.z = circle_center[2];
                // 设置期望偏航角
                agent_cmd[i].desired_yaw = 0.0;
                // 发布控制命令
                ugv_cmd_pub[i].publish(agent_cmd[i]);
            }
            // 等待1秒以确保智能体开始移动
            sleep(1.0);
            // 进行10秒的画圆操作
            while (time_trajectory < agent_time)
            {
                // 更新第一个智能体的角度
                float cos_angle;
                float sin_angle;
                angle[0] = time_trajectory * omega;
                // 更新其他智能体的角度
                for (int i = 1; i < agent_num; i++)
                {
                    angle[i] = angle[0] - i * 2 * M_PI / agent_num;
                }
                // 发布控制命令
                for (int i = 0; i < agent_num; i++)
                {
                    cos_angle = cos(angle[i]); // 计算余弦值
                    sin_angle = sin(angle[i]); // 计算正弦值
                                               // 设置智能体ID
                    agent_cmd[i].agent_id = i + 1;
                    // 设置控制状态
                    agent_cmd[i].control_state = sunray_msgs::agent_cmd::POS_CONTROL;
                    // 设置命令来源
                    agent_cmd[i].cmd_source = "circle_trajectory";
                    // 计算目标X坐标
                    agent_cmd[i].desired_pos.x = circle_radius * cos_angle + circle_center[0];
                    // 计算目标Y坐标
                    agent_cmd[i].desired_pos.y = circle_radius * sin_angle + circle_center[1];
                    // 设置目标Z坐标
                    agent_cmd[i].desired_pos.z = circle_center[2];
                    // 设置期望偏航角
                    agent_cmd[i].desired_yaw = 0.0;
                    // 发布控制命令
                    ugv_cmd_pub[i].publish(agent_cmd[i]);
                }
                // 增加时间
                time_trajectory = time_trajectory + 0.01;
                // 输出当前追踪时间
                cout << GREEN << "Trajectory tracking: " << time_trajectory << " / " << trajectory_total_time << " [ s ]" << TAIL << endl;
                // 每次循环暂停0.01秒
                ros::Duration(0.01).sleep();
            }
            // 重置启动命令
            received_start_cmd = false;
        }
        // 回调函数,timer开始运行
        ros::spinOnce();
        // 根据设定频率休眠，保持稳定的循环频率
        rate.sleep();
    }
    return 0;
}
