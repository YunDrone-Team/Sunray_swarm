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
float circle_radius;                             // 圆的半径
Eigen::Vector3f circle_center;                   // 圆心坐标
float omega;                                     // 角速度
bool demo_start_flag = false;                 // 标记是否接收到开始命令
sunray_msgs::orca_cmd orca_cmd;                         // ORCA指令

float time_trajectory = 0.0;                     // 当前轨迹时间
float trajectory_total_time;                     // 轨迹总时间
sunray_msgs::agent_cmd agent_cmd[MAX_AGENT_NUM]; // 智能体控制命令数组
float angle[MAX_AGENT_NUM];                      // 存储每个智能体的当前角度
float cos_angle;                                 // 当前角度的余弦值
float sin_angle;                                 // 当前角度的正弦值
float linear_vel;                                // 线速度
float direction;                                 // 方向，1或-1
int agent_time;                                  // 每次画圆的持续时间




ros::Publisher orca_goal_pub[MAX_AGENT_NUM];     // 目标点发布者
ros::Subscriber orca_state_sub[MAX_AGENT_NUM];   // ORCA状态订阅者
ros::Subscriber swarm_circle_cmd_sub;            // 画圈命令的订阅者
sunray_msgs::orca_state orca_state[MAX_AGENT_NUM];      // ORCA状态
ros::Publisher orca_cmd_pub;                        // 发布ORCA指令

// 信号处理函数
void mySigintHandler(int sig) {
    ROS_INFO("[circle_trajectory] exit...");
    ros::shutdown();
}

// 处理ORCA状态回调
void rmtt_orca_state_cb(const sunray_msgs::orca_stateConstPtr& msg, int i) {
    // 更新指定智能体的ORCA状态
    orca_state[i] = *msg;
}

// 处理画圈命令的回调函数
void swarm_circle_cb(const std_msgs::Bool::ConstPtr &msg) {
    demo_start_flag = msg->data; // 设置接收到的开始命令
    // 设置ORCA命令为HOME
    orca_cmd.orca_cmd = sunray_msgs::orca_cmd::SET_HOME;
    // 发布命令
    orca_cmd_pub.publish(orca_cmd);
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
    nh.param<int>("agent_time", agent_time, 20);

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

    // 根据智能体类型设置名称前缀
    string agent_prefix;
    if (agent_type == sunray_msgs::agent_state::RMTT)
    {
        agent_prefix = "rmtt_";
    }
    else if (agent_type == sunray_msgs::agent_state::UGV)
    {
        agent_prefix = "ugv_";
    }
    else
    {
        agent_prefix = "unkonown_";
    }
    // 【发布】ORCA指令 本节点 ->  ORCA
    orca_cmd_pub = nh.advertise<sunray_msgs::orca_cmd>("/sunray_swarm/" + agent_prefix + "/orca_cmd", 1);
    string agent_name;
    for (int i = 0; i < agent_num; i++) {
        agent_name = "/" + agent_prefix + std::to_string(i + 1);
        orca_goal_pub[i] = nh.advertise<geometry_msgs::Point>("/sunray_swarm" + agent_name + "/goal_point", 1);
        orca_state_sub[i] = nh.subscribe<sunray_msgs::orca_state>("/sunray_swarm" + agent_name + "/agent_orca_state", 1, boost::bind(&rmtt_orca_state_cb, _1, i));
    }
    swarm_circle_cmd_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/demo/swarm_circle", 1, swarm_circle_cb);

    while (ros::ok()) {
        if (demo_start_flag) {
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
            for (int i = 0; i < agent_num; i++) {
                float angle = i * 2 * M_PI / agent_num;
                geometry_msgs::Point goal_point;
                goal_point.x = circle_center[0] + circle_radius * cos(angle);
                goal_point.y = circle_center[1] + circle_radius * sin(angle);
                goal_point.z = circle_center[2];
                orca_goal_pub[i].publish(goal_point);

            }
            sleep(10);
            ros::Time start_time;
            start_time = ros::Time::now(); // 记录开始时间
            while ((ros::Time::now() - start_time).toSec() < agent_time) {
                for (int i = 0; i < agent_num; i++) {
                    float angle = omega * (ros::Time::now() - start_time).toSec() + i * 2 * M_PI / agent_num;
                    geometry_msgs::Point goal_point;
                    goal_point.x = circle_center[0] + circle_radius * cos(angle);
                    goal_point.y = circle_center[1] + circle_radius * sin(angle);
                    goal_point.z = circle_center[2];
                    orca_goal_pub[i].publish(goal_point);
                }
                ros::spinOnce();
                rate.sleep();
            }
            demo_start_flag = false;
        }
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

