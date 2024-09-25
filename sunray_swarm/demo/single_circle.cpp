#include <ros/ros.h>
#include "printf_utils.h"
#include "math_utils.h"
using namespace std;

ros::Publisher cmd_pub;      // 发布控制命令
float desired_yaw;           // 期望的偏航角
float circle_radius;         // 圆周轨迹的半径
float linear_vel;            // 线速度
float time_trajectory = 0.0; // 轨迹时间计数器
float trajectory_total_time; // 轨迹总时间
float omega;                 // 角速度

int start_cmd = 0;             // 开始命令标志
int agent_type;                // 代理类型，用于区分无人机和无人车
float agent_height;            // 设置无人机高度
int agent_num;                 // 设置无人机数量
int agent_id;                  // 设置无人机数量
ros::Subscriber agent_cmd_sub; // 触发条件
sunray_msgs::agent_cmd cmd;
bool received_start_cmd = false; // 设置开始点


string node_name;                       // 节点名称
ros::Publisher agent_cmd_pub;           // 发布控制命令
ros::Publisher text_info_pub;           // 发布文字提示消息
ros::Subscriber single_hover_sub;       // 触发条件

// 回调函数
void single_hover_cb(const std_msgs::Bool::ConstPtr &msg)
{
    received_start_cmd = msg->data; // 设置输入信息(trul、false)
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "single_circle");
    ros::NodeHandle nh;
    // 设置循环频率为10Hz
    ros::Rate rate(10);
    node_name = ros::this_node::getName();

    // 【参数】智能体编号
    nh.param<int>("agent_id", agent_id, 1);
    // 【参数】目标点位置——TODO
    nh.param<int>("agent_goals", agent_id, 1);

    string agent_name;
    agent_name = "/rmtt_" + std::to_string(agent_id);
    // 【订阅】触发指令 外部 -> 本节点 ——TODO设置为BOOL值变量
    single_hover_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/demo/single_circle", 1, single_hover_cb);
    // 【发布】控制指令 本节点 -> 控制节点
    agent_cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 10);
    // 【发布】文字提示消息  本节点 -> 地面站
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);

   
    while (ros::ok())
    {
        if (circle_radius != 0)
        {
            // 匀速圆周运动的角速度
            omega = linear_vel / circle_radius;
            if (received_start_cmd)
            {
                // ID 默认为1
                cmd.agent_id = 1;
                // 设置控制状态为位置控制模式
                cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
                // 设置起始位置的x坐标为圆周轨迹的半径
                cmd.desired_pos.x = circle_radius;
                // 设置起始位置的y坐标为0
                cmd.desired_pos.y = 0.0;
                // 根据代理类型设置高度
                cmd.desired_pos.z = agent_type == sunray_msgs::agent_state::RMTT || agent_type == sunray_msgs::agent_state::SIKONG ? agent_height : 0.0; // 高度设定
                // 设置起始位置的偏航角
                cmd.desired_yaw = desired_yaw;
                // 发布初始位置命令
                cmd_pub.publish(cmd);
                cout << GREEN << "Moving to start position: x=" << circle_radius << " y=0" << " z=" << cmd.desired_pos.z << " yaw=" << desired_yaw << TAIL << endl;
                // 等待5秒移动到目标点
                ros::Duration(5.0).sleep();
                // 设置起始时间
                double time_trajectory = 0.0;
                received_start_cmd = false;
            }
            else
            {
                if (time_trajectory < 20.0)
                {
                    float angle = time_trajectory * omega;
                    sunray_msgs::agent_cmd cmd;
                    cmd.agent_id = 1;
                    cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
                    cmd.desired_pos.x = circle_radius * cos(angle);
                    cmd.desired_pos.y = circle_radius * sin(angle);
                    cmd.desired_pos.z = agent_height;
                    cmd.desired_yaw = desired_yaw;
                    cmd_pub.publish(cmd);
                    cout << BLUE << "Moving in circle: x=" << cmd.desired_pos.x << " y=" << cmd.desired_pos.y << " z=" << cmd.desired_pos.z << " yaw=" << desired_yaw << TAIL << endl;
                    time_trajectory += 0.1;
                }
            }
        }
        else
        {
            // 如果半径为0，则角速度为0
            omega = 0.0;
        }

        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
