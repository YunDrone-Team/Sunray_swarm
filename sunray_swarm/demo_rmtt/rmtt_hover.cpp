#include <ros/ros.h>
#include "ros_msg_utils.h"
#include "printf_utils.h"


int agent_id;                        // 设置智能体编号
float hover_yaw;                     // 目标偏航角
string node_name;                    // 节点名称
geometry_msgs::Point position_hover; // 初始化位置
bool received_start_cmd = false;     // 接收到开始命令

std_msgs::Empty takeoff;
std_msgs::Empty land;

ros::Publisher agent_cmd_pub;     // 发布控制命令
ros::Publisher text_info_pub;     // 发布文字提示消息
ros::Subscriber single_hover_sub; // 触发条件
ros::Time last_command_time;      // 上次命令时间

ros::Publisher takeoff_pub;
ros::Publisher land_pub;

//TODO——起飞悬停设置
// void set_desired_position()
// {
//     desired_pos.x = agent_state.pos[0];
//     desired_pos.y = agent_state.pos[1];
//     desired_pos.z = agent_height;
// }

// 触发信号的回调函数，处理接收到的位置
void single_hover_cb(const std_msgs::Bool::ConstPtr &msg)
{
    received_start_cmd = true; // 设置标志为true
    last_command_time = ros::Time::now(); // 更新最后命令时间
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rmtt_hover");
    ros::NodeHandle nh("~");
    // 设置循环频率为10Hz
    ros::Rate rate(10);
    node_name = ros::this_node::getName();

    // 【参数】智能体编号
    nh.param<int>("agent_id", agent_id, 1);
    // 【参数】目标点位置——TODO：从参数服务器获取位置x、y、z、yaw偏航角
    nh.param<double>("hover_x", position_hover.x, 1.0);
    nh.param<double>("hover_y", position_hover.y, 2.0f);
    nh.param<double>("hover_z", position_hover.z, 1.0f);
    nh.param<float>("hover_yaw", hover_yaw, 0.0f);

    string agent_name;
    agent_name = "/rmtt_" + std::to_string(agent_id);
    // 【订阅】触发指令 外部 -> 本节点
    single_hover_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/demo/rmtt_hover", 1, single_hover_cb);
    // 【发布】控制指令 本节点 -> 控制节点
    agent_cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 10);
    // 【发布】文字提示消息  本节点 -> 地面站
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);
    // 【发布】无人机起飞指令 本节点 -> rmtt_driver
    takeoff_pub = nh.advertise<std_msgs::Empty>("/sunray_swarm/" + agent_name + "/takeoff", 1); 
    // 【发布】无人机降落指令 本节点 -> rmtt_driver
    land_pub = nh.advertise<std_msgs::Empty>("/sunray_swarm/" + agent_name + "/land", 1); 


    // 构建并发送悬停指令
    sunray_msgs::agent_cmd cmd;
    // 设置agent_id
    cmd.agent_id = agent_id;
    // 设置为起飞状态
    sunray_msgs::agent_cmd::TAKEOFF;
    //发布起飞命令
    takeoff_pub.publish(takeoff);
    // 设置指令来源
    cmd.cmd_source = "rmtt_hover";
    // 发布话题
    agent_cmd_pub.publish(cmd);
    // 等待3秒以确保无人机起飞
    ros::Duration(3.0).sleep(); 


    // 主程序
    while (ros::ok())
    {
        if (received_start_cmd)
        {
            // 起飞标志
            bool received_takeoff_flag = false;
            // 发送开始信息
            std_msgs::String start_info;
            start_info.data = "start hover";
            // 终端打印信息
            cout << GREEN << "start hover" << TAIL << endl;
            // 发布信息
            text_info_pub.publish(start_info);

            // 设置控制指令为POS_CONTROL
            cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;


            // 设置目标位置为接收到的位置
            cmd.desired_pos = position_hover;
            // 可以设置偏航角，这里设置为0
            cmd.desired_yaw = hover_yaw;
            // 发布话题
            agent_cmd_pub.publish(cmd);
            // 记录命令时间,用于判断15无操作降落时间
            last_command_time = ros::Time::now(); 
            // 地面站打印
            std_msgs::String text_info;
            text_info.data = node_name + ": send a new hover position!";
            text_info_pub.publish(text_info);
            // 控制台打印
            cout << GREEN << "POS_REF [X Y Z] : " << cmd.desired_pos.x << " m " << cmd.desired_pos.y << " m " << cmd.desired_pos.z << " m " << TAIL << endl;
            
            // 重置标志
            received_start_cmd = false;

        }
        // 处理回调函数
        ros::spinOnce();
        // 休眠
        rate.sleep();
    }

    return 0;
}