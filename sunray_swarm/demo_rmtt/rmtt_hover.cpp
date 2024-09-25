#include <ros/ros.h>
#include <signal.h>

#include "ros_msg_utils.h"
#include "printf_utils.h"

int agent_id; // 设置智能体编号
int agent_goals;
string node_name;                    // 节点名称
geometry_msgs::Point position_hover; // 初始化位置
bool received_start_cmd = false;     // 接收到开始命令

ros::Publisher agent_cmd_pub;     // 发布控制命令
ros::Publisher text_info_pub;     // 发布文字提示消息
ros::Subscriber single_hover_sub; // 触发条件

// 触发信号的回调函数，处理接收到的位置
void single_hover_cb(const geometry_msgs::Point::ConstPtr &msg)
{
    received_start_cmd = true; // 设置标志为true
    position_hover = *msg;     // 更新接收到的悬停位置信息
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "single_hover");
    ros::NodeHandle nh;
    // 设置循环频率为10Hz
    ros::Rate rate(10);
    node_name = ros::this_node::getName();

    // 【参数】智能体编号
    nh.param<int>("agent_id", agent_id, 1);
    // 【参数】目标点位置——TODO
    nh.param<int>("agent_goals", agent_goals, 1);

    string agent_name;
    agent_name = "/rmtt_" + std::to_string(agent_id);
    // 【订阅】触发指令 外部 -> 本节点 ——TODO设置为BOOL值变量
    single_hover_sub = nh.subscribe<geometry_msgs::Point>("/sunray_swarm/demo/single_hover", 1, single_hover_cb);
    // 【发布】控制指令 本节点 -> 控制节点
    agent_cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 10);
    // 【发布】文字提示消息  本节点 -> 地面站
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);

    while (ros::ok())
    {
        if (received_start_cmd)
        {
            // 起飞标志
            bool received_takeoff_flag = false;
            sunray_msgs::agent_cmd cmd;
            // 判断无人机是否起飞
            if (cmd.control_state == 11 && received_takeoff_flag)
            {
                // 设置agent_id
                cmd.agent_id = agent_id;
                // 设置控制指令为POS_CONTROL
                cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
                // 设置指令来源
                cmd.cmd_source = "single_hover";
                // 设置目标位置为接收到的位置
                cmd.desired_pos = position_hover;
                // 可以设置偏航角，这里设置为0
                cmd.desired_yaw = 0;
                // 发布话题
                agent_cmd_pub.publish(cmd);

                // 地面站打印
                std_msgs::String text_info;
                text_info.data = node_name + ": send a new hover position!";
                text_info_pub.publish(text_info);
                // 控制台打印
                cout << GREEN << "POS_REF [X Y Z] : " << cmd.desired_pos.x << " [ m ] " << cmd.desired_pos.y << " [ m ] " << cmd.desired_pos.z << " [ m ] " << TAIL << endl;

                // 重置标志
                received_start_cmd = false;
            }
            else
            {
                // 设置无人机降落
                cmd.control_state = 11;
                // 设置agent_id
                cmd.agent_id = agent_id;

                // 发布话题
                agent_cmd_pub.publish(cmd);
                received_takeoff_flag = true;
            }
        }

        // 处理回调函数
        ros::spinOnce();
        // 休眠
        rate.sleep();
    }

    return 0;
}