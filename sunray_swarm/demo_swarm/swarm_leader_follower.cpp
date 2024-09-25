#include <ros/ros.h>
#include <signal.h>
#include <vector>
#include "geometry_msgs/Point.h"
#include "sunray_msgs/agent_cmd.h"
#include "std_msgs/String.h"
#include "math_utils.h"


#define MAX_AGENT_NUM 3   

using namespace std;
int agent_type; // 代理类型，用于区分无人机和无人车
ros::Subscriber agent_cmd_sub;//触发条件


int agent_num = MAX_AGENT_NUM;
float agent_height;
ros::Publisher agent_cmd_pub[MAX_AGENT_NUM];
ros::Publisher text_info_pub;
geometry_msgs::Point reference_point;
geometry_msgs::Point offset[MAX_AGENT_NUM];

// 主机轨迹生成函数（可以根据需要修改）
void generate_reference_trajectory(geometry_msgs::Point &point, double time)
{
    point.x = 1.0 * sin(0.1 * time);
    point.y = 1.0 * cos(0.1 * time);
    point.z = agent_height;
}

void mySigintHandler(int sig)
{
    ROS_INFO("[leader_follower_control] exit...");
    ros::shutdown();
}

void printf_params()
{
    cout << "Agent number  : " << agent_num << endl;
    cout << "Agent height  : " << agent_height << endl;
}

void setup_offsets()
{
    // 设置2号和3号无人机的偏移量
    offset[1].x = 0.4; offset[1].y = 0.7; offset[1].z = 0.0;
    offset[2].x = -0.4; offset[2].y = 0.7; offset[2].z = 0.0;
}

void startCmdCallback(const std_msgs::Bool::ConstPtr& msg) {
    ros::Time start_time = ros::Time::now();
    if (msg->data) {
        while (ros::ok())
        {
            ros::Time current_time = ros::Time::now();
            double elapsed_time = (current_time - start_time).toSec();
            ros::Rate rate(10);  //10 Hz

            // 生成1号机的参考轨迹
            generate_reference_trajectory(reference_point, elapsed_time);

            // 发布1号机的控制指令
            sunray_msgs::agent_cmd agent_cmd[MAX_AGENT_NUM];
            agent_cmd[0].agent_id = 1;
            agent_cmd[0].control_state = sunray_msgs::agent_cmd::POS_CONTROL;
            agent_cmd[0].desired_pos = reference_point;
            agent_cmd[0].desired_yaw = 0.0;  // 假设固定yaw
            agent_cmd_pub[0].publish(agent_cmd[0]);

            // 2号和3号机跟随1号机
            for(int i = 1; i < agent_num; i++) 
            {
                agent_cmd[i].agent_id = i+1;
                agent_cmd[i].control_state = sunray_msgs::agent_cmd::POS_CONTROL;
                agent_cmd[i].desired_pos.x = reference_point.x + offset[i].x;
                agent_cmd[i].desired_pos.y = reference_point.y + offset[i].y;
                agent_cmd[i].desired_pos.z = reference_point.z + offset[i].z;
                agent_cmd[i].desired_yaw = 0.0;  // 假设固定yaw
                agent_cmd_pub[i].publish(agent_cmd[i]);
            }

            ros::spinOnce();
            rate.sleep();
        }
    } else {
        // 如果接收到的消息是false，也执行一些操作
        ROS_INFO("Received false signal, executing alternative task.");
        // 这里可以添加执行其他任务的代码
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "leader_follower_control");
    ros::NodeHandle nh("~");

    nh.param<float>("agent_height", agent_height, 1.0f);  // 默认飞行高度1米
    nh.param<int>("agent_type", agent_type, 1);  // 默认飞行高度1米


    printf_params();
    setup_offsets();

   // 定义一个字符串变量，用于存储代理前缀
    string agent_prefix;
    switch(agent_type) {
        case sunray_msgs::agent_state::RMTT:
            agent_prefix = "rmtt_";
            break;
        case sunray_msgs::agent_state::UGV:
            agent_prefix = "ugv_";
            break;
        case sunray_msgs::agent_state::SIKONG:
            agent_prefix = "sikong_";
            break;
        default:
            agent_prefix = "unknown_";
            break;
    }


    string agent_name;
    for(int i = 0; i < agent_num; i++) 
    {
        agent_name = "/" + agent_prefix + std::to_string(i+1);
        // 【发布】无人车控制指令
        agent_cmd_pub[i] = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 1);
    }
    // [订阅]触发条件
    agent_cmd_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/leader_follower", 1, startCmdCallback);
    
    // ros::Time start_time = ros::Time::now();

    while (ros::ok())
    {
        // ros::Time current_time = ros::Time::now();
        // double elapsed_time = (current_time - start_time).toSec();

        // // 生成1号机的参考轨迹
        // generate_reference_trajectory(reference_point, elapsed_time);

        // // 发布1号机的控制指令
        // sunray_msgs::agent_cmd agent_cmd[MAX_AGENT_NUM];
        // agent_cmd[0].agent_id = 1;
        // agent_cmd[0].control_state = sunray_msgs::agent_cmd::POS_CONTROL;
        // agent_cmd[0].desired_pos = reference_point;
        // agent_cmd[0].desired_yaw = 0.0;  // 假设固定yaw
        // agent_cmd_pub[0].publish(agent_cmd[0]);

        // // 2号和3号机跟随1号机
        // for(int i = 1; i < agent_num; i++) 
        // {
        //     agent_cmd[i].agent_id = i+1;
        //     agent_cmd[i].control_state = sunray_msgs::agent_cmd::POS_CONTROL;
        //     agent_cmd[i].desired_pos.x = reference_point.x + offset[i].x;
        //     agent_cmd[i].desired_pos.y = reference_point.y + offset[i].y;
        //     agent_cmd[i].desired_pos.z = reference_point.z + offset[i].z;
        //     agent_cmd[i].desired_yaw = 0.0;  // 假设固定yaw
        //     agent_cmd_pub[i].publish(agent_cmd[i]);
        // }

        ros::spinOnce();
        // 休眠0.1秒
        ros::Duration(0.1).sleep();
    }

    return 0;
}