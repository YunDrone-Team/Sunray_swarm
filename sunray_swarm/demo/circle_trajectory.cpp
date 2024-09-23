#include <ros/ros.h>
#include <signal.h>

#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"

#define MAX_AGENT_NUM 100   
    
using namespace std;
int agent_type;
int agent_num;
float desired_yaw;
int start_cmd = 0;
float time_trajectory = 0.0;
float trajectory_total_time;
sunray_msgs::agent_cmd agent_cmd[MAX_AGENT_NUM];
float angle[MAX_AGENT_NUM];
float cos_angle;
float sin_angle;
float circle_radius;
float linear_vel;
float omega;
float direction;
Eigen::Vector3f circle_center;

ros::Publisher ugv_cmd_pub[MAX_AGENT_NUM];
ros::Publisher text_info_pub;

void mySigintHandler(int sig)
{
    ROS_INFO("[circle_trajectory] exit...");
    ros::shutdown();
}

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
    ros::init(argc, argv, "circle_trajectory");
    ros::NodeHandle nh("~");
    ros::Rate rate(100.0);

    // 【参数】智能体类型
    nh.param<int>("agent_type", agent_type, 0);
    // 【参数】智能体编号
    nh.param<int>("agent_num", agent_num, 8);
    // 【参数】desired_yaw
    nh.param<float>("desired_yaw", desired_yaw, 0.0f);
    // 【参数】圆心X
    nh.param<float>("circle_center_x", circle_center[0], 0.0f);
    // 【参数】圆心Y
    nh.param<float>("circle_center_y", circle_center[1], 0.0f);
    // 【参数】智能体高度
    nh.param<float>("agent_height", circle_center[2], 1.0f);
    // 【参数】半径
    nh.param<float>("circle_radius", circle_radius, 1.0f);
    // 【参数】线速度
    nh.param<float>("linear_vel", linear_vel, 0.3f);
    // 【参数】圆的方向 1或-1
    nh.param<float>("direction", direction, 1.0f);

    if( circle_radius != 0)
    {
        omega = direction * fabs(float(linear_vel / circle_radius));
    }else
    {
        omega = 0.0;
    }

    printf_params();

    string agent_prefix;

    if(agent_type == sunray_msgs::agent_state::RMTT)
    {
        agent_prefix = "rmtt_";
    }else if(agent_type == sunray_msgs::agent_state::UGV)
    {
        agent_prefix = "ugv_";
    }else if(agent_type == sunray_msgs::agent_state::SIKONG)
    {
        agent_prefix = "sikong_";
    }else
    {
        agent_prefix = "unkonown_";
    }

    // 【发布】文字提示消息（回传至地面站显示）
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);
    // 【订阅】程序触发指令
    // start_cmd_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/circle_trajectory", 1, start_cmd_cb);

    string agent_name;
    for(int i = 0; i < agent_num; i++) 
    {
        agent_name = "/" + agent_prefix + std::to_string(i+1);
        // 【发布】无人车控制指令
        ugv_cmd_pub[i] = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 1);
    }
	
    cout << GREEN << "Please enter 1 to move to start pos..." << TAIL << endl;
    cin >> start_cmd;


    time_trajectory = 0.0;
    angle[0] = time_trajectory * omega;

    for(int i = 1; i < agent_num; i++) 
    {
        angle[i] = angle[0] - i*2*M_PI/agent_num;
    }

    for(int i = 0; i < agent_num; i++) 
    {
        cos_angle = cos(angle[i]);
        sin_angle = sin(angle[i]);

        agent_cmd[i].agent_id = i+1;
        agent_cmd[i].control_state = sunray_msgs::agent_cmd::POS_CONTROL;
        agent_cmd[i].cmd_source = "circle_trajectory";
        agent_cmd[i].desired_pos.x = circle_radius * cos_angle + circle_center[0];
        agent_cmd[i].desired_pos.y = circle_radius * sin_angle + circle_center[1];
        agent_cmd[i].desired_pos.z = circle_center[2];
        agent_cmd[i].desired_yaw = 0.0;
        ugv_cmd_pub[i].publish(agent_cmd[i]);
    }

    sleep(1.0);

    cout << GREEN << "Input the trajectory_total_time..." << TAIL << endl;
    cin >> trajectory_total_time;

    start_cmd = 2;

    // 主循环
    while (ros::ok() && time_trajectory < trajectory_total_time)
    {
        // 回调函数,timer开始运行
        ros::spinOnce();
        
        float cos_angle;
        float sin_angle;
        angle[0] = time_trajectory * omega;
        for(int i = 1; i < agent_num; i++) 
        {
            angle[i] = angle[0] - i*2*M_PI/agent_num;
        }
        // cout << GREEN << "angle..."<< angle[0] << TAIL << endl;

        for(int i = 0; i < agent_num; i++) 
        {
            cos_angle = cos(angle[i]);
            sin_angle = sin(angle[i]);

            agent_cmd[i].agent_id = i+1;
            agent_cmd[i].control_state = sunray_msgs::agent_cmd::POS_CONTROL;
            agent_cmd[i].cmd_source = "circle_trajectory";
            agent_cmd[i].desired_pos.x = circle_radius * cos_angle + circle_center[0];
            agent_cmd[i].desired_pos.y = circle_radius * sin_angle + circle_center[1];
            agent_cmd[i].desired_pos.z = circle_center[2];
            agent_cmd[i].desired_yaw = 0.0;
            ugv_cmd_pub[i].publish(agent_cmd[i]);
        }

        time_trajectory = time_trajectory + 0.01;
        cout << GREEN << "Trajectory tracking: " << time_trajectory << " / " << trajectory_total_time << " [ s ]" << TAIL << endl;
        // sleep
        ros::Duration(0.01).sleep();
    }

    return 0;


}
