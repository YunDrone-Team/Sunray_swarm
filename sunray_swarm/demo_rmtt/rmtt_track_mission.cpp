#include <ros/ros.h>
#include <signal.h>

#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"

#define MAX_AGENT_NUM 20   
    
using namespace std;
int agent_type;
int agent_num;
float agent_height;
string target_name;
int start_cmd = 0;
geometry_msgs::PoseStamped target_pos[MAX_AGENT_NUM];
float target_yaw[MAX_AGENT_NUM];
sunray_msgs::agent_cmd agent_cmd[MAX_AGENT_NUM];

ros::Subscriber target_pos_sub[MAX_AGENT_NUM];
ros::Publisher agent_cmd_pub[MAX_AGENT_NUM];
ros::Publisher text_info_pub;

void mySigintHandler(int sig)
{
    ROS_INFO("[circle_trajectory] exit...");
    ros::shutdown();
}
void target_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg, int i)
{
    target_pos[i] = *msg;

    Eigen::Quaterniond q_mocap = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Vector3d target_att = quaternion_to_euler(q_mocap);
    target_yaw[i] = target_att.z();
}

void printf_params()
{
    cout << GREEN << "agent_type    : " << agent_type << "" << TAIL << endl;
    cout << GREEN << "agent_num     : " << agent_num << "" << TAIL << endl;
    cout << GREEN << "agent_height   : " << agent_height << "" << TAIL << endl;
    cout << GREEN << "target_name   : " << target_name << "" << TAIL << endl;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "track_mission");
    ros::NodeHandle nh("~");
    ros::Rate rate(100.0);

    // 【参数】智能体类型
    nh.param<int>("agent_type", agent_type, 1);
    // 【参数】智能体编号
    nh.param<int>("agent_num", agent_num, 8);
    // 【参数】agent_height
    nh.param<float>("agent_height", agent_height, 0.0f);
    // 【参数】目标名称
    nh.param<string>("target_name", target_name, "none");

    printf_params();

    string agent_prefix;

    if(agent_type == sunray_msgs::agent_state::RMTT)
    {
        agent_prefix = "rmtt_";
    }else if(agent_type == sunray_msgs::agent_state::TIANBOT)
    {
        agent_prefix = "tianbot_";
    }else if(agent_type == sunray_msgs::agent_state::WHEELTEC)
    {
        agent_prefix = "wheeltec_";
    }else if(agent_type == sunray_msgs::agent_state::SIKONG)
    {
        agent_prefix = "sikong_";
    }else
    {
        agent_prefix = "unkonown_";
    }

    // 【发布】文字提示消息（回传至地面站显示）
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);

    string agent_name;
    for(int i = 0; i < agent_num; i++) 
    {
        agent_name = "/" + agent_prefix + std::to_string(i+1);
        target_name = "/" + target_name + "_" + std::to_string(i+1);
        target_pos_sub[i] = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node"+ target_name + "/pose", 1, boost::bind(&target_pos_cb,_1,i));
        // 【发布】无人车控制指令
        agent_cmd_pub[i] = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 1);
    }
            // [订阅]触发条件
    // agent_cmd_pub = nh.advertise<std_msgs::Bool>("/sunray_swarm/track_mission", 1， start_cmd_cb);

    cout << GREEN << "Please enter 1 to move to start pos..." << TAIL << endl;
    cin >> start_cmd;

    // for(int i = 0; i < agent_num; i++) 
    // {
    //     agent_cmd[i].agent_id = i+1;
    //     agent_cmd[i].control_state = sunray_msgs::agent_cmd::POS_CONTROL;
    //     agent_cmd[i].desired_pos.x = circle_radius * cos_angle + circle_center[0];
    //     agent_cmd[i].desired_pos.y = circle_radius * sin_angle + circle_center[1];
    //     agent_cmd[i].desired_pos.z = circle_center[2];
    //     agent_cmd[i].desired_yaw = 0.0;
    //     agent_cmd_pub[i].publish(agent_cmd[i]);
    // }

    sleep(1.0);

    cout << GREEN << "Please enter 1 to move to start track target..." << TAIL << endl;
    cin >> start_cmd;

    start_cmd = 2;

    // 主循环
    while (ros::ok())
    {
        // 回调函数,timer开始运行
        ros::spinOnce();
        
        for(int i = 0; i < agent_num; i++) 
        {
            agent_cmd[i].agent_id = i+1;
            agent_cmd[i].control_state = sunray_msgs::agent_cmd::POS_CONTROL;
            agent_cmd[i].cmd_source = "track_mission";
            agent_cmd[i].desired_pos.x = target_pos[i].pose.position.x;
            agent_cmd[i].desired_pos.y = target_pos[i].pose.position.y;
            agent_cmd[i].desired_pos.z = agent_height;
            agent_cmd[i].desired_yaw = target_yaw[i];
            agent_cmd_pub[i].publish(agent_cmd[i]);
        }

        // cout << GREEN << "Trajectory tracking: " << time_trajectory << " / " << trajectory_total_time << " [ s ]" << TAIL << endl;
        // sleep
        ros::Duration(0.05).sleep();
    }

    return 0;


}
