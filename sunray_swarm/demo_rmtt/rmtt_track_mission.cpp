#include <ros/ros.h>
#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"

#define MAX_AGENT_NUM 20   
    
using namespace std;
int agent_type;
int agent_id;;
float agent_height;
string target_name;
int start_cmd = 0;
geometry_msgs::PoseStamped target_pos;
float target_yaw;
sunray_msgs::agent_cmd agent_cmd;

ros::Subscriber target_pos_sub;
ros::Publisher agent_cmd_pub;
ros::Publisher text_info_pub;
bool received_start_cmd = false;     // 接收到开始命令
ros::Subscriber single_trackMission_sub;



void single_trackMission_cb(const std_msgs::Bool::ConstPtr &msg)
{
    received_start_cmd = true;     // 接收到开始命令
    start_cmd = 1;
}

void target_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg, int i)
{
    target_pos = *msg;

    Eigen::Quaterniond q_mocap = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Vector3d target_att = quaternion_to_euler(q_mocap);
    target_yaw = target_att.z();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "track_mission");
    ros::NodeHandle nh("~");
    ros::Rate rate(100.0);
 

    // 【参数】agent_height
    nh.param<float>("agent_height", agent_height, 1.0f);
    // 【参数】目标名称
    nh.param<string>("target_name", target_name, "none");


    cout << GREEN << "agent_height   : " << agent_height << "" << TAIL << endl;
    cout << GREEN << "target_name   : " << target_name << "" << TAIL << endl;



    string agent_name;
    agent_name = "/rmtt_" + std::to_string(agent_id);
    target_name = "/ugv_" + std::to_string(agent_id);
    
    target_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node"+ target_name + "/pose", 1, boost::bind(&target_pos_cb,_1,1));
    // 【订阅】触发指令 外部 -> 本节点 ——TODO设置为BOOL值变量
    single_trackMission_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/demo/single_trackMission", 1, single_trackMission_cb);
    // 【发布】控制指令 本节点 -> 控制节点
    agent_cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 1);
    // 【发布】文字提示消息  本节点 -> 地面站
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);
    
     sleep(1.0);


    // 主循环
    while (ros::ok())
    {
        if(start_cmd)
        {
            agent_cmd.agent_id =1;
            agent_cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
            agent_cmd.cmd_source = "track_mission";
            agent_cmd.desired_pos.x = target_pos.pose.position.x;
            agent_cmd.desired_pos.y = target_pos.pose.position.y;
            agent_cmd.desired_pos.z = agent_height;
            agent_cmd.desired_yaw = target_yaw;
            agent_cmd_pub.publish(agent_cmd);
        }

        // 回调函数,timer开始运行
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }

    return 0;


}
