#include <ros/ros.h>
#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"
#define MAX_AGENT_NUM 20

using namespace std;
int agent_type;                        // 智能体类型
int agent_id;                          // 智能体编号
float agent_height;                    // 智能体飞行高度
string target_name;                    // 目标名称
int start_cmd = 0;                     // 启动命令
geometry_msgs::PoseStamped target_pos; // 目标位置
float target_yaw;                      // 目标偏航角
sunray_msgs::agent_cmd agent_cmd;      // 控制命令消息

ros::Subscriber target_pos_sub;          // 订阅目标位置
ros::Publisher agent_cmd_pub;            // 发布控制命令
ros::Publisher text_info_pub;            // 发布文字提示消息
ros::Subscriber single_trackMission_sub; // 订阅轨迹任务触发信号

bool received_start_cmd = false; // 接收到开始命令
bool mission_completed = false;   // 任务是否完成

// 轨迹任务触发信号回调函数
void single_trackMission_cb(const std_msgs::Bool::ConstPtr &msg)
{
    received_start_cmd = msg->data; // 更新开始命令的状态
}

// 目标位置回调函数
void target_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg, int i)
{
    target_pos = *msg;
    // 将目标位置转换为欧拉角,获取四元数
    Eigen::Quaterniond q_mocap = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    // 转换为欧拉角
    Eigen::Vector3d target_att = quaternion_to_euler(q_mocap);
    // 获取目标的偏航角
    target_yaw = target_att.z();
}

int main(int argc, char **argv)
{
    // 初始化ROS节点,创建节点句柄
    ros::init(argc, argv, "track_mission");
    ros::NodeHandle nh("~");
    // 设置循环频率为100Hz
    ros::Rate rate(100.0);

    // 【参数】从参数服务器获取智能体高度
    nh.param<float>("agent_height", agent_height, 1.0f);
    // 【参数】目标名称
    nh.param<string>("target_name", target_name, "none");
    // 【参数】从参数服务器获取智能体编号
    nh.param("agent_id", agent_id, 1);
    // 打印智能体高度、目标名称
    cout << GREEN << "agent_height   : " << agent_height << "" << TAIL << endl;
    cout << GREEN << "target_name   : " << target_name << "" << TAIL << endl;
    // 声明字符串以存储代理名称
    string agent_name;
    // 构造代理名称
    agent_name = "/rmtt_" + std::to_string(agent_id);
    // 构造目标名称
    target_name = "/ugv_" + std::to_string(agent_id);
    // 【订阅】外部 -> 本节点目标位置
    target_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node" + target_name + "/pose", 1, boost::bind(&target_pos_cb, _1, 1));
    // 【订阅】触发指令 外部 -> 本节点 ——TODO设置为BOOL值变量
    single_trackMission_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/demo/rmtt_trackMission", 1, single_trackMission_cb);
    // 【发布】控制指令 本节点 -> 控制节点
    agent_cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 1);
    // 【发布】文字提示消息  本节点 -> 地面站
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);

    sleep(1.0);
    sunray_msgs::agent_cmd cmd;
    // 主循环
    while (ros::ok())
    {
        // 检查是否接收到开始命令
        if (received_start_cmd && !mission_completed)
        {
            // 设置为起飞状态
            cmd.control_state = 11;
            // 设置智能体编号
            agent_cmd.agent_id = 1;
            // 设置目标高度
            agent_cmd.desired_pos.z = agent_height;
            // 发布控制命令
            agent_cmd_pub.publish(agent_cmd);
            // 等待3秒确保起飞
            ros::Duration(3.0).sleep(); 
            // 设置控制模式为位置控制
            agent_cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
            // 指定命令来源为轨迹任务来源
            agent_cmd.cmd_source = "track_mission";
            // 设置目标位置
            // agent_cmd.desired_pos.x = target_pos.pose.position.x;
            // agent_cmd.desired_pos.y = target_pos.pose.position.y;
            agent_cmd.desired_pos = target_pos.pose.position;

            // 等待目标到达
            ros::Duration(2.0).sleep(); // 根据实际需求调整
            
            // 设置目标偏航角
            agent_cmd.desired_yaw = target_yaw;
            
            agent_cmd_pub.publish(agent_cmd);
            // 创建文本信息对象
            std_msgs::String text_info;
            // 发布提示消息
            text_info.data = "Agent driving to target: x=" + to_string(target_pos.pose.position.x) + " y=" + to_string(target_pos.pose.position.y) + " z=" + to_string(agent_height);

            // 等待15秒后发布降落指令
            ros::Duration(15.0).sleep();
            // 降落状态
            agent_cmd.control_state = 12;
            // 发布控制命令 
            agent_cmd_pub.publish(agent_cmd);
            cout << GREEN << "Landing command sent." << TAIL << endl;
            // 重置命令状态
            received_start_cmd = false;
            mission_completed = true;    // 标记任务已完成
        }
        
        // 回调函数,timer开始运行
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }

    return 0;
}
