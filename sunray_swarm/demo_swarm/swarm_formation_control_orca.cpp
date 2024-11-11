#include <ros/ros.h>
#include "printf_utils.h"
#include "math_utils.h"

#define MAX_AGENT_NUM 10   
using namespace std;

int agent_num;                                          //智能体数量
float agent_height;                                     //智能体飞行高度
ros::Publisher agent_cmd_pub[MAX_AGENT_NUM];            //存储每个智能体的控制指令发布者
ros::Publisher text_info_pub;                           //文字提示消息发布者
std::vector<geometry_msgs::Point> triangle_formation;   //三角形队形的坐标
std::vector<geometry_msgs::Point> line_formation;       //一字型队形的坐标
ros::Subscriber swarm_formation_cmd_sub;                //触发条件的订阅者
bool received_start_cmd = false;                        //标记是否接收到开始命令

int agent_type;                                         // 代理类型，用于区分无人机和无人车

ros::Subscriber orca_state_sub[MAX_AGENT_NUM]; // ORCA状态订阅器
ros::Publisher orca_goal_pub[MAX_AGENT_NUM];   // 智能体目标点发布器
sunray_msgs::orca_state orca_state[MAX_AGENT_NUM];      // ORCA状态
sunray_msgs::orca_cmd orca_cmd;                         // ORCA指令
ros::Publisher orca_cmd_pub;                        // 发布ORCA指令
void rmtt_orca_state_cb(const sunray_msgs::orca_stateConstPtr &msg, int i);
// 定义队形状态的枚举类型
enum FORMATION_STATE
{
    TRIANGLE = 0, // 三角形队形
    LINE = 1,     // 一字型队形
};
FORMATION_STATE formation_state;// 当前队形状态

// 处理队形切换命令的回调函数
void swarm_formation_cb(const std_msgs::Bool::ConstPtr& msg) 
{
    // 设置接收到的开始命令
    received_start_cmd = msg->data; // 设置接收到的开始命令
    // 设置ORCA命令为HOME
    orca_cmd.orca_cmd = sunray_msgs::orca_cmd::SET_HOME;
    // 发布命令
    orca_cmd_pub.publish(orca_cmd);
    // 设置延迟
    sleep(0.5);
}

// ORCA状态回调函数
void rmtt_orca_state_cb(const sunray_msgs::orca_stateConstPtr &msg, int i)
{
    orca_state[i] = *msg; // 更新指定智能体的ORCA状态
}
// 切换队形的函数
void switch_formation(FORMATION_STATE state)
{
    // 存储控制命令的数组
    sunray_msgs::agent_cmd agent_cmd[MAX_AGENT_NUM];
    // 存储控制命令的数组
    sunray_msgs::orca_cmd orca_cmd[MAX_AGENT_NUM];                    // ORCA指令
    // 用于存储当前队形的坐标
    std::vector<geometry_msgs::Point> formation;
    // 根据当前队形状态选择队形
    if (state == FORMATION_STATE::TRIANGLE)
    {
        // 选择三角形队形
        formation = triangle_formation;
    }
    else if (state == FORMATION_STATE::LINE)
    {
        // 选择一字型队形
        formation = line_formation;
    }
    // 发布每个智能体的控制命令
    for(int i = 0; i < agent_num; i++) 
    {
        // agent_cmd[i].agent_id = i+1; // 设置智能体ID
        // agent_cmd[i].control_state = sunray_msgs::agent_cmd::POS_CONTROL; // 设置控制状态为位置控制
        // agent_cmd[i].desired_pos.x = formation[i].x; // 设置目标X坐标
        // agent_cmd[i].desired_pos.y = formation[i].y; // 设置目标Y坐标
        // agent_cmd[i].desired_pos.z = agent_height; // 设置目标Z坐标
        // agent_cmd_pub[i].publish(agent_cmd[i]); // 发布控制命令
        geometry_msgs::Point target_point;
        target_point.x = formation[i].x;            // 设置目标X坐标
        target_point.y = formation[i].y;            // 设置目标Y坐标
        target_point.z = agent_height;              // 设置目标Z坐标
        orca_goal_pub[i].publish(target_point);     // 发布控制命令


    }
}



int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "formation_control");
    // 创建节点句柄
    ros::NodeHandle nh("~");
    // 设置节点的执行频率为10Hz
    ros::Rate rate(10);
    // 【参数】智能体数量 默认3台无人机/车
    nh.param<int>("agent_num", agent_num, 3); 
    // 【参数】智能体高度 默认飞行高度1米
    nh.param<float>("agent_height", agent_height, 1.0f); 
    // 【参数】智能体类型 默认rmtt
    nh.param<int>("agent_type", agent_type, 0);  

    // 设置三角形队形
    triangle_formation.resize(agent_num);                           // 调整三角形队形的大小
    triangle_formation[0].x = 0.1; triangle_formation[0].y = -1.0;  // 第一个智能体的位置
    triangle_formation[2].x = 1.0; triangle_formation[2].y = 1.0;   // 第三个智能体的位置
    triangle_formation[1].x = -1.0; triangle_formation[1].y = 1.0;  // 第二个智能体的位置
    // 更多无人机可以根据需要设置

    // 设置一字型队形
    line_formation.resize(agent_num);                               // 调整一字型队形的大小

    line_formation[0].x = 0; line_formation[0].y = 0;        // 设置第一个智能体的位置
    line_formation[1].x = -1; line_formation[1].y = 0;
    line_formation[2].x = 1; line_formation[2].y = 0;


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
        // 【发布】ORCA指令 本节点 ->  ORCA
    orca_cmd_pub = nh.advertise<sunray_msgs::orca_cmd>("/sunray_swarm/" + agent_prefix + "/orca_cmd", 1);

    // 初始化每个智能体的控制命令发布者
    string agent_name;
    for(int i = 0; i < agent_num; i++) 
    {
        // 生成智能体名称
        agent_name = "/" + agent_prefix + std::to_string(i+1);
        // 【发布】无人车控制指令
        // agent_cmd_pub[i] = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 1);
        // 【发布】无人机的目标点
        orca_goal_pub[i] = nh.advertise<geometry_msgs::Point>("/sunray_swarm" + agent_name + "/goal_point", 1);
        // 【订阅】无人机orca状态
        orca_state_sub[i] = nh.subscribe<sunray_msgs::orca_state>("/sunray_swarm" + agent_name + "/agent_orca_state", 1, boost::bind(&rmtt_orca_state_cb, _1, i));
    }
    // [订阅]触发条件
    swarm_formation_cmd_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/demo/swarm_formation_control", 1, swarm_formation_cb);
    // 初始化队形状态为三角形
    formation_state = FORMATION_STATE::TRIANGLE;
    // 主循环
    while (ros::ok())
    {
        // 检查是否接收到开始命令
        if(received_start_cmd)
        {
            // 在三角形和一字型队形之间切换
            switch_formation(formation_state);
            // 模拟在两种队形之间切换
            formation_state = (formation_state == FORMATION_STATE::TRIANGLE) ? FORMATION_STATE::LINE : FORMATION_STATE::TRIANGLE;
            // 每8秒切换一次
            ros::Duration(8.0).sleep();  
        }
        ros::spinOnce();
        // 休眠0.1秒
        ros::Duration(0.1).sleep();

    }
    return 0;
}