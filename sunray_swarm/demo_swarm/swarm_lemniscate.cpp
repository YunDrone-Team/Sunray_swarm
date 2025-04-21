/***********************************************************************************
 *  文件名: swarm_lemniscate.cpp                                                          
 *  作者: Yun Drone                                                                 
 *  描述: 集群demo：集群8字形轨迹移动（ORCA算法参与避障计算）agent_num：1-10均可
 *     1、从参数列表获取8字轨迹参数
 *     2、无人机使用地面站一键起飞，无人车直接启动
 *     3、等待启动指令后开始8字形运动
 *     4、需通过地面站手动降落
 ***********************************************************************************/

#include <ros/ros.h>
#include <signal.h>
#include "printf_utils.h"
#include "ros_msg_utils.h"

#define MAX_AGENT_NUM 100
using namespace std;

int agent_type;                        // 智能体类型
int agent_num;                         // 智能体数量
int agent_height;                      // 智能体固定的高度

Eigen::Vector3f circle_center;         // 圆参数：圆心坐标
float circle_radius;                   // 圆参数：圆周轨迹的半径
float linear_vel;                      // 圆参数：线速度
float omega;                           // 圆参数：角速度
float direction;                       // 圆参数：方向，1或-1
float time_trajectory = 0.0;           // 圆参数：轨迹时间计数器
float desired_yaw;                     // 期望偏航角

sunray_msgs::orca_cmd agent_orca_cmd;  // ORCA指令
std_msgs::String text_info;            // 打印消息
string node_name;                      // 节点名称

sunray_msgs::orca_state orca_state[MAX_AGENT_NUM];      // ORCA状态

ros::Subscriber orca_state_sub[MAX_AGENT_NUM];   // ORCA算法状态订阅
ros::Publisher orca_cmd_pub;                     // 发布ORCA指令
ros::Publisher orca_goal_pub[MAX_AGENT_NUM];     // ORCA算法目标点发布
ros::Publisher text_info_pub;                   // 发布信息到地面站

void mySigintHandler(int sig) 
{
    ROS_INFO("[swarm_circle] exit...");
    ros::shutdown();
}

// 处理ORCA状态回调
void rmtt_orca_state_cb(const sunray_msgs::orca_stateConstPtr& msg, int i) 
{
    // 更新指定智能体的ORCA状态
    orca_state[i] = *msg;
}

// 主函数
int main(int argc, char **argv) 
{
    // 初始化ROS节点
    ros::init(argc, argv, "swarm_lemniscate");
    // 创建节点句柄
    ros::NodeHandle nh("~");
    // 设置循环频率为10Hz
    ros::Rate rate(10.0);

    // 【参数】智能体类型 0代表RMTT，1代表UGV
    nh.param<int>("agent_type", agent_type, 0);
    // 【参数】智能体编号 智能体数量，默认为6
    nh.param<int>("agent_num", agent_num, 6);
    // 【参数】圆形轨迹参数：圆心X坐标
    nh.param<float>("circle_center_x", circle_center[0], 0.0f);
    // 【参数】圆形轨迹参数：圆心Y坐标
    nh.param<float>("circle_center_y", circle_center[1], 0.0f);
    // 【参数】圆形轨迹参数：半径
    nh.param<float>("circle_radius", circle_radius, 1.0f);
    // 【参数】圆形轨迹参数：线速度
    nh.param<float>("linear_vel", linear_vel, 0.1f);
    // 【参数】圆形轨迹参数：圆的方向 1或-1
    nh.param<float>("direction", direction, 1.0f);
    // 【参数】期望偏航角
    nh.param<float>("desired_yaw", desired_yaw, 0.0f);

    // 计算角速度
    if (circle_radius != 0)
    {
        omega = direction * fabs(float(linear_vel / circle_radius));
    }
    else
    {
        omega = 0.0;
    }
    // 根据智能体类型设置名称前缀
    string agent_prefix;
    if (agent_type == sunray_msgs::agent_state::RMTT)
    {
        agent_prefix = "rmtt";
        agent_height = 1.0;
        cout << GREEN << "agent_type    : rmtt" << TAIL << endl;
    }
    else if (agent_type == sunray_msgs::agent_state::UGV)
    {
        agent_prefix = "ugv";
        agent_height = 0.1;
        cout << GREEN << "agent_type    : ugv" << TAIL << endl;
    }

    // 修改参数描述
    cout << GREEN << "8字轨迹参数:" << TAIL << endl;
    cout << GREEN << "横向半径 : " << circle_radius << TAIL << endl;
    cout << GREEN << "纵向幅度 : " << circle_radius*0.5 << TAIL << endl;

    //【发布】ORCA算法指令 本节点 -> ORCA算法节点
    orca_cmd_pub = nh.advertise<sunray_msgs::orca_cmd>("/sunray_swarm/" + agent_prefix + "/orca_cmd", 1);
    //【发布】文字提示消息  本节点 -> 地面站
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);

    for (int i = 0; i < agent_num; i++) 
    {
        string agent_name = agent_prefix + "_" + std::to_string(i + 1);
        //【发布】每个智能体的ORCA算法目标点 本节点 -> ORCA算法节点
        orca_goal_pub[i] = nh.advertise<geometry_msgs::Point>("/sunray_swarm/" + agent_name + "/goal_point", 1);
        //【订阅】每个智能体的ORCA算法状态 本节点 -> ORCA算法节点
        orca_state_sub[i] = nh.subscribe<sunray_msgs::orca_state>("/sunray_swarm/" + agent_name + "/agent_orca_state", 1, boost::bind(&rmtt_orca_state_cb, _1, i));
    }

    sleep(1.0);
    node_name = "[" + ros::this_node::getName() + "] ---> ";

    text_info.data = node_name + "Demo init...";
    cout << GREEN << text_info.data << TAIL << endl;
    text_info_pub.publish(text_info);

    sleep(5.0);
    // 设置ORCA算法HOME点，并启动ORCA算法
    agent_orca_cmd.header.stamp = ros::Time::now();
    agent_orca_cmd.header.frame_id = "world";
    agent_orca_cmd.cmd_source = ros::this_node::getName();
    agent_orca_cmd.orca_cmd = sunray_msgs::orca_cmd::SET_HOME;
    orca_cmd_pub.publish(agent_orca_cmd);

    text_info.data = node_name + "Start orca...";
    cout << GREEN << text_info.data << TAIL << endl;
    text_info_pub.publish(text_info);
    sleep(3.0);

    time_trajectory = 0.0;
    // 将智能体移动到初始位置,初始位置分布在8字轨迹上
    for (int i = 0; i < agent_num; i++) 
    {
        // 修改相位计算公式：添加动态时间偏移
        float base_phase = i * M_PI / agent_num; // 基础相位差
        float dynamic_phase = omega * time_trajectory;
        float theta = dynamic_phase + base_phase;
        geometry_msgs::Point goal_point;
        goal_point.x = circle_center[0] + circle_radius * sin(theta);
        goal_point.y = circle_center[1] + circle_radius * sin(2*theta) * 0.5;      
        goal_point.z = desired_yaw;
        orca_goal_pub[i].publish(goal_point);
    }
    
    text_info.data = node_name + "Wait to the initial point...";
    cout << GREEN << text_info.data << TAIL << endl;
    text_info_pub.publish(text_info);
    orca_state[0].arrived_all_goal = false;
    int flag_a = 0;
    while (!(orca_state[0].arrived_all_goal)) 
    {
        flag_a++;
        sleep(1);
        ros::spinOnce();
        rate.sleep();
        if(flag_a>50)
        {
         break;   
        }
    }
    text_info.data = node_name + "Demo start...";
    cout << GREEN << text_info.data << TAIL << endl;
    text_info_pub.publish(text_info);


    // 8字轨迹生成
    while(ros::ok())
    {
        for (int i = 0; i < agent_num; i++) 
        {
            // 修改相位计算公式：添加动态时间偏移
            float base_phase = i * M_PI / agent_num; // 基础相位差
            float dynamic_phase = omega * time_trajectory;
            float theta = dynamic_phase + base_phase;
            
            geometry_msgs::Point goal_point;
            goal_point.x = circle_center[0] + circle_radius * sin(theta);
            goal_point.y = circle_center[1] + circle_radius * sin(2*theta) * 0.5;
            // 偏航角跟随圆形轨迹计算
            double vx,vy;
            vx = omega * circle_radius * cos(theta);
            vy = omega * circle_radius * cos(2*theta);
            goal_point.z = atan2(vy, vx);  
            orca_goal_pub[i].publish(goal_point);
        }


        // 更新时间计数器，由于循环频率为10Hz，因此设置为0.1秒
        time_trajectory += 0.1; // 保持10Hz更新率
        ros::spinOnce();
        rate.sleep();
    }

    text_info.data = node_name + "Demo finished...";
    cout << GREEN << text_info.data << TAIL << endl;
    text_info_pub.publish(text_info);
    return 0;
}