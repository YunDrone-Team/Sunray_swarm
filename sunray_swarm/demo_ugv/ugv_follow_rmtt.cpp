/***********************************************************************************
 *  文件名: ugv_follow_rmtt.cpp                                                          
 *  作者: Yun Drone                                                                 
 *  描述: ugv_demo：无人车跟随无人机
 *     1、无人机起飞，等待demo启动指令
 *     2、启动后根据动捕中无人机的位置进行追随控制（POS_CONTROL模式）
 *     3、可通过demo_start_flag暂停或恢复跟随
 *     4、本程序没有设置自动降落，可以停止跟随后，通过地面站降落
 ***********************************************************************************/

 #include <ros/ros.h>
 #include "printf_utils.h"
 #include "ros_msg_utils.h"
 #include "math_utils.h"
 
 using namespace std;
 
 int agent_id;                          // 智能体编号
 bool demo_start_flag = false;          // 接收到开始命令
 std_msgs::String text_info;            // 打印消息
 string target_name;                    // 目标名称
 sunray_msgs::agent_cmd agent_cmd;      // 控制命令消息
 geometry_msgs::PoseStamped target_pos; // 位置
 double target_yaw{0.0};                // 无人机yaw
 
 ros::Subscriber target_pos_sub;        // 订阅目标位置
 ros::Publisher agent_cmd_pub;          // 发布控制命令
 ros::Subscriber demo_start_flag_sub;   // 订阅开始命令
 ros::Publisher text_info_pub;          // 发布信息到地面站
 
 void demo_start_flag_cb(const std_msgs::Bool::ConstPtr &msg)
 {
     demo_start_flag = msg->data;    
 
     if(demo_start_flag)
     {
         text_info.data = "Get demo start cmd";
         cout << GREEN << text_info.data << TAIL << endl;
         text_info_pub.publish(text_info);
     }else
     {
         text_info.data = "Get demo stop cmd";
         cout << GREEN << text_info.data << TAIL << endl;
         text_info_pub.publish(text_info);
     }
 }
 
 // 目标位置回调函数
 void mocap_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
 {
     target_pos = *msg;
     // 将目标位置转换为欧拉角,获取四元数
     Eigen::Quaterniond q_mocap = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
     // 转换为欧拉角
     Eigen::Vector3d target_att = quaternion_to_euler(q_mocap);
     // 获取目标的偏航角
     target_yaw = target_att.z();
 }
 
 // 主函数
 int main(int argc, char **argv)
 {
     // 初始化ROS节点
     ros::init(argc, argv, "ugv_follow_rmtt");
     // 创建节点句柄
     ros::NodeHandle nh("~");
     // 设置节点的执行频率为10Hz
     ros::Rate rate(10);
 
     // 【参数】智能体编号
     nh.param<int>("agent_id", agent_id, 1);
     // 【参数】目标名称（动捕中设置）
     nh.param<string>("target_name", target_name, "rmtt_1");
 
     cout << GREEN << ros::this_node::getName() << " start." << TAIL << endl;
     cout << GREEN << "target_name      : " << target_name << TAIL << endl;
 
     string agent_name = "/ugv_" + std::to_string(agent_id);
     // 【订阅】无人机位置 VRPN（动捕） -> 本节点目标位置
     target_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/"+ target_name + "/pose", 1, mocap_pos_cb);
     // 【订阅】触发指令 外部 -> 本节点 
     demo_start_flag_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/demo/ugv_follow_rmtt", 1, demo_start_flag_cb);
     // 【发布】控制指令 本节点 -> 无人车控制节点
     agent_cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 10);
     // 【发布】文字提示消息  本节点 -> 地面站
     text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);
 
     sleep(5.0);
 
     // 主循环
     while (ros::ok())
     {
         // 等待demo启动
         if(!demo_start_flag)
         {
             // 处理一次回调函数
             ros::spinOnce();
             // sleep
             rate.sleep();
             continue;
         }
 
         agent_cmd.header.stamp = ros::Time::now();
         agent_cmd.header.frame_id = "world";
         agent_cmd.agent_id = agent_id;
         agent_cmd.cmd_source = ros::this_node::getName();
         agent_cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
         agent_cmd.desired_pos.x = target_pos.pose.position.x;
         agent_cmd.desired_pos.y = target_pos.pose.position.y;
         agent_cmd.desired_pos.z = 0.1;
         agent_cmd.desired_yaw = target_yaw;  
         agent_cmd_pub.publish(agent_cmd);
 
         ros::spinOnce();
         rate.sleep();
     }
 
     return 0;
 }