/***********************************************************************************
 *  文件名: ugv_follow_rmtt.cpp                                                          
 *  作者: Yun Drone                                                                 
 *  描述: ugv_demo：无人车跟随无人机
 *     1、无人机起飞
 *     2、启动后根据动捕中无人机的位置进行追随控制（POS_CONTROL模式）
 *     3、本程序没有设置自动降落，可以停止跟随后，通过地面站降落
 ***********************************************************************************/

 #include <ros/ros.h>
 #include "printf_utils.h"
 #include "ros_msg_utils.h"
 #include "math_utils.h"
 
 using namespace std;
 
 int agent_id;                          // 智能体编号
 bool land_flag = false;   
 std_msgs::String text_info;            // 打印消息
 string target_name;                    // 目标名称
 sunray_msgs::agent_cmd agent_cmd;      // 控制命令消息
 geometry_msgs::PoseStamped target_pos; // 位置
 double target_yaw{0.0};                // 无人机yaw
 string node_name;                      // 节点名称
 sunray_msgs::agent_cmd current_agent_cmd;
 ros::Subscriber target_pos_sub;        // 订阅目标位置
 ros::Subscriber agent_gs_cmd_sub;      // 订阅智能体控制指令 
 ros::Publisher agent_cmd_pub_ugv;          // 发布ugv控制命令
 ros::Publisher agent_cmd_pub_rmtt;          // 发布ugv控制命令
 ros::Publisher text_info_pub;          // 发布信息到地面站
 
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
 
 void agent_gs_cmd_cb(const sunray_msgs::agent_cmd::ConstPtr& msg)
 {
    if(msg->agent_id != agent_id && msg->agent_id != 99)
      {
         return;
      } 
      current_agent_cmd = *msg;
    if(current_agent_cmd.control_state == sunray_msgs::agent_cmd::LAND)
      {
         land_flag = true;
      }
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

    cout << GREEN << "target_name      : " << target_name << TAIL << endl;

    string agent_name_ugv = "/ugv_" + std::to_string(agent_id);
    string agent_name_rmtt = "/rmtt_" + std::to_string(agent_id);
    // 【订阅】无人机位置 VRPN（动捕） -> 本节点目标位置
    target_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/"+ target_name + "/pose", 1, mocap_pos_cb);
    // 【订阅】智能体控制指令 地面站 -> 本节点
    agent_gs_cmd_sub = nh.subscribe<sunray_msgs::agent_cmd>("/sunray_swarm/rmtt_gs/agent_cmd", 10, agent_gs_cmd_cb);  
    // 【发布】控制指令 本节点 -> 无人车控制节点
    agent_cmd_pub_ugv = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name_ugv + "/agent_cmd", 10);
    // 【发布】控制指令 本节点 -> 无人机控制节点
    agent_cmd_pub_rmtt = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name_rmtt + "/agent_cmd", 10);
    // 【发布】文字提示消息  本节点 -> 地面站
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);

    sleep(1.0);
    node_name = "[" + ros::this_node::getName() + "] ---> ";
    sleep(5.0);
      // 发送起飞指令
    agent_cmd.header.stamp = ros::Time::now();
    agent_cmd.header.frame_id = "world";
    agent_cmd.agent_id = agent_id;
    agent_cmd.cmd_source = ros::this_node::getName();
    agent_cmd.control_state = sunray_msgs::agent_cmd::TAKEOFF;
    agent_cmd_pub_rmtt.publish(agent_cmd); 

    text_info.data = node_name + "Takeoff...";
    cout << GREEN << text_info.data << TAIL << endl;
    text_info_pub.publish(text_info);
    sleep(5.0);
    text_info.data = node_name + "Demo init...";
    cout << GREEN << text_info.data << TAIL << endl;
    text_info_pub.publish(text_info);

    // 主循环
    while (ros::ok())
    {
         float rmtt_height = target_pos.pose.position.z;
         if((rmtt_height > 0.75) && (!land_flag))
         {
            agent_cmd.header.stamp = ros::Time::now();
            agent_cmd.header.frame_id = "world";
            agent_cmd.agent_id = agent_id;
            agent_cmd.cmd_source = ros::this_node::getName();
            agent_cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
            agent_cmd.desired_pos.x = target_pos.pose.position.x;
            agent_cmd.desired_pos.y = target_pos.pose.position.y;
            agent_cmd.desired_pos.z = 0.1;
            agent_cmd.desired_yaw = target_yaw;  
            agent_cmd_pub_ugv.publish(agent_cmd);
         }
         if(land_flag||target_pos.pose.position.z<0.75)
         {
          if(target_pos.pose.position.x > 0)
           {
             agent_cmd.desired_pos.x = target_pos.pose.position.x - 1;
           }
          else
          {
             agent_cmd.desired_pos.x = target_pos.pose.position.x + 1;
          }
          if(target_pos.pose.position.y > 0)
           {
             agent_cmd.desired_pos.y = target_pos.pose.position.y - 1;
           }
          else
          {
             agent_cmd.desired_pos.y = target_pos.pose.position.y + 1;
          }
          agent_cmd.desired_yaw = target_yaw;  
          agent_cmd_pub_ugv.publish(agent_cmd);
         }
         ros::spinOnce();
         rate.sleep();
    }
 
    text_info.data = node_name + "Demo finished...";
    cout << GREEN << text_info.data << TAIL << endl;
    text_info_pub.publish(text_info);
    return 0;
 }