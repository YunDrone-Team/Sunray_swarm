#include <ros/ros.h>
#include <signal.h>
#include <thread>
#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"

#define MAX_AGENT_NUM 10   
    
using namespace std;

int uav_id;
string node_name;
bool pub_goal1_flag{false};
bool pub_goal2_flag{false};
bool pub_goal3_flag{false};
bool pub_goal4_flag{false};
bool pub_return{false};

sunray_msgs::rmtt_state rmtt_state;
sunray_msgs::rmtt_orca rmtt_orca_state;
geometry_msgs::Point goal;
sunray_msgs::station_cmd station_cmd;
ros::Publisher station_cmd_pub;
ros::Publisher text_info_pub;
ros::Publisher rmtt_goal_pub;
ros::Subscriber rmtt_state_sub;
ros::Subscriber rmtt_orca_state_sub;
ros::Subscriber station_cmd_sub;
geometry_msgs::Point goal1,goal2,goal3,goal4;

void mySigintHandler(int sig)
{
    ROS_INFO("[rmtt_show] exit...");
    ros::shutdown();
}
void station_cmd_cb(const sunray_msgs::station_cmdConstPtr& msg)
{
    if(msg->mission_state == 1 && msg->uav_id ==99)
    {
        pub_goal1_flag = true;
        cout << BLUE << node_name << " start..." << TAIL << endl;
        sleep(6.0);
    }
    // 收到return指令，则重置所有标志位，退出当前任务
    if(msg->mission_state == 6 && msg->uav_id ==99)
    {
        pub_goal1_flag = false;
        pub_goal2_flag = false;
        pub_goal3_flag = false;
        pub_goal4_flag = false;
        pub_return = false;
        rmtt_orca_state.arrived_goal = false;
        cout << BLUE << node_name << " get return home..." << TAIL << endl;
    }
}
void rmtt_state_cb(const sunray_msgs::rmtt_stateConstPtr& msg)
{
    rmtt_state = *msg;
}
void rmtt_orca_state_cb(const sunray_msgs::rmtt_orcaConstPtr& msg)
{
    rmtt_orca_state = *msg;
}
void timercb_show(const ros::TimerEvent &e);
void setup_show_goals();
int main(int argc, char **argv)
{
    ros::init(argc, argv, "rmtt_show");
    ros::NodeHandle nh("~");
    ros::Rate rate(20.0);
    node_name = ros::this_node::getName();

    // 【参数】无人机编号
    nh.param<int>("uav_id", uav_id, 1);

    string agent_name = "/rmtt_" + std::to_string(uav_id);
    // 【订阅】地面站指令
    station_cmd_sub = nh.subscribe<sunray_msgs::station_cmd>("/sunray_swarm/station_cmd", 1, station_cmd_cb);
    // 【订阅】无人机状态数据
    rmtt_state_sub = nh.subscribe<sunray_msgs::rmtt_state>("/sunray_swarm" + agent_name + "/rmtt_state", 1, rmtt_state_cb);
    // 【订阅】无人机orca状态 回传地面站
    rmtt_orca_state_sub = nh.subscribe<sunray_msgs::rmtt_orca>("/sunray_swarm" + agent_name + "/rmtt_orca", 1, rmtt_orca_state_cb);
    // 【发布】无人机的目标点
    rmtt_goal_pub = nh.advertise<geometry_msgs::Point>("/sunray_swarm" + agent_name + "/goal_point", 1);
    // 【发布】地面站指令
    station_cmd_pub = nh.advertise<sunray_msgs::station_cmd>("/sunray_swarm" + agent_name + "/station_cmd", 1);
    // 【发布】文字提示消息（回传至地面站显示）
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);

    ros::Timer timer_show = nh.createTimer(ros::Duration(3.0), timercb_show);

    setup_show_goals();

    pub_goal1_flag = false;
    pub_goal2_flag = false;
    pub_goal3_flag = false;
    pub_goal4_flag = false;
    pub_return = false;
    rmtt_orca_state.arrived_goal = false;

    // 主循环
    while (ros::ok())
    {
        // 回调函数,timer开始运行
        ros::spinOnce();
        // sleep
        rate.sleep();
    }

    return 0;

}

void timercb_show(const ros::TimerEvent &e)
{
    if(pub_goal1_flag)
    {
        rmtt_goal_pub.publish(goal1);
        cout << BLUE << node_name << " pub_goal1..." << TAIL << endl;
        sleep(1.0);

        // orca_run
        station_cmd.mission_state = 5;
        station_cmd.uav_id = uav_id;
        station_cmd_pub.publish(station_cmd);

        // set flag
        pub_goal1_flag = false;
        pub_goal2_flag = true;
        pub_goal3_flag = false;
        pub_goal4_flag = false;
        pub_return = false;
        rmtt_orca_state.arrived_goal = false;
    }

    // rmtt1 抵达初始目标点,先降落，然后起飞，然后发送下一个目标点
    if(pub_goal2_flag && rmtt_orca_state.arrived_goal && rmtt_orca_state.goal[0]==goal1.x && rmtt_orca_state.goal[1]==goal1.y)
    {
        // land
        station_cmd.mission_state = 2;
        station_cmd.uav_id = uav_id;
        station_cmd_pub.publish(station_cmd);
        sleep(6.0);

        // takeoff
        station_cmd.mission_state = 1;
        station_cmd.uav_id = uav_id;
        station_cmd_pub.publish(station_cmd);
        sleep(6.0);

        // pub_new
        rmtt_goal_pub.publish(goal2);
        cout << BLUE << node_name << " pub_goal2..." << TAIL << endl;

        // orca_run
        station_cmd.mission_state = 5;
        station_cmd.uav_id = uav_id;
        station_cmd_pub.publish(station_cmd);

        // set flag
        pub_goal1_flag = false;
        pub_goal2_flag = false;
        pub_goal3_flag = true;
        pub_goal4_flag = false;
        pub_return = false;
        rmtt_orca_state.arrived_goal = false;
    }

    if(pub_goal3_flag && rmtt_orca_state.arrived_goal && rmtt_orca_state.goal[0]==goal2.x && rmtt_orca_state.goal[1]==goal2.y)
    {
        // land
        station_cmd.mission_state = 2;
        station_cmd.uav_id = uav_id;
        station_cmd_pub.publish(station_cmd);
        sleep(6.0);

        // takeoff
        station_cmd.mission_state = 1;
        station_cmd.uav_id = uav_id;
        station_cmd_pub.publish(station_cmd);
        sleep(6.0);

        // pub_new
        rmtt_goal_pub.publish(goal3);
        cout << BLUE << node_name << " pub_goal3..." << TAIL << endl;

        // orca_run
        station_cmd.mission_state = 5;
        station_cmd.uav_id = uav_id;
        station_cmd_pub.publish(station_cmd);

        // set flag
        pub_goal1_flag = false;
        pub_goal2_flag = false;
        pub_goal3_flag = false;
        pub_goal4_flag = true;
        pub_return = false;
        rmtt_orca_state.arrived_goal = false;
    }

    if(pub_goal4_flag && rmtt_orca_state.arrived_goal && rmtt_orca_state.goal[0]==goal3.x && rmtt_orca_state.goal[1]==goal3.y)
    {
        // land
        station_cmd.mission_state = 2;
        station_cmd.uav_id = uav_id;
        station_cmd_pub.publish(station_cmd);
        sleep(6.0);

        // takeoff
        station_cmd.mission_state = 1;
        station_cmd.uav_id = uav_id;
        station_cmd_pub.publish(station_cmd);
        sleep(6.0);

        // pub_new
        rmtt_goal_pub.publish(goal4);
        cout << BLUE << node_name << " pub_goal4..." << TAIL << endl;

        // orca_run
        station_cmd.mission_state = 5;
        station_cmd.uav_id = uav_id;
        station_cmd_pub.publish(station_cmd);
        
        // set flag
        pub_goal1_flag = false;
        pub_goal2_flag = false;
        pub_goal3_flag = false;
        pub_goal4_flag = false;
        pub_return = true;
        rmtt_orca_state.arrived_goal = false;
    }

    if(pub_return && rmtt_orca_state.arrived_goal && rmtt_orca_state.goal[0]==goal4.x && rmtt_orca_state.goal[1]==goal4.y)
    {
        // land
        station_cmd.mission_state = 2;
        station_cmd.uav_id = uav_id;
        station_cmd_pub.publish(station_cmd);
        sleep(6.0);

        pub_goal1_flag = false;
        pub_goal2_flag = false;
        pub_goal3_flag = false;
        pub_goal4_flag = false;
        pub_return = false;
    }

}

void setup_show_goals()
{
    if(uav_id == 1)
    {
        goal1.x = 1.5;
        goal1.y = 1.5;
        goal2.x = -1.5;
        goal2.y = 1.5;
        goal3.x = 1.5;
        goal3.y = 1.5;
        goal4.x = -1.5;
        goal4.y = 1.5;
    }else if(uav_id == 2)
    {
        goal1.x = 1.5;
        goal1.y = 0.5;
        goal2.x = -1.5;
        goal2.y = 0.5;
        goal3.x = 1.5;
        goal3.y = 0.5;
        goal4.x = -1.5;
        goal4.y = 0.5;
    }else if(uav_id == 3)
    {
        goal1.x = 1.5;
        goal1.y = -0.5;
        goal2.x = -1.5;
        goal2.y = -0.5;
        goal3.x = 1.5;
        goal3.y = -0.5;
        goal4.x = -1.5;
        goal4.y = -0.5;
    }else if(uav_id == 4)
    {
        goal1.x = 1.5;
        goal1.y = -1.5;
        goal2.x = -1.5;
        goal2.y = -1.5;
        goal3.x = 1.5;
        goal3.y = -1.5;
        goal4.x = -1.5;
        goal4.y = -1.5;
    }else if(uav_id == 5)
    {
        goal1.x = -1.5;
        goal1.y = 1.5;
        goal2.x = 1.5;
        goal2.y = 1.5;
        goal3.x = -1.5;
        goal3.y = 1.5;
        goal4.x = 1.5;
        goal4.y = 1.5;
    }else if(uav_id == 6)
    {
        goal1.x = -1.5;
        goal1.y = 0.5;
        goal2.x = 1.5;
        goal2.y = 0.5;
        goal3.x = -1.5;
        goal3.y = 0.5;
        goal4.x = 1.5;
        goal4.y = 0.5;
    }else if(uav_id == 7)
    {
        goal1.x = -1.5;
        goal1.y = -0.5;
        goal2.x = 1.5;
        goal2.y = -0.5;
        goal3.x = -1.5;
        goal3.y = -0.5;
        goal4.x = 1.5;
        goal4.y = -0.5;
    }else if(uav_id == 8)
    {
        goal1.x = -1.5;
        goal1.y = -1.5;
        goal2.x = 1.5;
        goal2.y = -1.5;
        goal3.x = -1.5;
        goal3.y = -1.5;
        goal4.x = 1.5;
        goal4.y = -1.5;
    }
    
}