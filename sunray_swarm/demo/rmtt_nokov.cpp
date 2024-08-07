#include <ros/ros.h>
#include <signal.h>
#include <thread>
#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"

#define MAX_AGENT_NUM 10   
    
using namespace std;

int rmtt_num;
string node_name;
bool pub_goal1_flag{false};
bool pub_goal2_flag{false};
bool pub_goal3_flag{false};
bool pub_goal4_flag{false};
bool pub_goal5_flag{false};
bool pub_return{false};

sunray_msgs::rmtt_state rmtt_state[MAX_AGENT_NUM];
sunray_msgs::rmtt_orca rmtt_orca_state[MAX_AGENT_NUM];
sunray_msgs::station_cmd station_cmd;
ros::Publisher station_cmd_pub;
ros::Publisher text_info_pub;
ros::Publisher rmtt_goal_pub[MAX_AGENT_NUM];
ros::Subscriber rmtt_state_sub[MAX_AGENT_NUM];
ros::Subscriber rmtt_orca_state_sub[MAX_AGENT_NUM];
ros::Subscriber station_cmd_sub;
geometry_msgs::Point goal1[MAX_AGENT_NUM],goal2[MAX_AGENT_NUM],goal3[MAX_AGENT_NUM],goal4[MAX_AGENT_NUM],goal5[MAX_AGENT_NUM];

void mySigintHandler(int sig)
{
    ROS_INFO("[rmtt_nokov] exit...");
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
}
void rmtt_state_cb(const sunray_msgs::rmtt_stateConstPtr& msg, int i)
{
    rmtt_state[i] = *msg;
}
void rmtt_orca_state_cb(const sunray_msgs::rmtt_orcaConstPtr& msg, int i)
{
    rmtt_orca_state[i] = *msg;
}
void timercb_show(const ros::TimerEvent &e);
void setup_show_goals();
int main(int argc, char **argv)
{
    ros::init(argc, argv, "rmtt_nokov");
    ros::NodeHandle nh("~");
    ros::Rate rate(20.0);
    node_name = ros::this_node::getName();

    // 【参数】无人机编号
    nh.param<int>("rmtt_num", rmtt_num, 5);

    string agent_name;
    // 【订阅】地面站指令
    station_cmd_sub = nh.subscribe<sunray_msgs::station_cmd>("/sunray_swarm/station_cmd", 1, station_cmd_cb);
    // 【发布】地面站指令
    station_cmd_pub = nh.advertise<sunray_msgs::station_cmd>("/sunray_swarm/station_cmd", 1);
    // 【发布】文字提示消息（回传至地面站显示）
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);

    for(int i = 0; i < rmtt_num; i++) 
    {
        agent_name = "/rmtt_" + std::to_string(i+1);
        // 【订阅】无人机状态数据
        rmtt_state_sub[i] = nh.subscribe<sunray_msgs::rmtt_state>("/sunray_swarm" + agent_name + "/rmtt_state", 1, boost::bind(&rmtt_state_cb,_1,i));
        // 【订阅】无人机orca状态 回传地面站
		rmtt_orca_state_sub[i] = nh.subscribe<sunray_msgs::rmtt_orca>("/sunray_swarm" + agent_name + "/rmtt_orca", 1, boost::bind(&rmtt_orca_state_cb,_1,i));
        // 【发布】无人机的目标点
        rmtt_goal_pub[i] = nh.advertise<geometry_msgs::Point>("/sunray_swarm" + agent_name + "/goal_point", 1);
    }

    ros::Timer timer_show = nh.createTimer(ros::Duration(3.0), timercb_show);

    setup_show_goals();

    pub_goal1_flag = false;
    pub_goal2_flag = false;
    pub_goal3_flag = false;
    pub_goal4_flag = false;
    pub_goal5_flag = false;
    pub_return = false;

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
        for(int i = 0; i < rmtt_num; i++) 
        {
            rmtt_goal_pub[i].publish(goal1[i]);
            cout << BLUE << node_name << " pub_goal1..." << TAIL << endl;
            sleep(0.1);
        }

        // orca_run
        station_cmd.mission_state = 5;
        station_cmd.uav_id = 99;
        station_cmd_pub.publish(station_cmd);

        // set flag
        pub_goal1_flag = false;
        pub_goal2_flag = true;
        pub_goal3_flag = false;
        pub_goal4_flag = false;
        pub_goal5_flag = false;
        pub_return = false;
    }

    // rmtt1 抵达初始目标点,先降落，然后起飞，然后发送下一个目标点
    if(pub_goal2_flag && rmtt_orca_state[0].arrived_all_goal)
    {
        sleep(5.0);
        // pub_new
        for(int i = 0; i < rmtt_num; i++) 
        {
            rmtt_goal_pub[i].publish(goal2[i]);
            cout << BLUE << node_name << " pub_goal2..." << TAIL << endl;
            sleep(0.1);
        }

        // set flag
        pub_goal1_flag = false;
        pub_goal2_flag = false;
        pub_goal3_flag = true;
        pub_goal4_flag = false;
        pub_goal5_flag = false;
        pub_return = false;
        rmtt_orca_state[0].arrived_all_goal = false;
    }

    if(pub_goal3_flag && rmtt_orca_state[0].arrived_all_goal)
    {
        sleep(5.0);
        // pub_new
        for(int i = 0; i < rmtt_num; i++) 
        {
            rmtt_goal_pub[i].publish(goal3[i]);
            cout << BLUE << node_name << " pub_goal3..." << TAIL << endl;
            sleep(0.1);
        }

        // set flag
        pub_goal1_flag = false;
        pub_goal2_flag = false;
        pub_goal3_flag = false;
        pub_goal4_flag = true;
        pub_goal5_flag = false;
        pub_return = false;
        rmtt_orca_state[0].arrived_all_goal = false;

    }

    if(pub_goal4_flag && rmtt_orca_state[0].arrived_all_goal)
    {
        sleep(5.0);
        // pub_new
        for(int i = 0; i < rmtt_num; i++) 
        {
            rmtt_goal_pub[i].publish(goal4[i]);
            cout << BLUE << node_name << " pub_goal4..." << TAIL << endl;
            sleep(0.1);
        }
        
        // set flag
        pub_goal1_flag = false;
        pub_goal2_flag = false;
        pub_goal3_flag = false;
        pub_goal4_flag = false;
        pub_goal5_flag = true;
        pub_return = false;
        rmtt_orca_state[0].arrived_all_goal = false;

    }

    if(pub_goal5_flag && rmtt_orca_state[0].arrived_all_goal)
    {
        sleep(5.0);
        // pub_new
        for(int i = 0; i < rmtt_num; i++) 
        {
            rmtt_goal_pub[i].publish(goal5[i]);
            cout << BLUE << node_name << " pub_goal5..." << TAIL << endl;
            sleep(0.1);
        }
        
        // set flag
        pub_goal1_flag = false;
        pub_goal2_flag = false;
        pub_goal3_flag = false;
        pub_goal4_flag = false;
        pub_goal5_flag = false;
        pub_return = true;
        rmtt_orca_state[0].arrived_all_goal = false;

    }

    if(pub_return && rmtt_orca_state[0].arrived_all_goal)
    {
        sleep(5.0);
        // land
        station_cmd.mission_state = 6;
        station_cmd.uav_id = 99;
        station_cmd_pub.publish(station_cmd);
        sleep(6.0);
        cout << BLUE << node_name << " pub return..." << TAIL << endl;

        pub_goal1_flag = false;
        pub_goal2_flag = false;
        pub_goal3_flag = false;
        pub_goal4_flag = false;
        pub_goal5_flag = false;
        pub_return = false;
    }

}

void setup_show_goals()
{
    // goal1[0].x = -1.5;
    // goal1[0].y = 1.0;
    // goal2[0].x = -1.5;
    // goal2[0].y = 1.0;
    // goal3[0].x = -1.5;
    // goal3[0].y = 0.5;
    // goal4[0].x = -1.5;
    // goal4[0].y = 1.0;
    // goal5[0].x = -2.0;
    // goal5[0].y = 0.0;

    // goal1[1].x = 1.5;
    // goal1[1].y = 1.0;
    // goal2[1].x = 0.5;
    // goal2[1].y = 1.0;
    // goal3[1].x = 0.0;
    // goal3[1].y = 0.5;
    // goal4[1].x = 0.5;
    // goal4[1].y = 1.0;
    // goal5[1].x = -0.5;
    // goal5[1].y = 1.0;

    // goal1[2].x = 0.0;
    // goal1[2].y = -0.0;
    // goal2[2].x = 1.5;
    // goal2[2].y = -0.0;
    // goal3[2].x = 1.5;
    // goal3[2].y = 0.5;
    // goal4[2].x = 1.5;
    // goal4[2].y = -0.0;
    // goal5[2].x = 1.2;
    // goal5[2].y = 1.5;

    // goal1[3].x = -1.5;
    // goal1[3].y = -1.0;
    // goal2[3].x = -1.5;
    // goal2[3].y = -1.0;
    // goal3[3].x = -1.5;
    // goal3[3].y = -0.5;
    // goal4[3].x = -1.5;
    // goal4[3].y = -1.0;
    // goal5[3].x = -0.5;
    // goal5[3].y = -1.0;

    // goal1[4].x = 1.5;
    // goal1[4].y = -1.0;
    // goal2[4].x = 0.5;
    // goal2[4].y = -1.0;
    // goal3[4].x = 1.5;
    // goal3[4].y = -0.5;
    // goal4[4].x = 0.5;
    // goal4[4].y = -1.0;
    // goal5[4].x = 1.2;
    // goal5[4].y = -1.5;

    // // 字母 N
    // goal1[0].x = -2.0; goal1[0].y = 1.5;   // 顶部左端
    // goal1[1].x = 2.0; goal1[1].y = 1.5;  // 底部左端
    // goal1[2].x = 0.0; goal1[2].y = 0.0;   // 中部交点
    // goal1[3].x = 2.0; goal1[3].y = -1.5;  // 顶部右端
    // goal1[4].x = -2.0; goal1[4].y = -1.5; // 底部右端

    // // 字母 O
    // goal2[0].x = 2.0;  goal2[0].y = 0.0;  // 上部中点
    // goal2[1].x = 0.0;  goal2[1].y = 2.0; // 下部中点
    // goal2[2].x = -1.4; goal2[2].y = 1.0;  // 上部边点
    // goal2[3].x = -1.4; goal2[3].y = -1.0; // 下部边点
    // goal2[4].x = 0.0;  goal2[4].y = -2.0;  // 中心点

    // // 字母 K
    // goal3[0].x = 2.0; goal3[0].y = 1.5;  // 上部交点
    // goal3[1].x = 0.0; goal3[1].y = 1.5; // 下部交点
    // goal3[2].x = -2.0; goal3[2].y = 1.5;  // 中部竖线
    // goal3[3].x = 1.5; goal3[3].y = -1.5;  // 斜线上端
    // goal3[4].x = -1.5; goal3[4].y = -1.5; // 斜线下端

    // goal4[0].x = 2.0;  goal4[0].y = 0.0;  // 上部中点
    // goal4[1].x = 0.0;  goal4[1].y = 2.0; // 下部中点
    // goal4[2].x = -1.4; goal4[2].y = 1.0;  // 上部边点
    // goal4[3].x = -1.4; goal4[3].y = -1.0; // 下部边点
    // goal4[4].x = 0.0;  goal4[4].y = -2.0;  // 中心点

    // // 字母 V
    // goal5[0].x = 2.0;  goal5[0].y = 2.0;  // 左上点
    // goal5[1].x = 0.0;  goal5[1].y = 1.0; // 右上点
    // goal5[2].x = -2.0;  goal5[2].y = 0.0;  // 底部点
    // goal5[3].x = 0.0;  goal5[3].y = -1;  // 左下点
    // goal5[4].x = 2.0;  goal5[4].y = -2.0; // 右下点

        // 字母 N
    goal1[0].x = -1.0; goal1[0].y = 1.5;   // 顶部左端
    goal1[1].x = 1.2; goal1[1].y = 1.5;  // 底部左端
    goal1[2].x = 0.1; goal1[2].y = 0.15;   // 中部交点
    goal1[3].x = 1.2; goal1[3].y = -1.3;  // 顶部右端
    goal1[4].x = -1.0; goal1[4].y = -1.3; // 底部右端

    // 字母 O
    goal2[0].x = 0.1;  goal2[0].y = 1.5;  // 上部中点
    goal2[1].x = 1.4;  goal2[1].y = 0.5; // 下部中点
    goal2[2].x = -0.8; goal2[2].y = 0.0;  // 上部边点
    goal2[3].x = 1.4; goal2[3].y = -0.5; // 下部边点
    goal2[4].x = 0.1;  goal2[4].y = -1.5;  // 中心点

    // 字母 K
    goal3[0].x = 0.1; goal3[0].y = 0.9;  // 上部交点
    goal3[1].x = 1.2; goal3[1].y = 0.9; // 下部交点
    goal3[2].x = -0.8; goal3[2].y = 0.9;  // 中部竖线
    goal3[3].x = 1.2; goal3[3].y = -0.9;  // 斜线上端
    goal3[4].x = -0.8; goal3[4].y = -0.9; // 斜线下端

    // 字母 O
    goal4[0].x = 0.1;  goal4[0].y = 1.5;  // 上部中点
    goal4[1].x = 1.4;  goal4[1].y = 0.5; // 下部中点
    goal4[2].x = -0.8; goal4[2].y = 0.0;  // 上部边点
    goal4[3].x = 1.4; goal4[3].y = -0.5; // 下部边点
    goal4[4].x = 0.1;  goal4[4].y = -1.5;  // 中心点

    // 字母 V
    goal5[0].x = 0.1;  goal5[0].y = 0.6;  // 左上点
    goal5[1].x = 1.1;  goal5[1].y = 1.1; // 右上点
    goal5[2].x = -0.7;  goal5[2].y = 0.0;  // 底部点
    goal5[3].x = 1.1;  goal5[3].y = -1.1;  // 左下点
    goal5[4].x = 0.1;  goal5[4].y = -0.6; // 右下点

}