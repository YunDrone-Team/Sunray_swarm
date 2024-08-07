#include <ros/ros.h>
#include <signal.h>

#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"

#define MAX_AGENT_NUM 100   
    
using namespace std;
int rmtt_num = 8;
sunray_msgs::rmtt_state rmtt_state[MAX_AGENT_NUM];
sunray_msgs::rmtt_orca rmtt_orca_state[MAX_AGENT_NUM];
std_msgs::Empty land;
std_msgs::Empty takeoff;
geometry_msgs::Point goal;
sunray_msgs::station_cmd station_cmd;
ros::Publisher station_cmd_pub;
ros::Publisher text_info_pub;
ros::Publisher rmtt_goal_pub[MAX_AGENT_NUM];
ros::Publisher takeoff_pub[MAX_AGENT_NUM];
ros::Publisher land_pub[MAX_AGENT_NUM];
ros::Subscriber rmtt_state_sub[MAX_AGENT_NUM];
ros::Subscriber rmtt_orca_state_sub[MAX_AGENT_NUM];

void rmtt_state_cb(const sunray_msgs::rmtt_stateConstPtr& msg, int i)
{
    rmtt_state[i] = *msg;
}
void rmtt_orca_state_cb(const sunray_msgs::rmtt_orcaConstPtr& msg, int i)
{
    rmtt_orca_state[i] = *msg;
}
void timercb_uav1(const ros::TimerEvent &e);
void timercb_uav2(const ros::TimerEvent &e);
void timercb_uav3(const ros::TimerEvent &e);
void timercb_uav4(const ros::TimerEvent &e);
void timercb_uav5(const ros::TimerEvent &e);
void timercb_uav6(const ros::TimerEvent &e);
void timercb_uav7(const ros::TimerEvent &e);
void timercb_uav8(const ros::TimerEvent &e);

void mySigintHandler(int sig)
{
    ROS_INFO("[rmtt_state_node] exit...");
    ros::shutdown();
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "rmtt_8uav");
    ros::NodeHandle nh("~");
    ros::Rate rate(50.0);

    // setupSubscribersAndPublishers(nh);
    // setupTimers(nh);
    

    // 【发布】地面站指令
    station_cmd_pub = nh.advertise<sunray_msgs::station_cmd>("/sunray_swarm/station_cmd", 1);
    // 【发布】文字提示消息（回传至地面站显示）
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);

    string agent_name;
    for(int i = 0; i < rmtt_num; i++) 
    {
        agent_name = "/rmtt_" + std::to_string(i+1);
        // 【订阅】无人机状态数据
        rmtt_state_sub[i] = nh.subscribe<sunray_msgs::rmtt_state>("/sunray_swarm" + agent_name + "/rmtt_state", 1, boost::bind(&rmtt_state_cb,_1,i));
        // 【订阅】无人机orca状态 回传地面站
		rmtt_orca_state_sub[i] = nh.subscribe<sunray_msgs::rmtt_orca>("/sunray_swarm" + agent_name + "/rmtt_orca", 1, boost::bind(&rmtt_orca_state_cb,_1,i));
        // 【发布】无人机的目标点
        rmtt_goal_pub[i] = nh.advertise<geometry_msgs::Point>("/sunray_swarm" + agent_name + "/goal_point", 1);
        // 【发布】无人机起飞指令
		takeoff_pub[i] = nh.advertise<std_msgs::Empty>("/sunray_swarm" + agent_name + "/takeoff", 1); 
        // 【发布】无人机降落指令
		land_pub[i] = nh.advertise<std_msgs::Empty>("/sunray_swarm" + agent_name + "/land", 1); 
    }

    ros::Timer timer_rmtt1 = nh.createTimer(ros::Duration(0.1), timercb_uav1);
    ros::Timer timer_rmtt2 = nh.createTimer(ros::Duration(0.1), timercb_uav2);
    ros::Timer timer_rmtt3 = nh.createTimer(ros::Duration(0.1), timercb_uav3);
    ros::Timer timer_rmtt4 = nh.createTimer(ros::Duration(0.1), timercb_uav4);
    ros::Timer timer_rmtt5 = nh.createTimer(ros::Duration(0.1), timercb_uav5);
    ros::Timer timer_rmtt6 = nh.createTimer(ros::Duration(0.1), timercb_uav6);
    ros::Timer timer_rmtt7 = nh.createTimer(ros::Duration(0.1), timercb_uav7);
    ros::Timer timer_rmtt8 = nh.createTimer(ros::Duration(0.1), timercb_uav8);

    // TAKEOFF
    station_cmd.mission_state = 1;
    station_cmd_pub.publish(station_cmd);
    sleep(10.0);
    // 发送一个预设模式，将rmtt_orca中预设模式改为所有飞机的初始期望点,此时rmtt_orca状态为ORCA_SETUP -> HOLD
    // 先发初始目标点，再发setup
    goal.x = 2.0;
    goal.y = -1.0;
    goal.z = 1.0;
    rmtt_goal_pub[0].publish(goal);
    goal.x = 3.0;
    goal.y = -1.0;
    goal.z = 1.0;
    rmtt_goal_pub[1].publish(goal);
    goal.x = 3.0;
    goal.y = -3.0;
    goal.z = 1.0;
    rmtt_goal_pub[2].publish(goal);
    goal.x = -1.0;
    goal.y = -1.0;
    goal.z = 1.0;
    rmtt_goal_pub[3].publish(goal);
    goal.x = -2.0;
    goal.y = -1.0;
    goal.z = 1.0;
    rmtt_goal_pub[4].publish(goal);
    goal.x = 2.0;
    goal.y = 1.0;
    goal.z = 1.0;
    rmtt_goal_pub[5].publish(goal);
    goal.x = 1.5;
    goal.y = -1.5;
    goal.z = 1.0;
    rmtt_goal_pub[6].publish(goal);
    goal.x = 2.0;
    goal.y = -3.5;
    goal.z = 1.0;
    rmtt_goal_pub[7].publish(goal);
    station_cmd.mission_state = 4;
    station_cmd.scenario_id = 99;
    station_cmd_pub.publish(station_cmd);

    // 发送ORCA_RUN指令，无人机开始执行初始期望点，此时rmtt_orca状态为ORCA_RUN
    station_cmd.mission_state = 5;
    station_cmd_pub.publish(station_cmd);

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

void setupTimers(ros::NodeHandle &nh) {
    // 创建各无人机的定时器
    ros::Timer timer_rmtt1 = nh.createTimer(ros::Duration(0.1), timercb_uav1);
    ros::Timer timer_rmtt2 = nh.createTimer(ros::Duration(0.1), timercb_uav2);

}


void timercb_uav1(const ros::TimerEvent &e)
{
    // // rmtt1 抵达初始目标点[0,-4],先降落，然后起飞，然后发送下一个目标点
    // if(rmtt_orca_state[0].arrived_goal && rmtt_orca_state[0].goal[0]==0.0 && rmtt_orca_state[0].goal[1]==-4.0)
    // {
    //     land_pub[0].publish(land);
    //     sleep(5.0);
    //     takeoff_pub[0].publish(takeoff);
    //     sleep(5.0);
    //     // 填写固定值
    //     goal.x = -2.0;
    //     goal.y = -3.0;
    //     goal.z = 1.0;
    //     rmtt_goal_pub[0].publish(goal);
    // }

    // // 检查无人机1是否到达第二个目标点
    // if(rmtt_orca_state[0].arrived_goal && rmtt_orca_state[0].goal[0]==2.0 && rmtt_orca_state[0].goal[1]==-2.0)
    // {
    //     land_pub[0].publish(land);
    //     sleep(5.0);
    //     takeoff_pub[0].publish(takeoff);
    //     sleep(5.0);
    //     // 更新到第三个目标点
    //     goal.x = 1.0;
    //     goal.y = -3.0;
    //     goal.z = 1.0;
    //     rmtt_goal_pub[0].publish(goal);
    // }
    // // 检查无人机1是否到达第三个目标点
    // if(rmtt_orca_state[0].arrived_goal && rmtt_orca_state[0].goal[0]==1.0 && rmtt_orca_state[0].goal[1]==-3.0)
    // {
    //     land_pub[0].publish(land);
    //     sleep(5.0);
    //     takeoff_pub[0].publish(takeoff);
    //     sleep(5.0);
    //     // 进行任务完成的处理或返回初始点
    // }

    // rmtt1 抵达初始目标点[0,-4],先降落，然后起飞，然后发送下一个目标点
    if(rmtt_orca_state[0].arrived_goal && rmtt_orca_state[0].goal[0]==0.0 && rmtt_orca_state[0].goal[1]==-4.0)
    {
        land_pub[0].publish(land);
        sleep(5.0);
        takeoff_pub[0].publish(takeoff);
        sleep(5.0);
        // 填写固定值
        goal.x = 2.0;
        goal.y = -1.0;
        goal.z = 1.0;
        rmtt_goal_pub[0].publish(goal);
    }

    // rmtt1 抵达第二个目标点,先降落，然后起飞，然后发送第三个目标点
    if(rmtt_orca_state[0].arrived_goal && rmtt_orca_state[0].goal[0]==2.0 && rmtt_orca_state[0].goal[1]==-1.0)
    {
        land_pub[0].publish(land);
        sleep(5.0);
        takeoff_pub[0].publish(takeoff);
        sleep(5.0);
        // 填写固定值
        goal.x = 2.0;
        goal.y = -3.0;
        goal.z = 1.0;
        rmtt_goal_pub[0].publish(goal);
    }

    // rmtt1 抵达第三个目标点,先降落，然后起飞，然后发送第四个目标点
    if(rmtt_orca_state[0].arrived_goal && rmtt_orca_state[0].goal[0]==2.0 && rmtt_orca_state[0].goal[1]==-2.0)
    {
        land_pub[0].publish(land);
        sleep(5.0);
        takeoff_pub[0].publish(takeoff);
        sleep(5.0);
        // 填写固定值
        goal.x = 1.0;
        goal.y = 3.0;
        goal.z = 1.0;
        rmtt_goal_pub[0].publish(goal);
    }
    if(rmtt_orca_state[0].arrived_goal && rmtt_orca_state[0].goal[0]==3.0 && rmtt_orca_state[0].goal[1]==-3.0)
    {
        land_pub[0].publish(land);
        sleep(5.0);
        takeoff_pub[0].publish(takeoff);
        sleep(5.0);
        // 填写固定值
        goal.x = 1.0;
        goal.y = -3.0;
        goal.z = 1.0;
        rmtt_goal_pub[3].publish(goal);
    }
    if(rmtt_orca_state[0].arrived_goal && rmtt_orca_state[0].goal[0]==1.0 && rmtt_orca_state[0].goal[1]==-3.0)
    {
        land_pub[2].publish(land);
        sleep(5.0);
        takeoff_pub[2].publish(takeoff);
        sleep(5.0);
    }
    
}
void timercb_uav2(const ros::TimerEvent &e)
{
    // 同上
    if(rmtt_orca_state[1].arrived_goal && rmtt_orca_state[1].goal[0]==2.0 && rmtt_orca_state[1].goal[1]==-1.0)
    {
        land_pub[1].publish(land);
        sleep(5.0);
        takeoff_pub[1].publish(takeoff);
        sleep(5.0);
        goal.x = 2.0;
        goal.y = -2.0;
        goal.z = 1.0;
        rmtt_goal_pub[1].publish(goal);
    }
    if(rmtt_orca_state[1].arrived_goal && rmtt_orca_state[1].goal[0]==2.0 && rmtt_orca_state[1].goal[1]==-2.0)
    {
        land_pub[1].publish(land);
        sleep(5.0);
        takeoff_pub[1].publish(takeoff);
        sleep(5.0);
        goal.x = 1.0;
        goal.y = -3.0;
        goal.z = 1.0;
        rmtt_goal_pub[1].publish(goal);
    }
    if(rmtt_orca_state[1].arrived_goal && rmtt_orca_state[1].goal[0]==1.0 && rmtt_orca_state[1].goal[1]==-3.0)
    {
        land_pub[1].publish(land);
        sleep(5.0);
        takeoff_pub[1].publish(takeoff);
        sleep(5.0);
    }

    
}
void timercb_uav3(const ros::TimerEvent &e)
{
        // 同上
    if(rmtt_orca_state[2].arrived_goal && rmtt_orca_state[2].goal[0]==2.0 && rmtt_orca_state[2].goal[1]==-1.0)
    {
        land_pub[2].publish(land);
        sleep(5.0);
        takeoff_pub[2].publish(takeoff);
        sleep(5.0);
        goal.x = 2.0;
        goal.y = -2.0;
        goal.z = 1.0;
        rmtt_goal_pub[2].publish(goal);
    }
    if(rmtt_orca_state[2].arrived_goal && rmtt_orca_state[2].goal[0]==2.0 && rmtt_orca_state[2].goal[1]==-2.0)
    {
        land_pub[2].publish(land);
        sleep(5.0);
        takeoff_pub[2].publish(takeoff);
        sleep(5.0);
        goal.x = 1.0;
        goal.y = -3.0;
        goal.z = 1.0;
        rmtt_goal_pub[2].publish(goal);
    }
    if(rmtt_orca_state[2].arrived_goal && rmtt_orca_state[2].goal[0]==1.0 && rmtt_orca_state[2].goal[1]==-3.0)
    {
        land_pub[2].publish(land);
        sleep(5.0);
        takeoff_pub[2].publish(takeoff);
        sleep(5.0);
    }


}
void timercb_uav4(const ros::TimerEvent &e)
{
    if(rmtt_orca_state[3].arrived_goal && rmtt_orca_state[3].goal[0]==2.0 && rmtt_orca_state[3].goal[1]==-1.0)
    {
        land_pub[3].publish(land);
        sleep(5.0);
        takeoff_pub[3].publish(takeoff);
        sleep(5.0);
        goal.x = 2.0;
        goal.y = -2.0;
        goal.z = 1.0;
        rmtt_goal_pub[3].publish(goal);
    }
}
void timercb_uav5(const ros::TimerEvent &e)
{
    if(rmtt_orca_state[4].arrived_goal && rmtt_orca_state[4].goal[0]==2.0 && rmtt_orca_state[4].goal[1]==-1.0)
    {
        land_pub[4].publish(land);
        sleep(5.0);
        takeoff_pub[4].publish(takeoff);
        sleep(5.0);
        goal.x = 2.0;
        goal.y = -2.0;
        goal.z = 1.0;
        rmtt_goal_pub[4].publish(goal);
    }
}
void timercb_uav6(const ros::TimerEvent &e)
{
    if(rmtt_orca_state[5].arrived_goal && rmtt_orca_state[5].goal[0]==2.0 && rmtt_orca_state[5].goal[1]==-1.0)
    {
        land_pub[5].publish(land);
        sleep(5.0);
        takeoff_pub[5].publish(takeoff);
        sleep(5.0);
        goal.x = 2.0;
        goal.y = -2.0;
        goal.z = 1.0;
        rmtt_goal_pub[5].publish(goal);
    }
}
void timercb_uav7(const ros::TimerEvent &e)
{
    if(rmtt_orca_state[6].arrived_goal && rmtt_orca_state[6].goal[0]==2.0 && rmtt_orca_state[6].goal[1]==-1.0)
    {
        land_pub[6].publish(land);
        sleep(5.0);
        takeoff_pub[6].publish(takeoff);
        sleep(5.0);
        goal.x = 2.0;
        goal.y = -2.0;
        goal.z = 1.0;
        rmtt_goal_pub[6].publish(goal);
    }
}
void timercb_uav8(const ros::TimerEvent &e)
{
    if(rmtt_orca_state[7].arrived_goal && rmtt_orca_state[7].goal[0]==2.0 && rmtt_orca_state[7].goal[1]==-1.0)
    {
        land_pub[7].publish(land);
        sleep(5.0);
        takeoff_pub[7].publish(takeoff);
        sleep(5.0);
        goal.x = 2.0;
        goal.y = -2.0;
        goal.z = 1.0;
        rmtt_goal_pub[7].publish(goal);
    }
}