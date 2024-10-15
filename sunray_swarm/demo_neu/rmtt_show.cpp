#include <ros/ros.h>
#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"

#define MAX_AGENT_NUM 10


using namespace std;

int rmtt_num = 8;
int agent_id;
string node_name;
bool pub_goal1_flag{false};
bool pub_goal2_flag{false};
bool pub_goal3_flag{false};
bool pub_goal4_flag{false};
bool pub_return{false};
float thres = 0.1;


sunray_msgs::agent_state rmtt_state;
//display
sunray_msgs::orca_state agent_orca_state[MAX_AGENT_NUM];
// sunray_msgs::orca_state agent_orca_state;
sunray_msgs::agent_cmd station_cmd; 

ros::Publisher station_cmd_pub; // 地面站命令发布者
ros::Publisher text_info_pub;  // 文字信息发布者
// ros::Publisher rmtt_goal_pub;   // 无人机目标点发布者


ros::Publisher orca_goal_pub[MAX_AGENT_NUM];        // 发布ORCA目标点
ros::Subscriber orca_state_sub[MAX_AGENT_NUM];      // 订阅ORCA状态

ros::Subscriber rmtt_state_sub; // 无人机状态订阅者
ros::Subscriber agent_orca_state_sub; // 无人机ORCA状态订阅者
ros::Subscriber station_cmd_sub;   // 地面站命令订阅者
geometry_msgs::Point goal[MAX_AGENT_NUM];  // 各个无人机目标点数组
// 设置新接受的新点
geometry_msgs::Point new_goal;  // 新接收到的目标点
ros::Subscriber rmtt_goal_sub;  // 目标点订阅者
//发布目标点序号
ros::Publisher goal_id_pub;

// bool all_uavs_finished[MAX_AGENT_NUM] = {false};//跟踪每架无人机任务点

enum CONTROL_STATE
{
    INIT = 0,
    HOLD = 1,
    POS_CONTROL = 2,
    VEL_CONTROL_BODY = 3,
    VEL_CONTROL_ENU = 4,
    TAKEOFF = 11,
    LAND = 12,
    ORCA_CMD = 99,
};
CONTROL_STATE control_state;

int goal_id{1};
//无人机任务点完成情况
// void markTaskAsFinished(int agent_id)
// {
//     all_uavs_finished[agent_id] = true;
// }
// //判断所有无人机任务点情况，触发返航逻辑
// bool areAllUavsFinished()
// {
//     for (int i = 0; i < rmtt_num; ++i)
//     {
//         if (!all_uavs_finished[i])
//             return false;
//     }
//     return true;
// }

//无人机目标点序号id
void publish_goal_id(int goal_id)
{
    std_msgs::UInt32 msg;
    msg.data = goal_id;
    goal_id_pub.publish(msg);
    cout << BLUE << node_name << ": Published goal ID: " << goal_id << TAIL << endl;
}

// 自定义目标点更新
void rmtt_goal_cb(const geometry_msgs::Point::ConstPtr &msg)
{
    new_goal = *msg;

    //z作为索引，指定哪个无人机接受
    goal[(int)new_goal.z].x = new_goal.x;
    goal[(int)new_goal.z].y = new_goal.y;
    goal[(int)new_goal.z].z = 1.0;


    //只更改下一个目标点
    // if(goal_id>4)
    // {
    //     return;
    // }

    // next
    // goal[(int)goal_id].x = new_goal.x;
    // goal[(int)goal_id].y = new_goal.y;
    // goal[(int)goal_id].z = 1.0;
}

void mySigintHandler(int sig)
{
    ROS_INFO("[rmtt_show] exit...");
    ros::shutdown();
}
void station_cmd_cb(const sunray_msgs::agent_cmdConstPtr &msg)
{
    if (msg->control_state == sunray_msgs::agent_cmd::TAKEOFF && msg->agent_id == 99)
    {
        pub_goal1_flag = true;
        cout << BLUE << node_name << " start..." << TAIL << endl;
        sleep(6.0);
    }
    // 收到return指令，则重置所有标志位，退出当前任务
    if (msg->control_state == 99 && msg->agent_id == 1)
    {
        pub_goal1_flag = false;
        pub_goal2_flag = false;
        pub_goal3_flag = false;
        pub_goal4_flag = false;
        pub_return = false;
        for (int i = 0; i < rmtt_num; i++)
        {
            agent_orca_state[i].arrived_goal = false;
        }
        
        cout << BLUE << node_name << " get return home..." << TAIL << endl;
    }
}
void rmtt_state_cb(const sunray_msgs::agent_stateConstPtr &msg)
{
    rmtt_state = *msg;
}
void rmtt_orca_state_cb(const sunray_msgs::orca_stateConstPtr &msg)
{
    for (int i = 0; i < rmtt_num; i++)
        {
            agent_orca_state[i] = *msg;
        }
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
    nh.param<int>("agent_id", agent_id, 1);

    string agent_name = "/rmtt_" + std::to_string(agent_id);
    // string agent_name;
    // 【订阅】地面站指令
    station_cmd_sub = nh.subscribe<sunray_msgs::agent_cmd>("/sunray_swarm"+ agent_name + "/agent_cmd", 1, station_cmd_cb);
    // 【订阅】无人机状态数据
    rmtt_state_sub = nh.subscribe<sunray_msgs::agent_state>("/sunray_swarm" + agent_name + "/agent_state", 1, rmtt_state_cb);
    // 【订阅】无人机orca状态 回传地面站
    agent_orca_state_sub = nh.subscribe<sunray_msgs::orca_state>("/sunray_swarm" + agent_name + "/orca_state", 1, rmtt_orca_state_cb);
    // // 【发布】无人机的目标点
    // rmtt_goal_pub = nh.advertise<geometry_msgs::Point>("/sunray_swarm" + agent_name + "/goal_point", 1);

    // 为每个智能体创建发布和订阅
    for(int i = 0; i < agent_num; i++) 
    {
        agent_name = "/" + agent_prefix + std::to_string(i+1);
        // 【发布】无人机的目标点
        orca_goal_pub[i] = nh.advertise<geometry_msgs::Point>("/sunray_swarm" + agent_name + "/goal_point", 1);
        // 【订阅】无人机orca状态
		orca_state_sub[i] = nh.subscribe<sunray_msgs::orca_state>("/sunray_swarm" + agent_name + "/agent_orca_state", 1, boost::bind(&rmtt_orca_state_cb,_1,i));
    }

    // 【订阅】无人机的目标点display
    // rmtt_goal_sub = nh.advertise<sunray_msgs::orca_cmd>("/sunray_swarm" + agent_name + "/goal_point", 1, rmtt_orca_goals_cb);
    // 【发布】地面站指令
    station_cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 1);
    // 【发布】文字提示消息（回传至地面站显示）
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);

    ros::Timer timer_show = nh.createTimer(ros::Duration(3.0), timercb_show);

    // 订阅无人机目标点
    rmtt_goal_sub = nh.subscribe<geometry_msgs::Point>("/sunray_swarm" + agent_name + "/goal_point_show", 1, rmtt_goal_cb);
    // 发布目标点序号
    goal_id_pub = nh.advertise<std_msgs::UInt32>("/sunray_swarm" + agent_name + "/goal_id", 1);


    setup_show_goals();

    pub_goal1_flag = false;
    pub_goal2_flag = false;
    pub_goal3_flag = false;
    pub_goal4_flag = false;
    pub_return = false;
    for (int i = 0; i < rmtt_num; i++)
    {
        agent_orca_state[i].arrived_goal = false;
    }
    

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

    // cout << BLUE << node_name << " id: " << agent_id << "..." << TAIL << endl;
    // cout << BLUE << node_name << " pub_goal1... X: " << goal[1].x << " Y: " << goal[1].y << TAIL << endl;
    // cout << BLUE << node_name << " pub_goal2... X: " << goal[2].x << " Y: " << goal[2].y << TAIL << endl;
    // cout << BLUE << node_name << " pub_goal3... X: " << goal[3].x << " Y: " << goal[3].y << TAIL << endl;
    // cout << BLUE << node_name << " pub_goal4... X: " << goal[4].x << " Y: " << goal[4].y << TAIL << endl;

    if (pub_goal1_flag)
    {
        
        rmtt_goal_pub.publish();
        for(int i = 0; i < agent_num; i++) 
        {
            orca_goal_pub[i].publish(goal[i]);    // 发布N形队形目标点
            sleep(0.1);                             // 延迟
        }
        cout << BLUE << node_name << " pub_goal1..." << TAIL << endl;
        sleep(1.0);

        // orca_run
        station_cmd.control_state = sunray_msgs::agent_cmd::LAND;
        station_cmd.agent_id = 99;
        station_cmd_pub.publish(station_cmd);

        // set flag
        pub_goal1_flag = false;
        pub_goal2_flag = true;
        pub_goal3_flag = false;
        pub_goal4_flag = false;
        pub_return = false;

        agent_orca_state[1].arrived_goal = false;
        goal_id = 1;
        publish_goal_id(goal_id);
        
        // cout<<agent_orca_state[1].goal<<endl;

        cout<<"pub_goal1_flag"<<endl;
    }
    // rmtt1 抵达初始目标点,先降落，然后起飞，然后发送下一个目标点
    // if (pub_goal2_flag && agent_orca_state.arrived_goal && abs(agent_orca_state.goal[0] - goal[1].x)<thres && abs(agent_orca_state.goal[1] - goal[1].y)<thres)
    if (pub_goal2_flag && !agent_orca_state[1].arrived_goal && abs(agent_orca_state[1].goal[0] - goal[1].x)<thres && abs(agent_orca_state[1].goal[1] - goal[1].y)<thres)
    {
        sleep(0.5);
        // land
        station_cmd.control_state = sunray_msgs::agent_cmd::LAND;
        station_cmd.agent_id = 99;
        station_cmd_pub.publish(station_cmd);
        // 等待15秒后发布降落指令
        ros::Duration(6.0).sleep();

        // takeoff
        station_cmd.control_state = sunray_msgs::agent_cmd::TAKEOFF ;
        station_cmd.agent_id = agent_id;
        station_cmd_pub.publish(station_cmd);
        // 等待15秒后发布降落指令
        ros::Duration(6.0).sleep();

        // pub_new
        rmtt_goal_pub.publish(goal[2]);
        cout << BLUE << node_name << " pub_goal2..." << TAIL << endl;

        // orca_run
        station_cmd.control_state = 99;
        station_cmd.agent_id = agent_id;
        station_cmd_pub.publish(station_cmd);

        // set flag
        pub_goal1_flag = false;
        pub_goal2_flag = false;
        pub_goal3_flag = true;
        pub_goal4_flag = false;
        pub_return = false;
        agent_orca_state[2].arrived_goal = false;
        goal_id = 2;
        publish_goal_id(goal_id);
        cout<<"pub_goal2_flag"<<endl;
    }

    if (pub_goal3_flag && !agent_orca_state[2].arrived_goal )
    {
        // cout << " pub_goal1..." << goal[2].x<< goal[1].y << endl;
        // cout << " pub_goal1..." << agent_orca_state.goal[0]<< agent_orca_state.goal[1] << endl;
        // land
        sleep(0.5);
        station_cmd.control_state = LAND;
        station_cmd.agent_id = agent_id;
        station_cmd_pub.publish(station_cmd);
        sleep(6.0);

        // takeoff
        station_cmd.control_state = TAKEOFF;
        station_cmd.agent_id = agent_id;
        station_cmd_pub.publish(station_cmd);
        sleep(6.0);

        // pub_new
        rmtt_goal_pub.publish(goal[3]);
        cout << BLUE << node_name << " pub_goal3..." << TAIL << endl;

        // orca_run
        // station_cmd.control_state = 99;
        station_cmd.agent_id = agent_id;
        station_cmd_pub.publish(station_cmd);

        // set flag
        pub_goal1_flag = false;
        pub_goal2_flag = false;
        pub_goal3_flag = false;
        pub_goal4_flag = true;
        pub_return = false;
        agent_orca_state[3].arrived_goal = false;
        goal_id = 3;
        publish_goal_id(goal_id);
        cout<<"pub_goal3_flag"<<endl;
    }

    if (pub_goal4_flag && !agent_orca_state[3].arrived_goal )
    {
        // land
        station_cmd.control_state == sunray_msgs::agent_cmd::LAND;
        station_cmd.agent_id = agent_id;
        station_cmd_pub.publish(station_cmd);
        sleep(6.0);

        // takeoff
        station_cmd.control_state == sunray_msgs::agent_cmd::TAKEOFF;
        station_cmd.agent_id = agent_id;
        station_cmd_pub.publish(station_cmd);
        sleep(6.0);

        // pub_new
        rmtt_goal_pub.publish(goal[4]);
        cout << BLUE << node_name << " pub_goal4..." << TAIL << endl;

        // orca_run
        // station_cmd.control_state = 99;
        station_cmd.agent_id = agent_id;
        station_cmd_pub.publish(station_cmd);

        // set flag
        pub_goal1_flag = false;
        pub_goal2_flag = false;
        pub_goal3_flag = false;
        pub_goal4_flag = false;
        pub_return = false;
        agent_orca_state[4].arrived_goal = false;
        goal_id = 4;
        publish_goal_id(goal_id);
        cout<<"pub_goal4_flag"<<endl;
    }

    if (pub_return && agent_orca_state[4].arrived_goal)
    {

        // land
        station_cmd.control_state == sunray_msgs::agent_cmd::LAND;
        station_cmd.agent_id = agent_id;
        station_cmd_pub.publish(station_cmd);
        sleep(6.0);

        // takeoff
        // station_cmd.control_state = 6;
        // station_cmd.agent_id = agent_id;
        // station_cmd_pub.publish(station_cmd);
        // sleep(6.0);

        pub_goal1_flag = false;
        pub_goal2_flag = false;
        pub_goal3_flag = false;
        pub_goal4_flag = false;
        pub_return = false;
    }
}



void setup_show_goals()
{
    if (agent_id == 1)
    {
        goal[1].x = 1.5;
        goal[1].y = 1.5;
        goal[2].x = -1.5;
        goal[2].y = 1.5;
        goal[3].x = 1.5;
        goal[3].y = 1.5;
        goal[4].x = -1.5;
        goal[4].y = 1.5;
    }
    else if (agent_id == 2)
    {
        goal[1].x = 1.5;
        goal[1].y = 0.5;
        goal[2].x = -1.5;
        goal[2].y = 0.5;
        goal[3].x = 1.5;
        goal[3].y = 0.5;
        goal[4].x = -1.5;
        goal[4].y = 0.5;
    }
    else if (agent_id == 3)
    {
        goal[1].x = 1.5;
        goal[1].y = -0.5;
        goal[2].x = -1.5;
        goal[2].y = -0.5;
        goal[3].x = 1.5;
        goal[3].y = -0.5;
        goal[4].x = -1.5;
        goal[4].y = -0.5;
    }
    else if (agent_id == 4)
    {
        goal[1].x = 1.5;
        goal[1].y = -1.5;
        goal[2].x = -1.5;
        goal[2].y = -1.5;
        goal[3].x = 1.5;
        goal[3].y = -1.5;
        goal[4].x = -1.5;
        goal[4].y = -1.5;
    }
    else if (agent_id == 5)
    {
        goal[1].x = -1.5;
        goal[1].y = 1.5;
        goal[2].x = 1.5;
        goal[2].y = 1.5;
        goal[3].x = -1.5;
        goal[3].y = 1.5;
        goal[4].x = 1.5;
        goal[4].y = 1.5;
    }
    else if (agent_id == 6)
    {
        goal[1].x = -1.5;
        goal[1].y = 0.5;
        goal[2].x = 1.5;
        goal[2].y = 0.5;
        goal[3].x = -1.5;
        goal[3].y = 0.5;
        goal[4].x = 1.5;
        goal[4].y = 0.5;
    }
}





