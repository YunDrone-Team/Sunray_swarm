// "操作人员在地面站选择1-6号飞机，依次完成如下操作：
// 地面站读取预设的6个目标点，可以选择直接使用预设的6个目标点或者修改其中任意一个or多个目标点，
// 按下启动之后该无人机开始执行任务，依次飞行1-6号点，飞完后返回起飞点"

#include <ros/ros.h>
#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"
#include "rmtt_control.h"

#define MAX_AGENT_NUM 10


using namespace std;

// int rmtt_num = 8;
int agent_id;
string node_name;
bool pub_goal1_flag{false};
bool pub_goal2_flag{false};
bool pub_goal3_flag{false};
bool pub_goal4_flag{false};
bool pub_return{false};
float thres = 0.1;


sunray_msgs::agent_state rmtt_state;
sunray_msgs::orca_state rmtt_orca_state;
sunray_msgs::agent_cmd station_cmd;
ros::Publisher station_cmd_pub;
ros::Publisher text_info_pub;
ros::Publisher rmtt_goal_pub;
ros::Subscriber rmtt_state_sub;
ros::Subscriber rmtt_orca_state_sub;
ros::Subscriber station_cmd_sub;
geometry_msgs::Point goal[MAX_AGENT_NUM];
// 设置新接受的新点
geometry_msgs::Point new_goal;
ros::Subscriber rmtt_goal_sub;
//发布目标点序号
ros::Publisher goal_id_pub;

// bool all_uavs_finished[MAX_AGENT_NUM] = {false};//跟踪每架无人机任务点

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
    if (msg->control_state == 1 && msg->agent_id == 99)
    {
        pub_goal1_flag = true;
        cout << BLUE << node_name << " start..." << TAIL << endl;
        sleep(6.0);
    }
    // 收到return指令，则重置所有标志位，退出当前任务
    if (msg->control_state == 6 && msg->agent_id == 99)
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
void rmtt_state_cb(const sunray_msgs::agent_stateConstPtr &msg)
{
    rmtt_state = *msg;
}
void rmtt_orca_state_cb(const sunray_msgs::orca_stateConstPtr &msg)
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
    nh.param<int>("agent_id", agent_id, 1);

    string agent_name = "/rmtt_" + std::to_string(agent_id);
    // 【订阅】地面站指令
    station_cmd_sub = nh.subscribe<sunray_msgs::agent_cmd>("/sunray_rmtt/station_cmd", 1, station_cmd_cb);
    // 【订阅】无人机状态数据
    rmtt_state_sub = nh.subscribe<sunray_msgs::agent_state>("/sunray_rmtt" + agent_name + "/rmtt_state", 1, rmtt_state_cb);
    // 【订阅】无人机orca状态 回传地面站
    rmtt_orca_state_sub = nh.subscribe<sunray_msgs::orca_state>("/sunray_rmtt" + agent_name + "/rmtt_orca", 1, rmtt_orca_state_cb);
    // 【发布】无人机的目标点
    rmtt_goal_pub = nh.advertise<geometry_msgs::Point>("/sunray_rmtt" + agent_name + "/goal_point", 1);
    // 【发布】地面站指令
    station_cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_rmtt" + agent_name + "/station_cmd", 1);
    // 【发布】文字提示消息（回传至地面站显示）
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_rmtt/text_info", 1);

    ros::Timer timer_show = nh.createTimer(ros::Duration(3.0), timercb_show);

    // 订阅无人机目标点
    rmtt_goal_sub = nh.subscribe<geometry_msgs::Point>("/sunray_rmtt" + agent_name + "/goal_point_show", 1, rmtt_goal_cb);
    // 发布目标点序号
    goal_id_pub = nh.advertise<std_msgs::UInt32>("/sunray_rmtt" + agent_name + "/goal_id", 1);


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

    // cout << BLUE << node_name << " id: " << agent_id << "..." << TAIL << endl;
    // cout << BLUE << node_name << " pub_goal1... X: " << goal[1].x << " Y: " << goal[1].y << TAIL << endl;
    // cout << BLUE << node_name << " pub_goal2... X: " << goal[2].x << " Y: " << goal[2].y << TAIL << endl;
    // cout << BLUE << node_name << " pub_goal3... X: " << goal[3].x << " Y: " << goal[3].y << TAIL << endl;
    // cout << BLUE << node_name << " pub_goal4... X: " << goal[4].x << " Y: " << goal[4].y << TAIL << endl;

    if (pub_goal1_flag)
    {
        
        rmtt_goal_pub.publish(goal[1]);
        cout << BLUE << node_name << " pub_goal1..." << TAIL << endl;
        sleep(1.0);

        // orca_run
        station_cmd.control_state = 5;
        station_cmd.agent_id = agent_id;
        station_cmd_pub.publish(station_cmd);

        // set flag
        pub_goal1_flag = false;
        pub_goal2_flag = true;
        pub_goal3_flag = false;
        pub_goal4_flag = false;
        pub_return = false;
        rmtt_orca_state.arrived_goal = false;
        goal_id = 1;
        publish_goal_id(goal_id);
    }

    // rmtt1 抵达初始目标点,先降落，然后起飞，然后发送下一个目标点
    if (pub_goal2_flag && rmtt_orca_state.arrived_goal && abs(rmtt_orca_state.goal[0] - goal[1].x)<thres && abs(rmtt_orca_state.goal[1] - goal[1].y)<thres)
    {

        sleep(0.5);
        // land
        station_cmd.control_state = 2;
        station_cmd.agent_id = agent_id;
        station_cmd_pub.publish(station_cmd);
        sleep(6.0);

        // takeoff
        station_cmd.control_state = 1;
        station_cmd.agent_id = agent_id;
        station_cmd_pub.publish(station_cmd);
        sleep(6.0);

        // pub_new
        rmtt_goal_pub.publish(goal[2]);
        cout << BLUE << node_name << " pub_goal2..." << TAIL << endl;

        // orca_run
        station_cmd.control_state = 5;
        station_cmd.agent_id = agent_id;
        station_cmd_pub.publish(station_cmd);

        // set flag
        pub_goal1_flag = false;
        pub_goal2_flag = false;
        pub_goal3_flag = true;
        pub_goal4_flag = false;
        pub_return = false;
        rmtt_orca_state.arrived_goal = false;
        goal_id = 2;
        publish_goal_id(goal_id);
    }

    if (pub_goal3_flag && rmtt_orca_state.arrived_goal && abs(rmtt_orca_state.goal[0] - goal[2].x)<thres  && abs(rmtt_orca_state.goal[1] - goal[2].y)<thres )
    {
        // cout << " pub_goal1..." << goal[2].x<< goal[1].y << endl;
        // cout << " pub_goal1..." << rmtt_orca_state.goal[0]<< rmtt_orca_state.goal[1] << endl;
        // land
        sleep(0.5);
        station_cmd.control_state = 2;
        station_cmd.agent_id = agent_id;
        station_cmd_pub.publish(station_cmd);
        sleep(6.0);

        // takeoff
        station_cmd.control_state = 1;
        station_cmd.agent_id = agent_id;
        station_cmd_pub.publish(station_cmd);
        sleep(6.0);

        // pub_new
        rmtt_goal_pub.publish(goal[3]);
        cout << BLUE << node_name << " pub_goal3..." << TAIL << endl;

        // orca_run
        station_cmd.control_state = 5;
        station_cmd.agent_id = agent_id;
        station_cmd_pub.publish(station_cmd);

        // set flag
        pub_goal1_flag = false;
        pub_goal2_flag = false;
        pub_goal3_flag = false;
        pub_goal4_flag = true;
        pub_return = false;
        rmtt_orca_state.arrived_goal = false;
        goal_id = 3;
        publish_goal_id(goal_id);
    }

    if (pub_goal4_flag && rmtt_orca_state.arrived_goal && abs(rmtt_orca_state.goal[0] - goal[3].x)<thres  && abs(rmtt_orca_state.goal[1] - goal[3].y)<thres )
    {
        // land
        station_cmd.control_state = 2;
        station_cmd.agent_id = agent_id;
        station_cmd_pub.publish(station_cmd);
        sleep(6.0);

        // takeoff
        station_cmd.control_state = 1;
        station_cmd.agent_id = agent_id;
        station_cmd_pub.publish(station_cmd);
        sleep(6.0);

        // pub_new
        rmtt_goal_pub.publish(goal[4]);
        cout << BLUE << node_name << " pub_goal4..." << TAIL << endl;

        // orca_run
        station_cmd.control_state = 5;
        station_cmd.agent_id = agent_id;
        station_cmd_pub.publish(station_cmd);

        // set flag
        pub_goal1_flag = false;
        pub_goal2_flag = false;
        pub_goal3_flag = false;
        pub_goal4_flag = false;
        pub_return = true;
        rmtt_orca_state.arrived_goal = false;
        goal_id = 4;
        publish_goal_id(goal_id);
    }

    // if (pub_return && rmtt_orca_state.arrived_goal && abs(rmtt_orca_state.goal[0] - goal[4].x)<thres  && abs(rmtt_orca_state.goal[1] - goal[4].y)<thres )
    // {

    //     // takeoff
    //     station_cmd.control_state = 1;
    //     station_cmd.agent_id = agent_id;
    //     station_cmd_pub.publish(station_cmd);
    //     sleep(6.0);

    //     // takeoff
    //     station_cmd.control_state = 6;
    //     station_cmd.agent_id = agent_id;
    //     station_cmd_pub.publish(station_cmd);
    //     sleep(6.0);



    //     pub_goal1_flag = false;
    //     pub_goal2_flag = false;
    //     pub_goal3_flag = false;
    //     pub_goal4_flag = false;
    //     pub_return = false;
    // }
    if (pub_return && rmtt_orca_state.arrived_goal && abs(rmtt_orca_state.goal[0] - goal[3].x)<thres  && abs(rmtt_orca_state.goal[1] - goal[3].y)<thres )
    {

        // takeoff
        station_cmd.control_state = 6;
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