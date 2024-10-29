/*
接收地面站传入的值：TODO地面站：读取预设的6个目标点，可以选择直接使用预设的6个目标点或者修改其中任意一个or多个目标点。
每个飞机都设定完了之后，按下一键启动（选择指定n台飞机）之后6个无人机同时开始执行任务，依次飞行1-6号点，飞完后返回起飞点
*/

#include <ros/ros.h>
#include <signal.h>

#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"

#define MAX_AGENT_NUM 10 // 定义最大智能体数量
using namespace std;

int agent_type;                 // 智能体类型
int agent_id;                   // 智能体ID
int agent_num;                  // 智能体数量
float agent_height;             // 智能体高度
string node_name;               // 节点名称
sunray_msgs::orca_cmd orca_cmd; // ORCA指令
sunray_msgs::agent_cmd cmd;
sunray_msgs::agent_cmd agent_cmd[MAX_AGENT_NUM]; // 智能体控制指令
geometry_msgs::Point goal_1[MAX_AGENT_NUM];      // 1形队形目标点
geometry_msgs::Point goal_2[MAX_AGENT_NUM];      // 2形队形目标点
geometry_msgs::Point goal_3[MAX_AGENT_NUM];      // 3形队形目标点
// geometry_msgs::Point goal_4[MAX_AGENT_NUM];      // 4形队形目标点
// geometry_msgs::Point goal_5[MAX_AGENT_NUM];      // 5形队形目标点
// geometry_msgs::Point goal_6[MAX_AGENT_NUM];      // 6形队形目标点
sunray_msgs::agent_state agent_state;            // 智能体状态

// ROS相关的发布者、订阅者
ros::Publisher text_info_pub;   // 发布文字信息（如任务状态）
ros::Publisher orca_cmd_pub;    // 发布ORCA指令
ros::Publisher orca_goal_pub;   // 发布ORCA目标点
ros::Publisher agent_cmd_pub;   // 发布智能体的控制命令
ros::Subscriber start_cmd_sub;  // 订阅启动命令
ros::Subscriber agent_state_sub;// 订阅智能体的状态
ros::Subscriber agent_goal_sub; // 订阅地面站发送的目标点

// 各智能体到达状态和起始点
geometry_msgs::Point set_home[MAX_AGENT_NUM];               // 存储每个智能体的起飞点
bool updataPoint[MAX_AGENT_NUM] = {false};                  // 每个智能体的更新目标标志位
std::vector<ros::Publisher> goal_pubs;                      // 每个智能体的目标点发布者
std::vector<std::vector<geometry_msgs::Point>> agent_goals; // 存储每个智能体的目标点

// 函数声明
// 函数声明
void init_goals();                                          // 初始化目标点
void update_goal(int i);                                    // 更新指定智能体的目标点
void sendNowGoal(int id);                                   // 立即发送目标点给指定智能体
geometry_msgs::Point getNowGoal(int i);                     // 获取当前智能体的目标点
void agent_state_cb(const sunray_msgs::agent_stateConstPtr &msg); // 智能体状态的回调函数
void check_and_update_goal(int i);                          // 检查并更新智能体的目标点
void start_cmd_cb(const std_msgs::BoolConstPtr &msg);       // 启动指令的回调函数
void printf_params();                                       // 打印智能体的参数信息
void agent_goal_cb(const sunray_msgs::orca_cmdConstPtr &msg); // 处理地面站发送的目标点
void agent_cmd_land_takeoff();                              // 控制智能体的起飞和降落
void timercb_show(const ros::TimerEvent &e);                // 定时器回调函数，用于显示目标点
void mySigintHandler(int sig);                              // 信号处理函数，用于程序退出时的清理操作

// 当前目标ID
int current_goal_id;                // 当前智能体的目标点ID
bool reached_goal[MAX_AGENT_NUM];   // 标志位，表示每个智能体是否到达目标点
float thres = 0.08;                  // 距离阈值，控制是否到达目标点的判断依据
bool pub_goal_once = true;          // 标志位，控制目标点是否只发布一次
bool orca_return_home = true;       //发布return标志位


// 信号处理函数
void mySigintHandler(int sig)
{
    // 打印退出信息
    ROS_INFO("[formation_nokov] exit..."); 
    // 关闭ROS
    ros::shutdown(); 
}
// 初始化函数
void init_goals()
{
    // 打印节点名称和设置HOME命令
    cout << BLUE << node_name << " ORCA: SET_HOME" << TAIL << endl;
    // 设置ORCA命令为返回起点的命令
    orca_cmd.orca_cmd = sunray_msgs::orca_cmd::SET_HOME;
    // 发布ORCA命令
    orca_cmd_pub.publish(orca_cmd);
    // 设置延迟1秒，确保指令的顺利发布
    sleep(1.0);
    // 初始化当前目标ID为1
    current_goal_id = 1;
    // 将智能体的到达状态设置为false，表示尚未到达目标
    reached_goal[agent_id - 1] = false;
    // 发布初始目标点
    orca_goal_pub.publish(goal_1[agent_id - 1]);
    // 打印目标点信息
    cout << GREEN << "RMTT_" << agent_id << ": moving to goal 1. [" << goal_1[agent_id - 1].x << "," << goal_1[agent_id - 1].y << "]" << TAIL << endl;
    // 记录智能体的起飞点位置
    set_home[agent_id - 1].x = agent_state.pos[0];
    set_home[agent_id - 1].y = agent_state.pos[1];
}

// 智能体状态的回调函数
void agent_state_cb(const sunray_msgs::agent_stateConstPtr &msg)
{
    // 更新指定智能体的ORCA状态
    agent_state = *msg;
}

// 启动指令回调
void start_cmd_cb(const std_msgs::BoolConstPtr &msg)
{
    if(orca_return_home)
    {
        // 设置延迟0.5秒，等待指令的处理
        sleep(0.5);
        
        // 设置标志位为false，表示不再只发布一次目标点
        pub_goal_once = false;
        
        // 初始化目标点
        init_goals();
    }
}

// 打印参数函数
void printf_params()
{
    cout << GREEN << "agent_type    : " << agent_type << "" << TAIL << endl;
    cout << GREEN << "agent_num     : " << agent_num << "" << TAIL << endl;
    cout << GREEN << "agent_height   : " << agent_height << "" << TAIL << endl;
}

// 主函数
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "neu_display");
    // 创建节点句柄
    ros::NodeHandle nh("~");
    // 设置循环频率为20Hz
    ros::Rate rate(20.0);
    // 获取节点名称
    node_name = ros::this_node::getName();

    // 【参数】智能体类型 获取智能体类型参数
    nh.param<int>("agent_type", agent_type, 0);
    // 【参数】智能体编号 获取智能体数量参数
    nh.param<int>("agent_num", agent_num, 6);
    // 【参数】agent_height 获取智能体高度参数
    nh.param<float>("agent_height", agent_height, 1.0f);

    nh.param<int>("agent_id", agent_id, 1);

    // 为每个智能体分配目标点数组
    agent_goals.resize(agent_num);
    // 为每个智能体创建一个目标点发布者
    goal_pubs.resize(agent_num);

    // 打印获取的智能体参数信息
    // printf_params();
    // 定义存储智能体名称的字符串变量
    string agent_name;
    // 定义存储智能体前缀的字符串变量
    string agent_prefix;

    // 智能体类型设置前缀
    agent_prefix = "rmtt_";
    agent_name = "/" + agent_prefix + std::to_string(agent_id);

    // 【发布】ORCA指令 本节点 ->  ORCA
    orca_cmd_pub = nh.advertise<sunray_msgs::orca_cmd>("/sunray_swarm/" + agent_prefix + "/orca_cmd", 1);
    // 【发布】文字提示消息（回传至地面站显示）
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);
    // 【发布】无人机的目标点
    orca_goal_pub = nh.advertise<geometry_msgs::Point>("/sunray_swarm" + agent_name + "/goal_point", 1);
    // 【发布】起飞和降落命令
    agent_cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 1);
    // 【订阅】程序触发指令
    start_cmd_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm" + agent_name + "/neu_display_goals", 1, start_cmd_cb);
    // 【订阅】订阅地面站目标点
    agent_goal_sub = nh.subscribe<sunray_msgs::orca_cmd>("/sunray_swarm" + agent_name + "/all_goal_point", 1, boost::bind(&agent_goal_cb, _1));
    // 【订阅】无人机orca状态 agent_state_sub
    agent_state_sub = nh.subscribe<sunray_msgs::agent_state>("/sunray_swarm" + agent_name + "/agent_state", 1, boost::bind(&agent_state_cb, _1));
    
    // 创建定时器
    ros::Timer timer_show = nh.createTimer(ros::Duration(1.0), timercb_show);

    // 主循环
    while (ros::ok())
    {
        // 每次循环都处理一次回调函数
        ros::spinOnce();
        // 触发回调
        if (!pub_goal_once)
        {
            check_and_update_goal(agent_id - 1);
            orca_return_home = false;
        }
        rate.sleep();
    }
    return 0;
}

// 定时器回调（用于显示目标点）
void timercb_show(const ros::TimerEvent &e)
{
    // 如果需要更新目标点，则调用sendNowGoal函数
    if (updataPoint[agent_id - 1])
        sendNowGoal(agent_id - 1);
}

// 触发起飞降落
void agent_cmd_land_takeoff()
{
    // 延迟2秒
    sleep(2.0);
    // 设置智能体命令为降落
    sunray_msgs::agent_cmd cmd;
    cmd.agent_id = agent_id;
    // 发布降落命令
    cmd.control_state = sunray_msgs::agent_cmd::LAND;
    agent_cmd_pub.publish(cmd);
    // 如果当前目标ID为7，表示完成所有任务，返回起飞点
    if (current_goal_id == 4)
    {
        updataPoint[agent_id - 1] = false;
        return;
    }
    // 延迟5秒，等待降落完成
    sleep(5.0);
    // 设置智能体命令为起飞
    cmd.agent_id = agent_id;
    cmd.control_state = sunray_msgs::agent_cmd::TAKEOFF;
    // 发布起飞命令
    agent_cmd_pub.publish(cmd);
    // 延迟5秒，等待起飞完成
    sleep(7.0);
}

// 订阅地面站回调TODO
/*读取地面站发送预设的6个无人机的6个目标点
*/
void agent_goal_cb(const sunray_msgs::orca_cmdConstPtr &msg)
{
    // 检查接收到的目标点是否少于6个，若不足，则退出回调
    if (msg->goal_point.size() < 3)
    {
        return;
    }
    // 更新每个智能体的6个目标点
    goal_1[agent_id - 1] = msg->goal_point[0];
    goal_2[agent_id - 1] = msg->goal_point[1];
    goal_3[agent_id - 1] = msg->goal_point[2];
    // goal_4[agent_id - 1] = msg->goal_point[3];
    // goal_5[agent_id - 1] = msg->goal_point[4];
    // goal_6[agent_id - 1] = msg->goal_point[5];
}

// 更新智能体目标点的函数
void update_goal(int i)
{
    // 根据当前目标ID更新智能体的下一个目标点
    switch (current_goal_id)
    {
    case 1:
        orca_goal_pub.publish(goal_2[i]); // 发布2形队形目标点
        break;
    case 2:
        orca_goal_pub.publish(goal_3[i]); // 发布3形队形目标点
        break;
    // case 3:
        //设置当输入点为99时跳过目标点
        // if(goal_4[i].x = 99)
        // {
        //      ;
        //     orca_goal_pub.publish(set_home[i]); 
        // }
    //     orca_goal_pub.publish(goal_4[i]); // 发布4形队形目标点
    //     break;
    // case 4:
    //     orca_goal_pub.publish(goal_5[i]); // 发布5形队形目标点
    //     break;
    // case 5:
    //     orca_goal_pub.publish(goal_6[i]); // 发布6形队形目标点
    //     break;
    case 3:
        // orca_cmd.orca_cmd = sunray_msgs::orca_cmd::RETURN_HOME; // 设置ORCA命令为返回起点
        orca_goal_pub.publish(set_home[i]);
        // orca_cmd_pub.publish(orca_cmd);                         // 发布命令
        break;
    default:
        // 所有目标点完成，发布降落命令
        pub_goal_once = true;
        cmd.control_state = sunray_msgs::agent_cmd::LAND; // 降落命令
        agent_cmd_pub.publish(cmd);
        sleep(1.0);
        break;
    }
}
// 发送当前目标点给指定的智能体
void sendNowGoal(int id)
{
    // 根据当前目标ID发布对应的目标点
    switch (current_goal_id)
    {
    case 1:
        orca_goal_pub.publish(goal_1[id]); // 发布1形队形目标点
        cout << BLUE << "RMTT_" << id + 1 << ": Time moving to goal 1 at [" << goal_1[id].x << "," << goal_1[id].y << "]" << TAIL << endl;
        // cout << BLUE << node_name << ": ORCA add agents_" << i+1 << " at [" << agent_state[i].pos[0] << "," << agent_state[i].pos[1] << "]"<< TAIL << endl;
        break;

    case 2:
        orca_goal_pub.publish(goal_2[id]); // 发布2形队形目标点
        cout << BLUE << "RMTT_" << id + 1 << ": Time moving to goal 2 at [" << goal_2[id].x << "," << goal_2[id].y << "]" << TAIL << endl;
        break;
    case 3:
        orca_goal_pub.publish(goal_3[id]); // 发布3形队形目标点
        cout << BLUE << "RMTT_" << id + 1 << ": Time moving to goal 3 at [" << goal_3[id].x << "," << goal_3[id].y << "]" << TAIL << endl;

        break;
    // case 4:
    //     orca_goal_pub.publish(goal_4[id]); // 发布4形队形目标点
    //     cout << BLUE << "RMTT_" << id + 1 << ": Time moving to goal 4 at [" << goal_4[id].x << "," << goal_4[id].y << "]" << TAIL << endl;
    //     break;
    // case 5:
    //     orca_goal_pub.publish(goal_5[id]); // 发布5形队形目标点
    //     cout << BLUE << "RMTT_" << id + 1 << ": Time moving to goal 5 at [" << goal_5[id].x << "," << goal_5[id].y << "]" << TAIL << endl;
    //     break;
    // case 6:
    //     orca_goal_pub.publish(goal_6[id]); // 发布6形队形目标点
    //     cout << BLUE << "RMTT_" << id + 1 << ": Time moving to goal 6 at [" << goal_6[id].x << "," << goal_6[id].y << "]" << TAIL << endl;
    //     break;

    case 4:
        // 所有目标点完成，返回起飞点
        cout << BLUE << "RMTT_" << id + 1 << ": reached final goal" << TAIL << endl;
        orca_goal_pub.publish(set_home[id]);
        break;
    default:
        pub_goal_once = true;
        orca_return_home = true;
        cmd.control_state = sunray_msgs::agent_cmd::LAND; // 降落命令
        agent_cmd_pub.publish(cmd);
        sleep(1.0);
        break;
    }
}

// 获取当前智能体的目标点
geometry_msgs::Point getNowGoal(int i)
{
    // 根据当前目标ID返回对应的目标点
    geometry_msgs::Point temp;
    switch (current_goal_id)
    {
    case 1:
        return goal_1[i];
        break;
    case 2:
        return goal_2[i];
        break;
    case 3:
        return goal_3[i];
        break;
    case 4:
        return set_home[i];
        break;
    // case 4:
    //     return goal_4[i];
    //     break;
    // case 5:
    //     return goal_5[i];
    //     break;
    // case 6:
    //     return goal_6[i];
    //     break;
    // case 7:
    //     return set_home[i];
    //     break;
    default:
        break;
    }

    temp.x = 0;
    temp.y = 0;
    return temp;
}

// 检查并更新目标点函数takeoff_and_land_sub
void check_and_update_goal(int i)
{
    // 设置更新标志位为true
    updataPoint[i] = true;
    // 计算当前智能体与目标点的距离
    float dist_x = abs(agent_state.pos[0] - getNowGoal(i).x);
    float dist_y = abs(agent_state.pos[1] - getNowGoal(i).y);

    if (dist_x < thres && dist_y < thres && !reached_goal[i])
    {
        // 关闭更新标志位
        updataPoint[i] = false;
        // 设置到达状态为true
        reached_goal[i] = true;
        // 调用起飞和降落的控制函数
        agent_cmd_land_takeoff();
        // 更新下一个目标点
        update_goal(i); 
        // 目标ID递增
        current_goal_id++; 
        // 打开更新标志位
        updataPoint[i] = true;
    }
    else if (reached_goal[i]) // 如果无人机已到达目标，检查是否需要重置状态
    {
        // 重置到达状态以便可以继续下一个目标
        reached_goal[i] = false;
    }
}