#include <ros/ros.h>
#include <signal.h>

#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"

#define MAX_AGENT_NUM 10                                // 定义最大智能体数量
using namespace std;

int agent_type;                                         // 智能体类型
int agent_num;                                          // 智能体数量
float agent_height;                                     // 智能体高度
string node_name;                                       // 节点名称
sunray_msgs::orca_cmd orca_cmd;                         // ORCA指令
sunray_msgs::agent_cmd agent_cmd[MAX_AGENT_NUM];        // 智能体控制指令
sunray_msgs::orca_state orca_state[MAX_AGENT_NUM];      // ORCA状态
geometry_msgs::Point goal_C[MAX_AGENT_NUM];             // N形队形目标点
geometry_msgs::Point goal_X[MAX_AGENT_NUM];             // O形队形目标点
geometry_msgs::Point goal_Y[MAX_AGENT_NUM];             // L形队形目标点
geometry_msgs::Point goal_V[MAX_AGENT_NUM];             // V形队形目标点

// 执行状态
enum FORMATION_STATE
{
    INIT = 0,               // 初始模式
    C = 1,                  // N形队形
    X = 2,                  // O形队形
    Y = 3,                  // K形队形
    O2 = 4,                 // O形队形（第二次）
    V = 5,                  // V形队形
    RETURN_HOME = 6,        // 返回起点
};
FORMATION_STATE formation_state;        // 当前队形状态

ros::Publisher text_info_pub;                       // 发布文字提示消息
ros::Publisher orca_cmd_pub;                        // 发布ORCA指令
ros::Subscriber start_cmd_sub;                      // 订阅启动指令
ros::Subscriber orca_state_sub[MAX_AGENT_NUM];      // 订阅ORCA状态
ros::Publisher orca_goal_pub[MAX_AGENT_NUM];        // 发布ORCA目标点

// 信号处理函数
void mySigintHandler(int sig)
{
    ROS_INFO("[formation_nokov] exit...");          // 打印退出信息
    ros::shutdown();                                // 关闭ROS
}
// 处理ORCA状态回调
void rmtt_orca_state_cb(const sunray_msgs::orca_stateConstPtr& msg, int i)
{
    // 更新指定智能体的ORCA状态
    orca_state[i] = *msg;
}
// 定时器回调（用于显示目标点）
void timercb_show(const ros::TimerEvent &e)
{
    
}
// 启动指令回调
void start_cmd_cb(const std_msgs::BoolConstPtr& msg)
{
    formation_state = FORMATION_STATE::C;// 设置队形状态为N
}
// 函数声明，用于设置目标点
void setup_show_goals();
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
    ros::init(argc, argv, "formation_nokov");
    // 创建节点句柄
    ros::NodeHandle nh("~");
    // 设置循环频率为20Hz
    ros::Rate rate(20.0);
    // 获取节点名称
    node_name = ros::this_node::getName();

    // 【参数】智能体类型 获取智能体类型参数
    nh.param<int>("agent_type", agent_type, 0);
    // 【参数】智能体编号 获取智能体数量参数
    nh.param<int>("agent_num", agent_num, 8);
    // 【参数】agent_height 获取智能体高度参数
    nh.param<float>("agent_height", agent_height, 0.0f);
    // 定义队形持续时间
    float formation_time = 5.0;

    printf_params();                             // 打印参数信息
    string agent_name;                           // 存储智能体名称
    string agent_prefix;                         // 存储智能体前缀

    // 根据智能体类型设置前缀
    if(agent_type == sunray_msgs::agent_state::RMTT)
    {
        agent_prefix = "rmtt_";
    }else if(agent_type == sunray_msgs::agent_state::UGV)
    {
        agent_prefix = "ugv_";
    }else if(agent_type == sunray_msgs::agent_state::SIKONG)
    {
        agent_prefix = "sikong_";
    }else
    {
        agent_prefix = "unkonown_";
    }

    // 【发布】ORCA指令 本节点 ->  ORCA
    orca_cmd_pub = nh.advertise<sunray_msgs::orca_cmd>("/sunray_swarm/" + agent_prefix + "/orca_cmd", 1);
    // 【发布】文字提示消息（回传至地面站显示）
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);
    // 【订阅】程序触发指令
    start_cmd_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm/demo/swarm_formation_nokov", 1, start_cmd_cb);

    // 为每个智能体创建发布和订阅
    for(int i = 0; i < agent_num; i++) 
    {
        agent_name = "/" + agent_prefix + std::to_string(i+1);
        // 【发布】无人机的目标点
        orca_goal_pub[i] = nh.advertise<geometry_msgs::Point>("/sunray_swarm" + agent_name + "/goal_point", 1);
        // 【订阅】无人机orca状态
		orca_state_sub[i] = nh.subscribe<sunray_msgs::orca_state>("/sunray_swarm" + agent_name + "/agent_orca_state", 1, boost::bind(&rmtt_orca_state_cb,_1,i));
    }
    // 创建定时器
    ros::Timer timer_show = nh.createTimer(ros::Duration(3.0), timercb_show);
    // 设置目标点
    setup_show_goals();
    // 初始化队形状态
    formation_state = FORMATION_STATE::INIT;
    // 发布目标点的标志
    bool pub_goal_once = false;

    // 主循环
    while (ros::ok())
    {
        // 回调函数
        ros::spinOnce();

        switch(formation_state)
        {
            case FORMATION_STATE::INIT:
            // 初始状态逻辑
            break;

            case FORMATION_STATE::C:
                // N形队形
                if(!pub_goal_once)
                {
                    cout << BLUE << node_name << " ORCA: SET_HOME" << TAIL << endl;
                    // 设置ORCA命令为HOME
                    orca_cmd.orca_cmd = sunray_msgs::orca_cmd::SET_HOME;
                    // 发布命令
                    orca_cmd_pub.publish(orca_cmd);
                    // 设置延迟
                    sleep(0.5);

                    cout << BLUE << node_name << " Formation: C" << TAIL << endl;
                    for(int i = 0; i < agent_num; i++) 
                    {
                        orca_goal_pub[i].publish(goal_C[i]);    // 发布N形队形目标点
                        sleep(0.1);                             // 延迟
                    }
                    pub_goal_once = true;                       // 标志设置为已发布
                    orca_state[0].arrived_all_goal = false;     // 重置到达目标状态
                    sleep(1.0);                                 // 延迟
                }
                // 检查所有智能体是否到达目标
                if(pub_goal_once && orca_state[0].arrived_all_goal)
                {
                    sleep(formation_time);                      // 等待队形时间
                    formation_state = FORMATION_STATE::X;       // 转换到O形队形
                    pub_goal_once = false;                      // 重置标志
                }
                break;
            case FORMATION_STATE::X:
                // O形队形
                if(!pub_goal_once)
                {
                    cout << BLUE << node_name << " Formation: X" << TAIL << endl;
                    for(int i = 0; i < agent_num; i++) 
                    {
                        orca_goal_pub[i].publish(goal_X[i]);    // 发布O形队形目标点
                        sleep(0.1);                             // 延迟
                    }
                    pub_goal_once = true;                       // 标志设置为已发布
                    orca_state[0].arrived_all_goal = false;     // 重置到达目标状态
                    sleep(1.0);                                 // 延迟
                }
                // 检查所有智能体是否到达目标
                if(pub_goal_once && orca_state[0].arrived_all_goal)
                {
                    sleep(formation_time);                      // 等待队形时间
                    formation_state = FORMATION_STATE::Y;       // 转换到K形队形
                    pub_goal_once = false;                      // 重置标志
                }
                break;
            case FORMATION_STATE::Y:
                // K形队形
                if(!pub_goal_once)
                {
                    cout << BLUE << node_name << " Formation: Y" << TAIL << endl;
                    for(int i = 0; i < agent_num; i++) 
                    {
                        orca_goal_pub[i].publish(goal_Y[i]);    // 发布K形队形目标点
                        sleep(0.1);                             // 延迟
                    }
                    pub_goal_once = true;                       // 标志设置为已发布
                    orca_state[0].arrived_all_goal = false;     // 重置到达目标状态
                    sleep(1.0);                                 // 延迟
                }
                // 检查所有智能体是否到达目标
                if(pub_goal_once && orca_state[0].arrived_all_goal)
                {
                    sleep(formation_time);                      // 等待队形时间
                    formation_state = FORMATION_STATE::RETURN_HOME;      // 转换到O2形队形
                    pub_goal_once = false;                      // 重置标志
                }
                break;
            // case FORMATION_STATE::O2:
            //     // O形队形
            //     if(!pub_goal_once)
            //     {
            //         cout << BLUE << node_name << " Formation: O" << TAIL << endl;
            //         for(int i = 0; i < agent_num; i++) 
            //         {
            //             orca_goal_pub[i].publish(goal_O[i]);    // 发布O2形队形目标点
            //             sleep(0.1);                             // 延迟
            //         }
            //         pub_goal_once = true;                       // 标志设置为已发布
            //         orca_state[0].arrived_all_goal = false;     // 重置到达目标状态
            //         sleep(1.0);                                 // 延迟
            //     }
            //     // 检查所有智能体是否到达目标
            //     if(pub_goal_once && orca_state[0].arrived_all_goal)
            //     {
            //         sleep(formation_time);                      // 等待队形时间
            //         formation_state = FORMATION_STATE::V;       // 转换到V形队形
            //         pub_goal_once = false;                      // 重置标志
            //     }
            //     break;
            // case FORMATION_STATE::V:
            //     // V形队形
            //     if(!pub_goal_once)
            //     {
            //         cout << BLUE << node_name << " Formation: V" << TAIL << endl;
            //         for(int i = 0; i < agent_num; i++) 
            //         {
            //             orca_goal_pub[i].publish(goal_V[i]);    // 发布V形队形目标点
            //             sleep(0.1);                             // 延迟
            //         }
            //         pub_goal_once = true;                       // 标志设置为已发布
            //         orca_state[0].arrived_all_goal = false;     // 重置到达目标状态
            //         sleep(1.0);                                 // 延迟
            //     }   
            //     // 检查所有智能体是否到达目标
            //     if(pub_goal_once && orca_state[0].arrived_all_goal)
            //     {
            //         sleep(formation_time);                      // 等待队形时间
            //         formation_state = FORMATION_STATE::RETURN_HOME; // 转换到返回起点状态
            //         pub_goal_once = false;                      // 重置标志
            //     }
            //     break;
            case FORMATION_STATE::RETURN_HOME:
                // Return
                if(!pub_goal_once)
                {
                    cout << BLUE << node_name << " ORCA: RETURN_HOME" << TAIL << endl;

                    orca_cmd.orca_cmd = sunray_msgs::orca_cmd::RETURN_HOME; // 设置ORCA命令为返回起点
                    orca_cmd_pub.publish(orca_cmd);                         // 发布命令
                    pub_goal_once = true;                                   // 标志设置为已发布
                    orca_state[0].arrived_all_goal = false;                 // 重置到达目标状态
                    sleep(1.0);                                             // 延迟
                }
                // 检查所有智能体是否到达目标
                if(pub_goal_once && orca_state[0].arrived_all_goal)
                {
                    formation_state = FORMATION_STATE::INIT;
                    pub_goal_once = false;                      // 重置标志
                }
                break;
        }

        // sleep
        rate.sleep();
    }

    return 0;

}

void setup_show_goals()
{
    // 字母 N
    goal_C[0].x = 0.86; goal_C[0].y = -0.35;  // C的顶部左端
goal_C[1].x = 1.1; goal_C[1].y = 0.55;   // C的弯曲上端
goal_C[2].x = 0; goal_C[2].y = 1.2;   // C的中部转折点
goal_C[3].x = -1.1; goal_C[3].y = 0.55;  // C的弯曲下端
goal_C[4].x = -0.86; goal_C[4].y = -0.35; // C的底部左端
    goal_C[5].x = 3.0; goal_C[5].y = 4.15;   // 中部交点
    goal_C[6].x = 3.2; goal_C[6].y = -4.3;  // 顶部右端
    goal_C[7].x = -2.0; goal_C[7].y = -4.3; // 底部右端

    // 字母 O
    goal_X[0].x = 1.2; goal_X[0].y = -0.9;   // X的左上端
goal_X[1].x = 1.2; goal_X[1].y = 0.9;  // X的右下端
goal_X[2].x = 0.0; goal_X[2].y = 0.2;   // X的中心交点
goal_X[3].x = -1.2; goal_X[3].y = 0.9;  // X的左下端
goal_X[4].x = -1.2; goal_X[4].y = -0.9;   // X的右上端
    goal_X[5].x = 3.0; goal_X[5].y = 4.15;   // 中部交点
    goal_X[6].x = 3.2; goal_X[6].y = -4.3;  // 顶部右端
    goal_X[7].x = -2.0; goal_X[7].y = -4.3; // 底部右端
    // 字母 K
    goal_Y[0].x = 1.3; goal_Y[0].y = -0.8;   // Y的顶部交点
goal_Y[1].x = 1.3; goal_Y[1].y = 0.8;   // Y的右上端
goal_Y[2].x = 0.3; goal_Y[2].y = 0;  // Y的左上端
goal_Y[3].x = -0.6; goal_Y[3].y = 0;  // Y的底部交点
goal_Y[4].x = -1.4; goal_Y[4].y = 0;  // Y的右下端
    goal_Y[5].x = 3.0; goal_Y[5].y = 4.15;   // 中部交点
    goal_Y[6].x = 3.2; goal_Y[6].y = -4.3;  // 顶部右端
    goal_Y[7].x = -2.0; goal_Y[7].y = -4.3; // 底部右端
    // 字母 V
    goal_V[0].x = 0.1;  goal_V[0].y = 0.6;  // 左上点
    goal_V[1].x = 1.1;  goal_V[1].y = 1.1; // 右上点
    goal_V[2].x = -0.7;  goal_V[2].y = 0.0;  // 底部点
    goal_V[3].x = 1.1;  goal_V[3].y = -1.1;  // 左下点
    goal_V[4].x = 0.1;  goal_V[4].y = -0.6; // 右下点
    goal_V[5].x = 3.0; goal_V[5].y = 4.15;   // 中部交点
    goal_V[6].x = 3.2; goal_V[6].y = -4.3;  // 顶部右端
    goal_V[7].x = -2.0; goal_V[7].y = -4.3; // 底部右端
}
