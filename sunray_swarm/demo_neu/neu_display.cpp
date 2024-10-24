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

int agent_type;                                  // 智能体类型
int agent_id;                                    // 智能体类型
int agent_num;                                   // 智能体数量
float agent_height;                              // 智能体高度
string node_name;                                // 节点名称
sunray_msgs::orca_cmd orca_cmd;                  // ORCA指令
sunray_msgs::agent_cmd agent_cmd[MAX_AGENT_NUM]; // 智能体控制指令
geometry_msgs::Point goal_1[MAX_AGENT_NUM];      // 1形队形目标点
geometry_msgs::Point goal_2[MAX_AGENT_NUM];      // 2形队形目标点
geometry_msgs::Point goal_3[MAX_AGENT_NUM];      // 3形队形目标点
geometry_msgs::Point goal_4[MAX_AGENT_NUM];      // 4形队形目标点
geometry_msgs::Point goal_5[MAX_AGENT_NUM];      // 5形队形目标点
geometry_msgs::Point goal_6[MAX_AGENT_NUM];      // 6形队形目标点
sunray_msgs::agent_state agent_state;
sunray_msgs::agent_cmd cmd;

float thres = 0.1; // 定义距离阈值
bool pub_goal_once = true;

// 执行状态
enum FORMATION_STATE
{
    INIT = 0,        // 初始模式
    GOAL_1 = 1,      // 1形队形
    GOAL_2 = 2,      // 2形队形
    GOAL_3 = 3,      // 3形队形
    GOAL_4 = 4,      // 4形队形
    GOAL_5 = 5,      // 5形队形
    RETURN_HOME = 6, // 返回起点
    TAKEOFF = 11,
    LAND = 12,
};
FORMATION_STATE formation_state; // 当前队形状态

ros::Publisher text_info_pub;                     // 发布文字提示消息
ros::Publisher orca_cmd_pub;                      // 发布ORCA指令
ros::Subscriber start_cmd_sub;                    // 订阅启动指令
ros::Subscriber agent_state_sub;                  // 订阅ORCA状态
ros::Publisher orca_goal_pub;                     // 发布ORCA目标点
ros::Subscriber agent_goal_sub;                   // 订阅地面站目标点
ros::Publisher agent_cmd_pub;                     // 新增发布起飞和降落命令的发布者
bool agent_reached_goal[MAX_AGENT_NUM] = {false}; // 每个无人机的到达状态
geometry_msgs::Point set_home[MAX_AGENT_NUM];

ros::Publisher takeoff_pub;
ros::Publisher land_pub;
std_msgs::Empty takeoff;
std_msgs::Empty land;
bool updataPoint[MAX_AGENT_NUM]={false};
// std::vector<std::vector<geometry_msgs::Point>> agent_goals(MAX_AGENT_NUM);
ros::Subscriber goals_sub[MAX_AGENT_NUM];
// std::vector<int> current_goal_index(MAX_AGENT_NUM, 0); // 目前的目标索引
// 全局发布器
// std::vector<ros::Publisher> goal_pubs(MAX_AGENT_NUM);

std::vector<ros::Publisher> goal_pubs; // 每架无人机一个目标点发布器
std::vector<int> current_goal_index;   // 每架无人机当前目标点索引
std::vector<std::vector<geometry_msgs::Point>> agent_goals(MAX_AGENT_NUM);
// 起飞和降落的控制函数
void send_takeoff_and_land_cmd(int agent_id, bool is_takeoff);
void publish_next_goal(int agent_id);
ros::Subscriber takeoff_and_land_sub; // 订阅地面站起飞降落
void agent_cmd_land_takeoff();

void timercb_show(const ros::TimerEvent &e);

// 信号处理函数
void mySigintHandler(int sig)
{
    ROS_INFO("[formation_nokov] exit..."); // 打印退出信息
    ros::shutdown();                       // 关闭ROS
}

int current_goal_id;              // 每个无人机的当前目标ID
bool reached_goal[MAX_AGENT_NUM]; // 每个无人机的到达状态标志
void check_and_update_goal(int i);

void update_goal(int i);
// 初始化函数
void init_goals()
{
    
    cout << BLUE << node_name << " ORCA: SET_HOME" << TAIL << endl;
    // 设置ORCA命令为HOME
    orca_cmd.orca_cmd = sunray_msgs::orca_cmd::SET_HOME;
    // 发布命令
    orca_cmd_pub.publish(orca_cmd);
    // 设置延迟
    sleep(1.0);
    // 初始化无人机的目标点数据和状态
    current_goal_id = 1;                // 所有无人机初始目标ID设为1
    reached_goal[agent_id - 1] = false; // 到达状态设为false

    // 发布初始目标点
    orca_goal_pub.publish(goal_1[agent_id - 1]);
    cout << "UAV " << agent_id << " moving to goal 1" << " goal_1[i].x " << goal_1[agent_id - 1].x << endl;
    set_home[agent_id - 1].x = agent_state.pos[0];
    set_home[agent_id - 1].y = agent_state.pos[1];

}
// 更新目标点函数
void update_goal(int i)
{
    switch (current_goal_id)
    {
    case 1:
        orca_goal_pub.publish(goal_2[i]); // 发布N形队形目标点
        cout << BLUE << "UAV " << i + 1 << " moving to goal 2 " << goal_2[i] << " " << TAIL << endl;
        break;
    case 2:
        orca_goal_pub.publish(goal_3[i]); // 发布O形队形目标点
        cout << BLUE << "UAV " << i + 1 << " moving to goal 3 " << goal_3[i] << " " << TAIL << endl;

        break;
    case 3:
        orca_goal_pub.publish(goal_4[i]); // 发布K形队形目标点
        cout << BLUE << "UAV " << i + 1 << " moving to goal 4 " << goal_4[i] << " " << TAIL << endl;
        break;
    case 4:
        orca_goal_pub.publish(goal_5[i]); // 发布K形队形目标点

        cout << BLUE << "UAV " << i + 1 << " moving to goal 5 " << goal_5[i] << " " << TAIL << endl;
        break;
    case 5:
        orca_goal_pub.publish(goal_6[i]); // 发布K形队形目标点
        cout << BLUE << "UAV " << i + 1 << " moving to goal 6 " << goal_5[i] << " " << TAIL << endl;
        break;

    case 6:
        cout << RED << "UAV " << i + 1 << " reached final goal" << TAIL << endl;
        // orca_cmd.orca_cmd = sunray_msgs::orca_cmd::RETURN_HOME; // 设置ORCA命令为返回起点
        orca_goal_pub.publish(set_home[i]); 
        // orca_cmd_pub.publish(orca_cmd);                         // 发布命令
        break;
    default:
        pub_goal_once = true;
        cmd.control_state = sunray_msgs::agent_cmd::LAND; // 降落命令
        agent_cmd_pub.publish(cmd);
        sleep(1.0);
        break;
    }
}


void sendNowGoal(int id)
{
    switch (current_goal_id)
    {
    case 1:
        orca_goal_pub.publish(goal_1[id]); // 发布N形队形目标点
        cout << BLUE << "UAV  " << id + 1 << " Time moving to goal 1 " << goal_1[id] << " " << TAIL << endl;
        break;        

    case 2:
        orca_goal_pub.publish(goal_2[id]); // 发布N形队形目标点
        cout << BLUE << "UAV " << id + 1 << " Time moving to goal 2 " << goal_2[id] << " " << TAIL << endl;
        break;
    case 3:
        orca_goal_pub.publish(goal_3[id]); // 发布O形队形目标点
        cout << BLUE << "UAV " << id + 1 << " Time moving to goal 3 " << goal_3[id] << " " << TAIL << endl;

        break;
    case 4:
        orca_goal_pub.publish(goal_4[id]); // 发布K形队形目标点
        cout << BLUE << "UAV " << id + 1 << " Time moving to goal 4 " << goal_4[id] << " " << TAIL << endl;
        break;
    case 5:
        orca_goal_pub.publish(goal_5[id]); // 发布K形队形目标点

        cout << BLUE << "UAV " << id + 1 << " Time moving to goal 5 " << goal_5[id] << " " << TAIL << endl;
        break;
    case 6:
        orca_goal_pub.publish(goal_6[id]); // 发布K形队形目标点
        cout << BLUE << "UAV " << id + 1 << " Time moving to goal 6 " << goal_5[id] << " " << TAIL << endl;
        break;

    case 7:
        cout << RED << "UAV " << id + 1 << " Time reached final goal" << TAIL << endl;
        orca_goal_pub.publish(set_home[id]); 
        break;
    default:
        pub_goal_once = true;
        cmd.control_state = sunray_msgs::agent_cmd::LAND; // 降落命令
        agent_cmd_pub.publish(cmd);
        sleep(1.0);
        break;
    }
}


geometry_msgs::Point getNowGoal(int i)
{
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
        return goal_4[i];
        break;
    case 5:
        return goal_5[i];
        break;
    case 6:
        return goal_6[i];
        break;
    case 7:
        return set_home[i];
        break;
    default:
        break;
    }

    temp.x = 0;
    temp.y = 0;
    return temp;
}

void agent_state_cb(const sunray_msgs::agent_stateConstPtr &msg)
{
    // 更新指定智能体的ORCA状态
    agent_state = *msg;
}

// 检查并更新目标点函数takeoff_and_land_sub
void check_and_update_goal(int i)
{
    updataPoint[i]=true;
    float dist_x = abs(agent_state.pos[0] - getNowGoal(i).x);
    float dist_y = abs(agent_state.pos[1] - getNowGoal(i).y);

    if (dist_x < thres && dist_y < thres && !reached_goal[i])
    {
        // 如果到达目标点，更新目标ID，并设置到达状态为true
        updataPoint[i]=false;

        reached_goal[i] = true;
        agent_cmd_land_takeoff();
        update_goal(i); // 更新下一个目标点

        current_goal_id++; // 目标ID递增
        updataPoint[i]=true;

    }
    else if (reached_goal[i]) // 如果无人机已到达目标，检查是否需要重置状态
    {
        // 重置到达状态以便可以继续下一个目标
        reached_goal[i] = false;
    }

}


// 函数声明
void setup_show_goals();
// 启动指令回调
void start_cmd_cb(const std_msgs::BoolConstPtr &msg)
{
    // // init_goals();
    // cout << BLUE << node_name << " ORCA: SET_HOME" << TAIL << endl;
    // // 设置ORCA命令为HOME
    // orca_cmd.orca_cmd = sunray_msgs::orca_cmd::SET_HOME;
    // // 发布命令
    // orca_cmd_pub.publish(orca_cmd);
    // 设置延迟
    sleep(0.5);
    pub_goal_once = false; // 设置队形状态为goal_1
    init_goals();
}

// 打印参数函数
void printf_params()
{
    cout << GREEN << "agent_type    : " << agent_type << "" << TAIL << endl;
    cout << GREEN << "agent_num     : " << agent_num << "" << TAIL << endl;
    cout << GREEN << "agent_height   : " << agent_height << "" << TAIL << endl;
}

void send_takeoff_and_land_cmd(int agent_id, bool is_takeoff)
{
    // cmd.agent_id = agent_id - 1;

    // cmd.control_state = sunray_msgs::agent_cmd::HOLD; // 降落命令
    // agent_cmd_pub.publish(cmd);
    // sleep(1.0);

    // cmd.control_state = sunray_msgs::agent_cmd::TAKEOFF; // 降落命令
    // agent_cmd_pub.publish(cmd);
    // cout << BLUE << node_name << " takeoff" << TAIL << endl;
    // sleep(5.0);

    // cmd.control_state = sunray_msgs::agent_cmd::LAND; // 降落命令
    // agent_cmd_pub.publish(cmd);
    // cout << BLUE << node_name << " land" << TAIL << endl;
    // sleep(5.0);

    // cmd.control_state = sunray_msgs::agent_cmd::HOLD;
    // ; // 降落命令
    // agent_cmd_pub.publish(cmd);
    // sleep(1.0);
    cmd.agent_id = agent_id;
    cmd.control_state = sunray_msgs::agent_cmd::LAND;
    agent_cmd_pub.publish(cmd);
    ROS_INFO("Agent %d land triggered.", agent_id);
    sleep(5.0);

    sunray_msgs::agent_cmd cmd;
    cmd.agent_id = agent_id;
    cmd.control_state = sunray_msgs::agent_cmd::TAKEOFF;
    agent_cmd_pub.publish(cmd);
    ROS_INFO("Agent %d takeoff triggered.", agent_id);
    sleep(5.0);

    
}

// 订阅地面站回调TODO
/*读取地面站发送预设的6个无人机的6个目标点

*/
void agent_goal_cb(const sunray_msgs::orca_cmdConstPtr &msg)
{

    cout << "agent_goal_cb  " << agent_id << "    " << msg->goal_point.size() << endl;
    geometry_msgs::Point Point1, Point2, Point3, Point4;

    if (msg->goal_point.size() < 6)
    {

        return;
    }

    goal_1[agent_id - 1] = msg->goal_point[0];
    goal_2[agent_id - 1] = msg->goal_point[1];
    goal_3[agent_id - 1] = msg->goal_point[2];
    goal_4[agent_id - 1] = msg->goal_point[3];
    goal_5[agent_id - 1] = msg->goal_point[4];
    goal_6[agent_id - 1] = msg->goal_point[5];
}

// TODO：触发起飞降落，true是起飞，false是降落
void agent_cmd_land_takeoff()
{
    sleep(2.0);
    sunray_msgs::agent_cmd cmd;
    cmd.agent_id = agent_id ;
    cmd.control_state = sunray_msgs::agent_cmd::LAND;
    agent_cmd_pub.publish(cmd);
    ROS_INFO("Agent %d land triggered.", agent_id);
    if(current_goal_id == 7)
    {
        updataPoint[agent_id-1]=false;
        return;
    }
    sleep(7.0);

    cmd.agent_id = agent_id;
    cmd.control_state = sunray_msgs::agent_cmd::TAKEOFF;
    agent_cmd_pub.publish(cmd);
    ROS_INFO("Agent %d take triggered.", agent_id);
    sleep(7.0);
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

    agent_goals.resize(agent_num);
    goal_pubs.resize(agent_num);

    // 定义队形持续时间
    float formation_time = 5.0;
    printf_params();     // 打印参数信息
    string agent_name;   // 存储智能体名称
    string agent_prefix; // 存储智能体前缀

    // 根据智能体类型设置前缀
    if (agent_type == sunray_msgs::agent_state::RMTT)
    {
        agent_prefix = "rmtt_";
    }

    // 【发布】ORCA指令 本节点 ->  ORCA
    orca_cmd_pub = nh.advertise<sunray_msgs::orca_cmd>("/sunray_swarm/" + agent_prefix + "/orca_cmd", 1);
    // 【发布】文字提示消息（回传至地面站显示）
    text_info_pub = nh.advertise<std_msgs::String>("/sunray_swarm/text_info", 1);
    // 【订阅】程序触发指令
    agent_name = "/" + agent_prefix + std::to_string(agent_id);
    start_cmd_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm" + agent_name + "/neu_display_goals", 1, start_cmd_cb);
    // 【发布】无人机的目标点（TODO）
    orca_goal_pub = nh.advertise<geometry_msgs::Point>("/sunray_swarm" + agent_name + "/goal_point", 1);

    // 【订阅】订阅地面站目标点
    agent_goal_sub = nh.subscribe<sunray_msgs::orca_cmd>("/sunray_swarm" + agent_name + "/all_goal_point", 1, boost::bind(&agent_goal_cb, _1));
    // 【订阅】无人机orca状态 agent_state_sub
    agent_state_sub = nh.subscribe<sunray_msgs::agent_state>("/sunray_swarm" + agent_name + "/agent_state", 1, boost::bind(&agent_state_cb, _1));

    // 【订阅】程序触发指令
    // takeoff_and_land_sub = nh.subscribe<std_msgs::Bool>("/sunray_swarm" + agent_name + "/takeoff_land", 1, agent_cmd_land_takeoff);

    // 【发布】起飞和降落命令
    agent_cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 1);

    // 创建定时器
    ros::Timer timer_show = nh.createTimer(ros::Duration(1.0), timercb_show);
    // 设置目标点
    setup_show_goals();
    // 初始化队形状态
    formation_state = FORMATION_STATE::INIT;

    // 主循环
    while (ros::ok())
    {
        // // N形队形
        ros::spinOnce();
        if (!pub_goal_once)
        {
            check_and_update_goal(agent_id - 1);
        }
        rate.sleep();
    }
    return 0;
}

// 定时器回调（用于显示目标点）
void timercb_show(const ros::TimerEvent &e)
{
    cout<<"timercb_show"<<endl;
    if(updataPoint[agent_id-1])
        sendNowGoal(agent_id-1);

}


void setup_show_goals()
{
    // 字母 N
    goal_1[0].x = -1.0;
    goal_1[0].y = 1.5; // 顶部左端
    goal_1[1].x = 1.2;
    goal_1[1].y = 1.5; // 底部左端
    goal_1[2].x = 0.1;
    goal_1[2].y = 0.15; // 中部交点
    goal_1[3].x = 1.2;
    goal_1[3].y = -1.3; // 顶部右端
    goal_1[4].x = -1.0;
    goal_1[4].y = -1.3; // 底部右端
    goal_1[5].x = 3.0;
    goal_1[5].y = 4.15; // 中部交点
    goal_1[6].x = 3.2;
    goal_1[6].y = -4.3; // 顶部右端
    goal_1[7].x = -2.0;
    goal_1[7].y = -4.3; // 底部右端

    // 字母 O
    goal_2[0].x = -0.5; // 下部边点
    goal_2[0].y = 0.1;
    goal_2[1].x = -1.5; // 中心点
    goal_2[1].y = 0.5;  // 下部中点
    goal_2[2].x = -0.8;
    goal_2[2].y = 0.0; // 上部边点
    goal_2[3].x = 1.4;
    goal_2[3].y = -0.5; // 下部边点
    goal_2[4].x = 0.1;
    goal_2[4].y = -1.5; // 中心点
    goal_2[5].x = 3.0;
    goal_2[5].y = 4.15; // 中部交点
    goal_2[6].x = 3.2;
    goal_2[6].y = -4.3; // 顶部右端
    goal_2[7].x = -2.0;
    goal_2[7].y = -4.3; // 底部右端
    // 字母 K
    goal_3[0].x = 0.1;
    goal_3[0].y = 0.9; // 上部交点
    goal_3[1].x = 1.2;
    goal_3[1].y = 0.9; // 下部交点
    goal_3[2].x = -0.8;
    goal_3[2].y = 0.9; // 中部竖线
    goal_3[3].x = 1.2;
    goal_3[3].y = -0.9; // 斜线上端
    goal_3[4].x = -0.8;
    goal_3[4].y = -0.9; // 斜线下端
    goal_3[5].x = 3.0;
    goal_3[5].y = 4.15; // 中部交点
    goal_3[6].x = 3.2;
    goal_3[6].y = -4.3; // 顶部右端
    goal_3[7].x = -2.0;
    goal_3[7].y = -4.3; // 底部右端
    // 字母 V
    goal_4[0].x = 0.1;
    goal_4[0].y = 0.6; // 左上点
    goal_4[1].x = 1.1;
    goal_4[1].y = 1.1; // 右上点
    goal_4[2].x = -0.7;
    goal_4[2].y = 0.0; // 底部点
    goal_4[3].x = 1.1;
    goal_4[3].y = -1.1; // 左下点
    goal_4[4].x = 0.1;
    goal_4[4].y = -0.6; // 右下点
    goal_4[5].x = 3.0;
    goal_4[5].y = 4.15; // 中部交点
    goal_4[6].x = 3.2;
    goal_4[6].y = -4.3; // 顶部右端
    goal_4[7].x = -2.0;
    goal_4[7].y = -4.3; // 底部右端

    goal_5[0].x = -1.0;
    goal_5[0].y = 1.5; // 顶部左端
    goal_5[1].x = 1.2;
    goal_5[1].y = 1.5; // 底部左端
    goal_5[2].x = 0.1;
    goal_5[2].y = 0.15; // 中部交点
    goal_5[3].x = 1.2;
    goal_5[3].y = -1.3; // 顶部右端
    goal_5[4].x = -1.0;
    goal_5[4].y = -1.3; // 底部右端
    goal_5[5].x = 3.0;
    goal_5[5].y = 4.15; // 中部交点
    goal_5[6].x = 3.2;
    goal_5[6].y = -4.3; // 顶部右端
    goal_5[7].x = -2.0;
    goal_5[7].y = -4.3; // 底部右端

    // 字母 O
    goal_5[0].x = 0.1;
    goal_5[0].y = 1.5; // 上部中点
    goal_5[1].x = 1.4;
    goal_5[1].y = 0.5; // 下部中点
    goal_5[2].x = -0.8;
    goal_5[2].y = 0.0; // 上部边点
    goal_5[3].x = 1.4;
    goal_5[3].y = -0.5; // 下部边点
    goal_5[4].x = 0.1;
    goal_5[4].y = -1.5; // 中心点
    goal_5[5].x = 3.0;
    goal_5[5].y = 4.15; // 中部交点
    goal_5[6].x = 3.2;
    goal_5[6].y = -4.3; // 顶部右端
    goal_5[7].x = -2.0;
    goal_5[7].y = -4.3; // 底部右端
}
