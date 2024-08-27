#include <ros/ros.h>
#include <signal.h>
#include <vector>
#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"


using namespace std;

ros::Publisher cmd_pub; // 发布控制命令
vector<geometry_msgs::Point> waypoints; // 航点列表
int agent_type; // Agent type

void mySigintHandler(int sig) {
    ROS_INFO("Shutting down agent waypoint control node...");
    ros::shutdown();
}

void navigateToWaypoints() {
    sunray_msgs::agent_cmd cmd;
    cmd.agent_id = 1; // 根据代理类型设置的ID
    cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;

    for (auto& waypoint : waypoints) {
        cmd.desired_pos = waypoint;
        cmd_pub.publish(cmd);
        cout << BLUE << "Agent navigating to waypoint: x=" << waypoint.x << " y=" << waypoint.y << " z=" << waypoint.z << endl;
        ros::Duration(6.0).sleep(); // 等待10秒以模拟到达该点
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "agent_waypoint_control");
    ros::NodeHandle nh;

    signal(SIGINT, mySigintHandler); // 设置信号处理函数
    nh.param<int>("agent_type", agent_type, 0); // 默认为1 (Tianbot)
    string agent_prefix;

    switch(agent_type) {
        case sunray_msgs::agent_state::RMTT:
            agent_prefix = "rmtt_";
            break;
        case sunray_msgs::agent_state::TIANBOT:
            agent_prefix = "tianbot_";
            break;
        case sunray_msgs::agent_state::WHEELTEC:
            agent_prefix = "wheeltec_";
            break;
        case sunray_msgs::agent_state::SIKONG:
            agent_prefix = "sikong_";
            break;
        default:
            agent_prefix = "unknown_";
            break;
    }

    cmd_pub = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm/" + agent_prefix + "1/agent_cmd", 1);

    float x, y, z;
    char choice;

    while (ros::ok()) {
        cout << GREEN << "Enter waypoint position (x, y" << ((agent_type == sunray_msgs::agent_state::TIANBOT || agent_type == sunray_msgs::agent_state::WHEELTEC) ? "" : ", z") << "): ";
        if (!(cin >> x >> y)) {
            cin.clear();
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
            cout << RED << "Invalid x or y coordinate input." << endl;
            continue;
        }
        if (agent_type == sunray_msgs::agent_state::RMTT || agent_type == sunray_msgs::agent_state::SIKONG) {
            if (!(cin >> z)) {
                cin.clear();
                cin.ignore(numeric_limits<streamsize>::max(), '\n');
                cout << RED << "Invalid z coordinate, setting z to default 1.0" << endl;
                z = 1.0;  // 默认高度为无人机
            }
        } else {
            z = 0.0;  // 无人车默认高度
        }

        geometry_msgs::Point point;
        point.x = x;
        point.y = y;
        point.z = z;
        waypoints.push_back(point);

        cout << GREEN << "Press 'n' to add another waypoint, or 's' to start navigating: ";
        cin >> choice;
        if (choice == 's' || choice == 'S') {
            navigateToWaypoints();
            waypoints.clear();
        }
        else if(choice != 'n' && choice != 'N') {
            cin.clear();
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
            cout << RED << "Invalid input, exiting program." << endl;
        }
    }

    return 0;
}