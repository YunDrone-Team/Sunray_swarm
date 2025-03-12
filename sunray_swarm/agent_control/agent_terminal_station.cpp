#include <ros/ros.h>
#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"

using namespace std;
#define MAX_NUM 20

ros::Publisher agent_cmd_pub[MAX_NUM];
ros::Publisher orca_cmd_pub;
sunray_msgs::agent_cmd agent_cmd;
sunray_msgs::orca_cmd orca_cmd;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agent_terminal_station");
    ros::NodeHandle nh("~");
	int agent_num;
	int agent_id;
    int agent_type;
    nh.param<int>("agent_type", agent_type, 0);
    nh.param<int>("agent_num", agent_num, 1);
    nh.param<int>("agent_id", agent_id, 1);

    string agent_prefix;
    if(agent_type == sunray_msgs::agent_state::RMTT)
    {
        agent_prefix = "/rmtt";
    }else if(agent_type == sunray_msgs::agent_state::UGV)
    {
        agent_prefix = "/ugv";
    }

	// 【发布】ORCA控制指令 本节点 -> 智能体控制节点  
	orca_cmd_pub = nh.advertise<sunray_msgs::orca_cmd>("/sunray_swarm" + agent_prefix + "/orca_cmd", 1);	

    string agent_name;
    for(int i = 0; i < agent_num; i++) 
    {
        agent_name = agent_prefix + "_" + std::to_string(i+1);
		// 【发布】智能体控制指令 本节点 -> 智能体控制节点
		agent_cmd_pub[i] = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 1);
    }

	agent_cmd.control_state = sunray_msgs::agent_cmd::INIT;
	agent_cmd.agent_id = 99;
	agent_cmd.cmd_source = "agent_terminal_station";
	agent_cmd.desired_pos.x = 0.0;
	agent_cmd.desired_pos.y = 0.0;
	agent_cmd.desired_pos.z = 0.0;
	agent_cmd.desired_yaw = 0.0;
	agent_cmd.desired_vel.linear.x = 0.0;
	agent_cmd.desired_vel.linear.y = 0.0;
	agent_cmd.desired_vel.linear.z = 0.0;
	agent_cmd.desired_vel.angular.z = 0.0;

    if(agent_type == sunray_msgs::agent_state::RMTT)
    {
        cout << GREEN << "agent_type : RMTT" << TAIL << endl;
    }else if(agent_type == sunray_msgs::agent_state::UGV)
    {
        cout << GREEN << "agent_type : UGV" << TAIL << endl;
    }else
    {
        cout << GREEN << "agent_type : UNKONWN" << TAIL << endl;
    }
    cout << GREEN << "agent_id: " << agent_id << TAIL << endl;
    cout << GREEN << "agent_num: " << agent_num << TAIL << endl;

	int start_cmd = 0;
	while(ros::ok())
	{
		cout << GREEN << "Please choose the agent_cmd: 0 for INIT, 1 for HOLD, 2 for POS_CONTROL, 3 for VEL_CONTROL_BODY, 4 for VEL_CONTROL_ENU, 11 for TAKEOFF(RMTT), 12 for LAND(RMTT), 99 for pub ORCA CMD..." << TAIL << endl;

		if (!(cin >> start_cmd)) 
		{
			// 清除错误内容并且跳过
			cin.clear(); 
			cin.ignore(numeric_limits<streamsize>::max(), '\n'); 
			cout << "[ERROR] Invalid input, please enter a number." << endl;
			continue;
		}
		switch (start_cmd) 
		{
			case 0:
                agent_cmd.control_state = sunray_msgs::agent_cmd::INIT;
				for(int i = 0; i < agent_num; i++) 
				{
					agent_cmd.agent_id = i+1;
					agent_cmd_pub[i].publish(agent_cmd);
				}
				break;
			
			case 1:
                agent_cmd.control_state = sunray_msgs::agent_cmd::HOLD;

				orca_cmd.orca_cmd = sunray_msgs::orca_cmd::ORCA_STOP;
				orca_cmd_pub.publish(orca_cmd);

				for(int i = 0; i < agent_num; i++) 
				{
					agent_cmd.agent_id = i+1;
					agent_cmd_pub[i].publish(agent_cmd);
				}
				break;

            case 2:
				cout << GREEN << "POS_CONTROL, Pls input the desired position and yaw angle" << TAIL << endl;
				cout << GREEN << "desired position: --- x [m] "  << TAIL << endl;
				cin >> agent_cmd.desired_pos.x;
				cout << GREEN << "desired position: --- y [m]"  << TAIL << endl;
				cin >> agent_cmd.desired_pos.y;
				cout << GREEN << "desired yaw: --- yaw [deg]:"  << TAIL << endl;
				cin >> agent_cmd.desired_yaw;
				agent_cmd.desired_yaw = agent_cmd.desired_yaw / 180.0 * M_PI;
				agent_cmd.control_state = sunray_msgs::agent_cmd::POS_CONTROL;
				for(int i = 0; i < agent_num; i++) 
				{
					agent_cmd.agent_id = i+1;
					agent_cmd_pub[i].publish(agent_cmd);
				}
				cout << GREEN << "POS_CONTROL, desired pos: [" << agent_cmd.desired_pos.x << "," << agent_cmd.desired_pos.y << "], desired yaw: " << agent_cmd.desired_yaw / M_PI * 180.0 << TAIL << endl;
			break; 

			case 3:
				cout << GREEN << "VEL_CONTROL_BODY, Pls input the desired vel and yaw rate" << TAIL << endl;
				cout << GREEN << "desired vel: --- x [m/s] "  << TAIL << endl;
				cin >> agent_cmd.desired_vel.linear.x;
				cout << GREEN << "desired vel: --- y [m/s]"  << TAIL << endl;
				cin >> agent_cmd.desired_vel.linear.y;
				cout << GREEN << "desired yaw_rate: --- yaw [deg/s]:"  << TAIL << endl;
				cin >> agent_cmd.desired_vel.angular.z;
				agent_cmd.desired_vel.angular.z = agent_cmd.desired_vel.angular.z / 180.0 * M_PI;
				agent_cmd.control_state = sunray_msgs::agent_cmd::VEL_CONTROL_BODY;
				for(int i = 0; i < agent_num; i++) 
				{
					agent_cmd.agent_id = i+1;
					agent_cmd_pub[i].publish(agent_cmd);
				}
				cout << GREEN << "VEL_CONTROL_BODY, desired vel: [" << agent_cmd.desired_vel.linear.x << "," << agent_cmd.desired_vel.linear.y << "], desired yaw: " << agent_cmd.desired_vel.angular.z / M_PI * 180.0 << TAIL << endl;
				break;

			case 4:
				cout << GREEN << "VEL_CONTROL_ENU, Pls input the desired vel and yaw" << TAIL << endl;
				cout << GREEN << "desired vel: --- x [m/s] "  << TAIL << endl;
				cin >> agent_cmd.desired_vel.linear.x;
				cout << GREEN << "desired vel: --- y [m/s]"  << TAIL << endl;
				cin >> agent_cmd.desired_vel.linear.y;
				cout << GREEN << "desired yaw: --- yaw [deg]:"  << TAIL << endl;
				cin >> agent_cmd.desired_yaw;
				agent_cmd.desired_yaw = agent_cmd.desired_yaw / 180.0 * M_PI;
				agent_cmd.control_state = sunray_msgs::agent_cmd::VEL_CONTROL_ENU;
				for(int i = 0; i < agent_num; i++) 
				{
					agent_cmd.agent_id = i+1;
					agent_cmd_pub[i].publish(agent_cmd);
				}
				cout << GREEN << "VEL_CONTROL_ENU, desired vel: [" << agent_cmd.desired_vel.linear.x << "," << agent_cmd.desired_vel.linear.y << "], desired yaw: " << agent_cmd.desired_yaw / M_PI * 180.0 << TAIL << endl;
				break;

			case 11:
                agent_cmd.control_state = sunray_msgs::agent_cmd::TAKEOFF;
				for(int i = 0; i < agent_num; i++) 
				{
					agent_cmd.agent_id = i+1;
					agent_cmd_pub[i].publish(agent_cmd);
				}
				break;

			case 12:
                agent_cmd.control_state = sunray_msgs::agent_cmd::LAND;
				for(int i = 0; i < agent_num; i++) 
				{
					agent_cmd.agent_id = i+1;
					agent_cmd_pub[i].publish(agent_cmd);
				}
				break;

			case 99:
				cout << GREEN << "orca_cmd: 0 for SET_HOME, 1 for RETURN_HOME, 3 for ORCA_STOP, 4 for ORCA_RESTART, 11 for ORCA_SCENARIO_1, 12 for ORCA_SCENARIO_2, 13 for ORCA_SCENARIO_3, 14 for ORCA_SCENARIO_4, 15 for ORCA_SCENARIO_5" << TAIL << endl;
				cin >> start_cmd;
				if(start_cmd == 0)
				{
					orca_cmd.orca_cmd = sunray_msgs::orca_cmd::SET_HOME;
				}else if(start_cmd == 1)
				{
					orca_cmd.orca_cmd = sunray_msgs::orca_cmd::RETURN_HOME;
				}else if(start_cmd == 3)
				{
					orca_cmd.orca_cmd = sunray_msgs::orca_cmd::ORCA_STOP;
				}else if(start_cmd == 4)
				{
					orca_cmd.orca_cmd = sunray_msgs::orca_cmd::ORCA_RESTART;
				}else if(start_cmd == 11)
				{
					orca_cmd.orca_cmd = sunray_msgs::orca_cmd::ORCA_SCENARIO_1;
				}else if(start_cmd == 12)
				{
					orca_cmd.orca_cmd = sunray_msgs::orca_cmd::ORCA_SCENARIO_2;
				}else if(start_cmd == 13)
				{
					orca_cmd.orca_cmd = sunray_msgs::orca_cmd::ORCA_SCENARIO_3;
				}else if(start_cmd == 14)
				{
					orca_cmd.orca_cmd = sunray_msgs::orca_cmd::ORCA_SCENARIO_4;
				}else if(start_cmd == 15)
				{
					orca_cmd.orca_cmd = sunray_msgs::orca_cmd::ORCA_SCENARIO_5;
				}
				orca_cmd_pub.publish(orca_cmd);

				break;

			default:
				cout << RED << "[ERROR] wrong input." << TAIL << endl;
				break;
			}
		ros::spinOnce();
		ros::Rate(10).sleep();
	}	

	return 0;
}
