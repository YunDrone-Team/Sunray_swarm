#include <ros/ros.h>
#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"

using namespace std;
#define MAX_NUM 20

ros::Publisher agent_cmd_pub[MAX_NUM];
sunray_msgs::agent_cmd agent_cmd;
sunray_msgs::orca_cmd orca_cmd;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "agent_station");
    ros::NodeHandle nh("~");
	int agent_num;
	int agent_id;
    int agent_type;
    nh.param<int>("agent_type", agent_type, 2);
    nh.param<int>("agent_num", agent_num, 8);
    nh.param<int>("agent_id", agent_id, 1);

	string agent_prefix;	
    if(agent_type == sunray_msgs::agent_state::RMTT)
    {
        agent_prefix = "/rmtt_";
    }else if(agent_type == sunray_msgs::agent_state::TIANBOT)
    {
        agent_prefix = "/tianbot_";
    }else if(agent_type == sunray_msgs::agent_state::WHEELTEC)
    {
        agent_prefix = "/wheeltec_";
    }else if(agent_type == sunray_msgs::agent_state::SIKONG)
    {
        agent_prefix = "/sikong_";
    }else
    {
        agent_prefix = "/unkonown_";
    }

    string agent_name = agent_prefix + std::to_string(agent_id);

	ros::Publisher orca_cmd_pub = nh.advertise<sunray_msgs::orca_cmd>("/sunray_swarm" + agent_prefix + "/orca_cmd", 1);	

    for(int i = 0; i < agent_num; i++) 
    {
        agent_name = agent_prefix + std::to_string(i+1);
		agent_cmd_pub[i] = nh.advertise<sunray_msgs::agent_cmd>("/sunray_swarm" + agent_name + "/agent_cmd", 1);
    }


	agent_cmd.control_state = sunray_msgs::agent_cmd::INIT;
	agent_cmd.agent_id = agent_id;
	agent_cmd.desired_pos.x = 0.0;
	agent_cmd.desired_pos.y = 0.0;
	agent_cmd.desired_pos.z = 0.0;
	agent_cmd.desired_yaw = 0.0;
	agent_cmd.desired_vel.linear.x = 0.0;
	agent_cmd.desired_vel.linear.y = 0.0;
	agent_cmd.desired_vel.linear.z = 0.0;
	agent_cmd.desired_vel.angular.z = 0.0;

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
				agent_cmd.agent_id = 1;
				agent_cmd_pub[0].publish(agent_cmd);
				break;
			
			case 1:
                agent_cmd.control_state = sunray_msgs::agent_cmd::HOLD;
				agent_cmd.agent_id = 1;
				agent_cmd_pub[0].publish(agent_cmd);
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
				agent_cmd.agent_id = 1;
				agent_cmd_pub[0].publish(agent_cmd);
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
				agent_cmd.agent_id = 1;
				agent_cmd_pub[0].publish(agent_cmd);
				cout << GREEN << "VEL_CONTROL_BODY, desired vel: [" << agent_cmd.desired_vel.linear.x << "," << agent_cmd.desired_vel.linear.y << "], desired yaw: " << agent_cmd.desired_vel.angular.z / M_PI * 180.0 << TAIL << endl;
				break;

			case 4:
				cout << GREEN << "VEL_CONTROL_ENU, Pls input the desired vel and yaw rate" << TAIL << endl;
				cout << GREEN << "desired vel: --- x [m/s] "  << TAIL << endl;
				cin >> agent_cmd.desired_vel.linear.x;
				cout << GREEN << "desired vel: --- y [m/s]"  << TAIL << endl;
				cin >> agent_cmd.desired_vel.linear.y;
				cout << GREEN << "desired yaw_rate: --- yaw [deg/s]:"  << TAIL << endl;
				cin >> agent_cmd.desired_vel.angular.z;
				agent_cmd.desired_vel.angular.z = agent_cmd.desired_vel.angular.z / 180.0 * M_PI;
				agent_cmd.control_state = sunray_msgs::agent_cmd::VEL_CONTROL_ENU;
				agent_cmd.agent_id = 1;
				agent_cmd_pub[0].publish(agent_cmd);
				cout << GREEN << "VEL_CONTROL_ENU, desired vel: [" << agent_cmd.desired_vel.linear.x << "," << agent_cmd.desired_vel.linear.y << "], desired yaw: " << agent_cmd.desired_vel.angular.z / M_PI * 180.0 << TAIL << endl;
				break;

			case 11:
                agent_cmd.control_state = sunray_msgs::agent_cmd::TAKEOFF;
				agent_cmd.agent_id = 1;
				agent_cmd_pub[0].publish(agent_cmd);
				break;

			case 12:
                agent_cmd.control_state = sunray_msgs::agent_cmd::LAND;
				agent_cmd.agent_id = 1;
				agent_cmd_pub[0].publish(agent_cmd);
				break;

			case 99:
				cout << GREEN << "orca_cmd: 0 for SET_HOME, 1 for RETURN_HOME, 2 for ORCA_SCENARIO_1, 3 for ORCA_SCENARIO_2, 4 for ORCA_SCENARIO_3, 5 for ORCA_SCENARIO_4, 6 for ORCA_SCENARIO_5" << TAIL << endl;
				cin >> start_cmd;
				if(start_cmd == 0)
				{
					orca_cmd.orca_cmd = sunray_msgs::orca_cmd::SET_HOME;
				}else if(start_cmd == 1)
				{
					orca_cmd.orca_cmd = sunray_msgs::orca_cmd::RETURN_HOME;
				}else if(start_cmd == 2)
				{
					orca_cmd.orca_cmd = sunray_msgs::orca_cmd::ORCA_SCENARIO_1;
				}else if(start_cmd == 3)
				{
					orca_cmd.orca_cmd = sunray_msgs::orca_cmd::ORCA_SCENARIO_2;
				}else if(start_cmd == 4)
				{
					orca_cmd.orca_cmd = sunray_msgs::orca_cmd::ORCA_SCENARIO_3;
				}else if(start_cmd == 5)
				{
					orca_cmd.orca_cmd = sunray_msgs::orca_cmd::ORCA_SCENARIO_4;
				}else if(start_cmd == 6)
				{
					orca_cmd.orca_cmd = sunray_msgs::orca_cmd::ORCA_SCENARIO_5;
				}
				else if(start_cmd == 99)
				{
					orca_cmd.orca_cmd = sunray_msgs::orca_cmd::ORCA_RUN;
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
