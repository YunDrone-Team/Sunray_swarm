#include <ros/ros.h>
#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"

using namespace std;
#define MAX_NUM 20

ros::Publisher rmtt_cmd_pub[MAX_NUM];
sunray_msgs::rmtt_cmd rmtt_cmd;
sunray_msgs::orca_cmd orca_cmd;
bool is_airborne = false;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rmtt_station");
    ros::NodeHandle nh("~");
	int agent_num;
	int agent_id;
    int agent_type;
    nh.param<int>("agent_type", agent_type, 1);
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

	ros::Publisher orca_cmd_pub = nh.advertise<sunray_msgs::orca_cmd>("/sunray_swarm/orca_cmd", 1);	

	for(int i = 0; i < agent_num; i++) 
	{
        agent_name = agent_prefix + std::to_string(i+1);
		rmtt_cmd_pub[i] = nh.advertise<sunray_msgs::rmtt_cmd>("/sunray_swarm" + agent_name + "/rmtt_cmd", 1);
	}

	rmtt_cmd.control_state = 0;
	rmtt_cmd.agent_id = agent_id;
	rmtt_cmd.desired_pos.x = 0.0;
	rmtt_cmd.desired_pos.y = 0.0;
	rmtt_cmd.desired_pos.z = 0.0;
	rmtt_cmd.desired_yaw = 0.0;
	rmtt_cmd.desired_vel.linear.x = 0.0;
	rmtt_cmd.desired_vel.linear.y = 0.0;
	rmtt_cmd.desired_vel.linear.z = 0.0;
	rmtt_cmd.desired_vel.angular.z = 0.0;


	int start_cmd = 0;
	while(ros::ok())
	{
		cout << GREEN << "Please choose the rmtt_cmd: 0 for INIT, 1 for TAKEOFF, 2 for LAND, 3 for HOLD, 4 for POS_CONTROL, 5 for VEL_CONTROL, 6 for ORCA_MODE, 7 for TRACK_MODE, 8 for RETURN_HOME, 9 for ORCA CMD..." << TAIL << endl;

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
                rmtt_cmd.control_state = 0;
				rmtt_cmd.agent_id = 1;
				rmtt_cmd_pub[0].publish(rmtt_cmd);
			break;

			case 1:
                rmtt_cmd.control_state = 1;
				rmtt_cmd.agent_id = 1;
				rmtt_cmd_pub[0].publish(rmtt_cmd);
			break;

            case 2:
				rmtt_cmd.control_state = 2;
				rmtt_cmd.agent_id = 1;
				rmtt_cmd_pub[0].publish(rmtt_cmd);
			break; 

			case 3:
				rmtt_cmd.control_state = 3;
				rmtt_cmd.agent_id = 1;
				rmtt_cmd_pub[0].publish(rmtt_cmd);
			break;

			case 4:
				cout << GREEN << "POS_CONTROL, Pls input the desired position and yaw angle" << TAIL << endl;
				cout << GREEN << "desired position: --- x [m] "  << TAIL << endl;
				cin >> rmtt_cmd.desired_pos.x;
				cout << GREEN << "desired position: --- y [m]"  << TAIL << endl;
				cin >> rmtt_cmd.desired_pos.y;
				cout << GREEN << "desired position: --- z [m]"  << TAIL << endl;
				cin >> rmtt_cmd.desired_pos.z;
				cout << GREEN << "desired yaw: --- yaw [deg]:"  << TAIL << endl;
				cin >> rmtt_cmd.desired_yaw;
				rmtt_cmd.desired_yaw = rmtt_cmd.desired_yaw / 180.0 * M_PI;
				rmtt_cmd.control_state = 4;
				rmtt_cmd.agent_id = 1;
				rmtt_cmd_pub[0].publish(rmtt_cmd);
				cout << GREEN << "POS_CONTROL, desired pos: [" << rmtt_cmd.desired_pos.x << "," << rmtt_cmd.desired_pos.y << "," << rmtt_cmd.desired_pos.z << "], desired yaw: " << rmtt_cmd.desired_yaw / M_PI * 180.0 << TAIL << endl;
			break; 

			case 5:
				cout << GREEN << "VEL_CONTROL, Pls input the desired vel and yaw rate" << TAIL << endl;
				cout << GREEN << "desired vel: --- x [m/s] "  << TAIL << endl;
				cin >> rmtt_cmd.desired_vel.linear.x;
				cout << GREEN << "desired vel: --- y [m/s]"  << TAIL << endl;
				cin >> rmtt_cmd.desired_vel.linear.y;
				cout << GREEN << "desired yaw_rate: --- yaw [deg/s]:"  << TAIL << endl;
				cin >> rmtt_cmd.desired_vel.angular.z;
				rmtt_cmd.desired_vel.angular.z = rmtt_cmd.desired_vel.angular.z / 180.0 * M_PI;
				rmtt_cmd.control_state = 5;
				rmtt_cmd.agent_id = 1;
				rmtt_cmd_pub[0].publish(rmtt_cmd);
				cout << GREEN << "VEL_CONTROL, desired vel: [" << rmtt_cmd.desired_vel.linear.x << "," << rmtt_cmd.desired_vel.linear.y << "], desired yaw: " << rmtt_cmd.desired_vel.angular.z / M_PI * 180.0 << TAIL << endl;
				break;

			case 6:
				rmtt_cmd.control_state = 6;
				rmtt_cmd.agent_id = 1;
				rmtt_cmd_pub[0].publish(rmtt_cmd);
				break;

			case 7:
				rmtt_cmd.control_state = 7;
				rmtt_cmd.agent_id = 1;
				rmtt_cmd_pub[0].publish(rmtt_cmd);
				break;

			case 8:
				rmtt_cmd.control_state = 8;
				rmtt_cmd.agent_id = 1;
				rmtt_cmd_pub[0].publish(rmtt_cmd);
				break;

			case 9:
				cout << GREEN << "orca_cmd: 0 for SET_HOME, 1 for RETURN_HOME, 2 for ORCA_SETUP, 3 for start agent ORCA" << TAIL << endl;
				cin >> start_cmd;
				if(start_cmd == 0)
				{
					orca_cmd.orca_cmd = 0;
					orca_cmd_pub.publish(orca_cmd);
				}else if(start_cmd == 1)
				{
					orca_cmd.orca_cmd = 1;
					orca_cmd_pub.publish(orca_cmd);
					rmtt_cmd.control_state = 6;
					for(int i = 0; i < agent_num; i++) 
					{
						rmtt_cmd_pub[i].publish(rmtt_cmd);
						sleep(0.1);
					}
				}else if(start_cmd == 2)
				{
					cout << GREEN << "choose scenario_id: 1-5 "  << TAIL << endl;
					int scenario_id;
					cin >> scenario_id;
					orca_cmd.orca_cmd = 2;
					orca_cmd.scenario_id = scenario_id;
					orca_cmd_pub.publish(orca_cmd);
				}else if(start_cmd == 3)
				{
					rmtt_cmd.control_state = 6;
					for(int i = 0; i < agent_num; i++) 
					{
						rmtt_cmd.agent_id = i+1;
						rmtt_cmd_pub[i].publish(rmtt_cmd);
						sleep(0.1);
					}
				}
				
				break;

			default:
				cout << "[ERROR] wrong input, valid commands are 1, 2, 3, 4, 5, 6 and 7." << endl;
			break;
			}
		ros::spinOnce();
		ros::Rate(10).sleep();
	}	

	return 0;
}
