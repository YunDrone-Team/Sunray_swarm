#include <ros/ros.h>
#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"

using namespace std;
#define MAX_NUM 20

ros::Publisher ugv_cmd_pub[MAX_NUM];
sunray_msgs::ugv_cmd ugv_cmd;
sunray_msgs::orca_cmd orca_cmd;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ugv_station");
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
		ugv_cmd_pub[i] = nh.advertise<sunray_msgs::ugv_cmd>("/sunray_swarm" + agent_name + "/ugv_cmd", 1);
    }


	ugv_cmd.control_state = 0;
	ugv_cmd.agent_id = agent_id;
	ugv_cmd.desired_pos.x = 0.0;
	ugv_cmd.desired_pos.y = 0.0;
	ugv_cmd.desired_pos.z = 0.0;
	ugv_cmd.desired_yaw = 0.0;
	ugv_cmd.desired_vel.linear.x = 0.0;
	ugv_cmd.desired_vel.linear.y = 0.0;
	ugv_cmd.desired_vel.linear.z = 0.0;
	ugv_cmd.desired_vel.angular.z = 0.0;

	int start_cmd = 0;
	while(ros::ok())
	{
		cout << GREEN << "Please choose the ugv_cmd: 0 for INIT, 1 for HOLD, 2 for POS_CONTROL, 3 for VEL_CONTROL, 4 for ORCA_MODE, 5 for TRACK_MODE, 6 for RETURN_HOME, 7 for ORCA CMD..." << TAIL << endl;

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
                ugv_cmd.control_state = 0;
				ugv_cmd.agent_id = 1;
				ugv_cmd_pub[0].publish(ugv_cmd);
				break;
			
			case 1:
                ugv_cmd.control_state = 1;
				ugv_cmd.agent_id = 1;
				ugv_cmd_pub[0].publish(ugv_cmd);
				break;

            case 2:
				cout << GREEN << "POS_CONTROL, Pls input the desired position and yaw angle" << TAIL << endl;
				cout << GREEN << "desired position: --- x [m] "  << TAIL << endl;
				cin >> ugv_cmd.desired_pos.x;
				cout << GREEN << "desired position: --- y [m]"  << TAIL << endl;
				cin >> ugv_cmd.desired_pos.y;
				cout << GREEN << "desired yaw: --- yaw [deg]:"  << TAIL << endl;
				cin >> ugv_cmd.desired_yaw;
				ugv_cmd.desired_yaw = ugv_cmd.desired_yaw / 180.0 * M_PI;
				ugv_cmd.control_state = 2;
				ugv_cmd.agent_id = 1;
				ugv_cmd_pub[0].publish(ugv_cmd);
				cout << GREEN << "POS_CONTROL, desired pos: [" << ugv_cmd.desired_pos.x << "," << ugv_cmd.desired_pos.y << "], desired yaw: " << ugv_cmd.desired_yaw / M_PI * 180.0 << TAIL << endl;
			break; 

			case 3:
				cout << GREEN << "VEL_CONTROL, Pls input the desired vel and yaw rate" << TAIL << endl;
				cout << GREEN << "desired vel: --- x [m/s] "  << TAIL << endl;
				cin >> ugv_cmd.desired_vel.linear.x;
				cout << GREEN << "desired vel: --- y [m/s]"  << TAIL << endl;
				cin >> ugv_cmd.desired_vel.linear.y;
				cout << GREEN << "desired yaw_rate: --- yaw [deg/s]:"  << TAIL << endl;
				cin >> ugv_cmd.desired_vel.angular.z;
				ugv_cmd.desired_vel.angular.z = ugv_cmd.desired_vel.angular.z / 180.0 * M_PI;
				ugv_cmd.control_state = 3;
				ugv_cmd.agent_id = 1;
				ugv_cmd_pub[0].publish(ugv_cmd);
				cout << GREEN << "VEL_CONTROL, desired vel: [" << ugv_cmd.desired_vel.linear.x << "," << ugv_cmd.desired_vel.linear.y << "], desired yaw: " << ugv_cmd.desired_vel.angular.z / M_PI * 180.0 << TAIL << endl;
				break;

			case 4:
				ugv_cmd.control_state = 4;
				ugv_cmd.agent_id = 1;
				ugv_cmd_pub[0].publish(ugv_cmd);
				break;

			case 5:
				ugv_cmd.control_state = 5;
				ugv_cmd.agent_id = 1;
				ugv_cmd_pub[0].publish(ugv_cmd);
				break;

			case 6:
				ugv_cmd.control_state = 6;
				ugv_cmd.agent_id = 1;
				ugv_cmd_pub[0].publish(ugv_cmd);
				break;

			case 7:
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
					ugv_cmd.control_state = 6;
					for(int i = 0; i < agent_num; i++) 
					{
						ugv_cmd_pub[i].publish(ugv_cmd);
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
					ugv_cmd.control_state = 4;
					for(int i = 0; i < agent_num; i++) 
					{
						ugv_cmd.agent_id = i+1;
						ugv_cmd_pub[i].publish(ugv_cmd);
						sleep(0.1);
					}
				}
				
				break;

			default:
				cout << RED << "[ERROR] wrong input, valid commands are 1, 2, 3, 4, 5, 6 and 7." << TAIL << endl;
				break;
			}
		ros::spinOnce();
		ros::Rate(10).sleep();
	}	

	return 0;
}
