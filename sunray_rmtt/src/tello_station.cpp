#include <ros/ros.h>
#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"


using namespace std;
#define MAX_NUM 10

ros::Publisher rmtt_goal_pub[MAX_NUM];
sunray_msgs::station_cmd station_cmd;
geometry_msgs::Point first_goal[MAX_NUM];
geometry_msgs::Point second_goal[MAX_NUM];
bool is_airborne = false;

int main(int argc, char **argv)
{
	// 针对TELLO无人机的ORCA测试节点
    ros::init(argc, argv, "tello_station");
    ros::NodeHandle nh("~");
	int rmtt_num;
    nh.param<int>("rmtt_num", rmtt_num, 8);

    // 【参数】地理围栏参数（超出围栏自动降落）
    nh.param<double>("first_goal/rmtt1_x", first_goal[0].x, 0.0f);
    nh.param<double>("first_goal/rmtt1_y", first_goal[0].y, 0.0f);
	nh.param<double>("first_goal/rmtt2_x", first_goal[1].x, 0.0f);
    nh.param<double>("first_goal/rmtt2_y", first_goal[1].y, 0.0f);
	nh.param<double>("first_goal/rmtt3_x", first_goal[2].x, 0.0f);
	nh.param<double>("first_goal/rmtt3_y", first_goal[2].y, 0.0f);
	nh.param<double>("first_goal/rmtt4_x", first_goal[3].x, 0.0f);
	nh.param<double>("first_goal/rmtt4_y", first_goal[3].y, 0.0f);
	nh.param<double>("first_goal/rmtt5_x", first_goal[4].x, 0.0f);
	nh.param<double>("first_goal/rmtt5_y", first_goal[4].y, 0.0f);
	nh.param<double>("first_goal/rmtt6_x", first_goal[5].x, 0.0f);
	nh.param<double>("first_goal/rmtt6_y", first_goal[5].y, 0.0f);
	nh.param<double>("first_goal/rmtt7_x", first_goal[6].x, 0.0f);
	nh.param<double>("first_goal/rmtt7_y", first_goal[6].y, 0.0f);
	nh.param<double>("first_goal/rmtt8_x", first_goal[7].x, 0.0f);
	nh.param<double>("first_goal/rmtt8_y", first_goal[7].y, 0.0f);
    nh.param<double>("second_goal/rmtt1_x", second_goal[0].x, 0.0f);
    nh.param<double>("second_goal/rmtt1_y", second_goal[0].y, 0.0f);
	nh.param<double>("second_goal/rmtt2_x", second_goal[1].x, 0.0f);
    nh.param<double>("second_goal/rmtt2_y", second_goal[1].y, 0.0f);
	nh.param<double>("second_goal/rmtt3_x", second_goal[2].x, 0.0f);
	nh.param<double>("second_goal/rmtt3_y", second_goal[2].y, 0.0f);
	nh.param<double>("second_goal/rmtt4_x", second_goal[3].x, 0.0f);
	nh.param<double>("second_goal/rmtt4_y", second_goal[3].y, 0.0f);
	nh.param<double>("second_goal/rmtt5_x", second_goal[4].x, 0.0f);
	nh.param<double>("second_goal/rmtt5_y", second_goal[4].y, 0.0f);
	nh.param<double>("second_goal/rmtt6_x", second_goal[5].x, 0.0f);
	nh.param<double>("second_goal/rmtt6_y", second_goal[5].y, 0.0f);
	nh.param<double>("second_goal/rmtt7_x", second_goal[6].x, 0.0f);
	nh.param<double>("second_goal/rmtt7_y", second_goal[6].y, 0.0f);
	nh.param<double>("second_goal/rmtt8_x", second_goal[7].x, 0.0f);
	nh.param<double>("second_goal/rmtt8_y", second_goal[7].y, 0.0f);


	ros::Publisher station_cmd_pub = nh.advertise<sunray_msgs::station_cmd>("/sunray_rmtt/station_cmd", 1);	

	for(int i = 0; i < rmtt_num; i++) 
	{
		string agent_name = "/rmtt_" + std::to_string(i+1);
		rmtt_goal_pub[i] = nh.advertise<geometry_msgs::Point>("/sunray_rmtt" + agent_name + "/goal_point", 1);
	}

	int start_cmd = 0;
	int scenario_id = 3;
	while(ros::ok())
	{
		cout << "Please choose the station_cmd: 1 for TAKEOFF, 2 for LAND, 3 for HOLD, 4 for ORCA_SETUP, 5 for ORCA_RUN, 6 for RETURN_HOME, 7 for 1st goal, 8 for 2nd goal"<<endl;

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
			case 1:
                station_cmd.mission_state = 1;
				station_cmd.uav_id = 99;
				station_cmd_pub.publish(station_cmd);
			break;

            case 2:
				station_cmd.mission_state = 2;
				station_cmd.uav_id = 99;
				station_cmd_pub.publish(station_cmd);
			break; 

			case 3:
				station_cmd.mission_state = 3;
				station_cmd.uav_id = 99;
				station_cmd_pub.publish(station_cmd);
			break;

			case 4:
			    cout << "Please choose senario id: " << endl;
				cout << " ID : ";
				cin >> scenario_id;

				station_cmd.mission_state = 4;
				station_cmd.scenario_id = scenario_id;
				station_cmd.uav_id = 99;
				station_cmd_pub.publish(station_cmd);
			break;

			case 5:
				station_cmd.mission_state = 5;
				station_cmd.uav_id = 99;
				station_cmd_pub.publish(station_cmd);
			break;

			case 6:
				station_cmd.mission_state = 6;
				station_cmd.uav_id = 99;
				station_cmd_pub.publish(station_cmd);
			break;

			case 7:
				for(int i = 0; i < rmtt_num; i++) 
				{
					rmtt_goal_pub[i].publish(first_goal[i]);
				}
			break;

			case 8:
				for(int i = 0; i < rmtt_num; i++) 
				{
					rmtt_goal_pub[i].publish(second_goal[i]);
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
