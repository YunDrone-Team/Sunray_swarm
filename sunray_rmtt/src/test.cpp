#include <ros/ros.h>
#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"

using namespace std;

int main(int argc, char **argv)
{
	// 针对TELLO无人机的ORCA测试节点
    ros::init(argc, argv, "test");
    ros::NodeHandle nh("~");

	int uav_id;
    // 【参数】无人机编号
    nh.param<int>("uav_id", uav_id, 2);
    string agent_name = "/rmtt_" + std::to_string(uav_id);

	ros::Publisher takeoff_pub = nh.advertise<std_msgs::Empty>("/sunray_rmtt" + agent_name + "/takeoff", 1); 
	ros::Publisher land_pub = nh.advertise<std_msgs::Empty>("/sunray_rmtt" + agent_name + "/land", 1);


	int start_cmd = 0;
	std_msgs::Empty takeoff;
	std_msgs::Empty land;
	while(ros::ok())
	{
		cout << "Please choose the station_cmd: 1 for TAKEOFF, 2 for LAND..."<<endl;

		if (!(cin >> start_cmd)) {
			// 清除错误内容并且跳过
			cin.clear(); 
			cin.ignore(numeric_limits<streamsize>::max(), '\n'); 
			cout << "[ERROR] Invalid input, please enter a number." << endl;
			continue;
		}
		switch (start_cmd) 
		{
		case 1:
			takeoff_pub.publish(takeoff); 
		break;
		case 2:
			land_pub.publish(land); 
		break;

			}
		ros::spinOnce();
		ros::Rate(10).sleep();
		sleep(1.0);
	}	

	return 0;
}
