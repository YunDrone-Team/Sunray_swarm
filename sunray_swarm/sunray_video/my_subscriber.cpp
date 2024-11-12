#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp>  
#include <cv_bridge/cv_bridge.h>  
#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"
 
int agent_type;
int agent_id;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)  
{  
	try  
	{  
		cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image); 
		// cv::waitKey(30);  
	}  
	catch (cv_bridge::Exception& e)  
	{  
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());  
	}  
}  
 
int main(int argc, char **argv)  
{  
	ros::init(argc, argv, "image_listener");  
	ros::NodeHandle nh; 
	// 【参数】智能体类型 智能体类型，默认为0
    nh.param<int>("agent_type", agent_type, 1);
    // 【参数】智能体编号
    nh.param<int>("agent_id", agent_id, 1);
	int agent_type;  
    // 智能体名称前缀
    string agent_prefix;
    // 根据智能体类型设置名称前缀

    if (agent_type == sunray_msgs::agent_state::RMTT)
    {
        agent_prefix = "rmtt_";
    }
    else if (agent_type == sunray_msgs::agent_state::UGV)
    {
        agent_prefix = "ugv_";
    }
    else if (agent_type == sunray_msgs::agent_state::SIKONG)
    {
        agent_prefix = "sikong_";
    }
    else
    {
        agent_prefix = "unkonown_";
    }
	// 生成智能体名称
    string agent_name = "/" + agent_prefix + std::to_string(agent_id);
	 
	cv::namedWindow("view");  
	cv::startWindowThread();  
	image_transport::ImageTransport it(nh);  
	image_transport::Subscriber sub = it.subscribe("/sunray_swarm" + agent_name +"/camera/image", 1,imageCallback);  
	ros::spin();  
	cv::destroyWindow("view");  
}  
