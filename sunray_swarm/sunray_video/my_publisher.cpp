#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp>  
#include <cv_bridge/cv_bridge.h>  
#include <sstream>  
#include "printf_utils.h"
#include "math_utils.h"
#include "ros_msg_utils.h"
 
int agent_type;
int agent_id;

int main(int argc, char** argv)  
{  
    ros::init(argc, argv, "image_publisher");  
    ros::NodeHandle nh;  
    // 【参数】智能体类型 智能体类型，默认为0
    nh.param<int>("agent_type", agent_type, 1);
    // 【参数】智能体编号
    nh.param<int>("agent_id", agent_id, 1);
    // Check if video source has been passed as a parameter  
    if(argv[1] == NULL)   
    {  
        ROS_INFO("argv[1]=NULL\n");  
        return 1;  
    }
    int agent_type;  
    // 智能体名称前缀

    // 生成智能体名称
    string agent_name = "/ugv_" + std::to_string(agent_id);


    image_transport::ImageTransport it(nh);  
    image_transport::Publisher pub = it.advertise("/sunray_swarm" + agent_name +"/camera/image", 1, "compressed");  
 

    std::istringstream video_sourceCmd(argv[1]);  
    int video_source;  
    // 检查数据源 
    if(!(video_sourceCmd >> video_source))   
    {  
        ROS_INFO("video_sourceCmd is %d\n",video_source);  
        return 1;  
    }  
 
    cv::VideoCapture cap(video_source);  
    //  检测索引设备  
    if(!cap.isOpened())   
    {  
        ROS_INFO("can not opencv video device\n");  
        return 1;  
    }  
    cv::Mat frame;  
    sensor_msgs::ImagePtr msg;  
 
    ros::Rate loop_rate(5);  
    while (nh.ok()) 
    {  
        cap >> frame;  
        // 检查获取视频流
        if(!frame.empty()) 
        {  
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();  
            pub.publish(msg);  
            //cv::Wait(1);  
    	}  
    }
    
    ros::spinOnce();  
    loop_rate.sleep();  
}  