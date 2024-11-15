#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <opencv2/highgui/highgui.hpp>  
#include <cv_bridge/cv_bridge.h>  
#include <sstream> // for converting the command line parameter to integer  
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
    image_transport::Publisher pub = it.advertise("/sunray_swarm" + agent_name +"/camera/image", 1);  
 
    // Convert the passed as command line parameter index for the video device to an integer  
    std::istringstream video_sourceCmd(argv[1]);  
    int video_source;  
    // Check if it is indeed a number  
    if(!(video_sourceCmd >> video_source))   
    {  
        ROS_INFO("video_sourceCmd is %d\n",video_source);  
        return 1;  
    }  
 
    cv::VideoCapture cap(video_source);  
    // Check if video device can be opened with the given index  
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
        // Check if grabbed frame is actually full with some content  
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


// #include "rclcpp/rclcpp.hpp"
// #include "sensor_msgs/msg/compressed_image.hpp"
// #include "opencv2/opencv.hpp"
// #include "cv_bridge/cv_bridge.h"
 
// using namespace std::chrono_literals;
 
// class CameraPublisher : public rclcpp::Node {
// public:
//     CameraPublisher()
//         : Node("camera_publisher"), count_(0) {
//         publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("compressed_image", 10);        
//         timer_ = this->create_wall_timer(100ms, std::bind(&CameraPublisher::publishImage, this));
//         cap_ = cv::VideoCapture(0); // Open default camera
 
//         printf("record compressed image!\n");
 
//         if (!cap_.isOpened()) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
//         }
//     }
 
// private:
//     void publishImage() {
//         cv::Mat frame;
//         cap_ >> frame; // Capture a frame from the camera
 
//         if (frame.empty()) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to capture frame");
//             return;
//         }
//         cv::Mat resized_frame;
//         cv::resize(frame, resized_frame, cv::Size(640, 480), cv::INTER_LINEAR);
 
 
 
//         std::vector<uchar> buf;
//         cv::imencode(".jpg", resized_frame, buf, {cv::IMWRITE_JPEG_QUALITY, 80}); // Adjust JPEG quality (0-100 scale)
 
//         sensor_msgs::msg::CompressedImage msg;
//         msg.format = "jpeg";
//         msg.data = buf;
 
//         publisher_->publish(msg);
 
 
//         count_++;
//         printf("record compressed image: %d\r", count_);
//     }
 
//     rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
//     rclcpp::TimerBase::SharedPtr timer_;
 
//     cv::VideoCapture cap_;
//     int count_;
// };
 
// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<CameraPublisher>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }