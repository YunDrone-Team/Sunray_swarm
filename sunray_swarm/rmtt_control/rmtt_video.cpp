#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

class TelloVideoStream
{
public:
    TelloVideoStream()
        : it_(nh_)
    {
        image_sub_ = it_.subscribe("/tello/image_raw", 1, &TelloVideoStream::imageCallback, this);
        cv::namedWindow("Tello Video Stream");
    }

    ~TelloVideoStream()
    {
        cv::destroyWindow("Tello Video Stream");
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::imshow("Tello Video Stream", cv_ptr->image);
        cv::waitKey(3);
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tello_video_stream");
    TelloVideoStream telloVideoStream;
    ros::spin();
    return 0;
}