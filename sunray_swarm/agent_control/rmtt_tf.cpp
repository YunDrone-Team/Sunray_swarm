#include <ros/ros.h>
#include <iostream>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Quaternion.h>
#include "printf_utils.h"

using namespace std;
class MapTFListener
{
public:
    MapTFListener()
    {
        // 初始化 ROS 节点句柄
        nh_ = ros::NodeHandle("~");
        
        int agent_id;
        // 【参数】智能体编号
        nh_.param<int>("agent_id", agent_id, 1);
        string agent_name = "rmtt_"+std::to_string(agent_id);
        

        // 订阅程序位置的发布器
        map_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/sunray_swarm/"+ agent_name +"/map_pose", 1);

        // 初始化 TF 监听器
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 设定地图框架和程序框架的名称
        map_frame_ = "sunray_swarm/"+ agent_name +"/map";        // 假设地图框架是 "map"
        program_frame_ = "sunray_swarm/"+ agent_name +"/base_link"; // 假设程序框架是 "base_link"
    }

    // 监听并获取 TF 变换
    void listenToTF()
    {
        ros::Rate rate(5); // 10 Hz
        while (ros::ok())
        {
            try
            {
                // 获取从地图到程序框架的变换
                geometry_msgs::TransformStamped transform_stamped =
                    tf_buffer_->lookupTransform(map_frame_, program_frame_, ros::Time(0));

                // 获取位移和平移
                geometry_msgs::PoseStamped pose;
                pose.header.stamp = ros::Time::now();
                pose.header.frame_id = map_frame_;

                pose.pose.position.x = transform_stamped.transform.translation.x;
                pose.pose.position.y = transform_stamped.transform.translation.y;
                pose.pose.position.z = transform_stamped.transform.translation.z;

                // 获取旋转（四元数）
                pose.pose.orientation = transform_stamped.transform.rotation;

                // 将 pose 发布到 /program_pose 话题
                map_pose_pub.publish(pose);
            }
            catch (const tf2::TransformException& ex)
            {
                ROS_WARN("TF Error: %s", ex.what());
                cout << GREEN << ">>>>>>>>>>>>>>>>>>> RMTT_CONTROL Parameters <<<<<<<<<<<<<<<<" << TAIL << endl;
            }

            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher map_pose_pub;         // 发布 Pose 信息
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::string map_frame_;          // 地图框架名称
    std::string program_frame_;      // 程序框架名称
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_tf_listener");
    MapTFListener listener;
    listener.listenToTF();

    return 0;
}