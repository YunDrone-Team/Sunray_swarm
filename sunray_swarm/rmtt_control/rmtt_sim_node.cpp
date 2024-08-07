#include <ros/ros.h>
#include "printf_utils.h"
#include "ros_msg_utils.h"
#include "rmtt_sim.h"
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rmtt_sim_node");
    ros::NodeHandle nh("~");
    ros::Rate rate(100.0);

    // rmtt仿真器
    RMTT_SIM rmtt_sim;
    rmtt_sim.init(nh);

    bool finished = false;

    sleep(5.0);

    // 主循环
    while (ros::ok())
    {
        // 回调函数
        ros::spinOnce();
        // 主循环函数
        finished = rmtt_sim.mainloop();
        // 休眠
        rate.sleep();
    }

    return 0;
}
