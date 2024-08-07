/*
 * @Author: Yuhua.Qi fatmoonqyp@126.com
 * @Date: 2024-07-11 22:29:25
 * @LastEditors: Yuhua.Qi fatmoonqyp@126.com
 * @LastEditTime: 2024-07-11 23:20:54
 * @FilePath: /Prometheus/home/amov/sunray_rmtt/sunray_rmtt/sim/rmtt_sim_node.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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
