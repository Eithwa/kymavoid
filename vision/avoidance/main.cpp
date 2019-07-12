#include "avoidance.h"

void SigintHandler(int sig)
{
    ROS_INFO("shutting down!");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "avoidance");
    ros::NodeHandle h_node;
    signal(SIGINT, SigintHandler);
    ros::Rate loop_rate(30); //program speed limit
    Vision interface;
    fflush(stdout); //更新文字緩衝區
    while (ros::ok())
    {
        interface.draw_interface();
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Node exit");
    printf("Process exit\n");
    return 0;
}
