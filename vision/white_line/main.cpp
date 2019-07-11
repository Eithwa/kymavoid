#include "white_line.h"

void SigintHandler(int sig)
{
    ROS_INFO("shutting down!");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization_white_line");
    ros::NodeHandle h_node;
    signal(SIGINT, SigintHandler);
    //Vision cam(VISION_TOPIC);
    Vision cam("/camera/image_raw");
    ros::spin();
    ROS_INFO("Node exit");
    printf("Process exit\n");
    return 0;
}
