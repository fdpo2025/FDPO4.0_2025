#include "navigation_controller_ss_node.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "navigation_controller_ss_node");
    ros::NodeHandle nh("~");

    NavigationControllerSS controller(nh);
    ROS_INFO("Navigation Controller SS node started");
    
    ros::spin();
    return 0;
}

