#include "pure_pursuit.h"
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "pure_pursuit_node");
    ros::NodeHandle nh("~");

    NavigationController pp(nh);

    ROS_INFO("Pure Pursuit node started");

    ros::spin();
    return 0;
}