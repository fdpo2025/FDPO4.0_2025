#include "path_planner_node.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "path_planner_node");
    ros::NodeHandle nh("~");

    PathPlanner planner(nh);
    ROS_INFO("Path Planner initialization...");

    ros::spin();

    return 0;
}
