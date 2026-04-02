#include "chris_planner_node.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "chris_planner_node");
    ros::NodeHandle nh("~");

    ChrisPlannerNode node(nh);

    ros::spin();
    return 0;
}
