#include "multi_planner_node.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "multi_planner_node");
    ros::NodeHandle nh("~");

    MultiPlannerNode node(nh);

    ros::spin();
    return 0;
}
