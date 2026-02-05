#include "plan_handler_node.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plan_handler_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    PlanHandlerNode node(nh);
    
    ROS_INFO("PlanHandlerNode started. Spinning...");
    ros::spin();
    
    return 0;
}


