#include <ros/ros.h>
#include "path_planner/path_planner_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_planner_node");
  ros::NodeHandle nh("~");

  PathPlannerNode node(nh);

  ros::spin();
  return 0;
}
