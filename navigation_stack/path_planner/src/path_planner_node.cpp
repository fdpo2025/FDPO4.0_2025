#include "path_planner_node.h"

PathPlanner::PathPlanner(ros::NodeHandle& nh)
  : nh_(nh)
{
  ROS_INFO("PathPlanner inicializado!");
}

void PathPlanner::run()
{
  ROS_INFO_THROTTLE(1.0, "Rodando planner...");
}
