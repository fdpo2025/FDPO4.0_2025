#include <ros/ros.h>

#include "custom_planner_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "custom_planner_node");
  ros::NodeHandle nh("~");

  CustomPlannerNode node(nh);
  ros::spin();
  return 0;
}

