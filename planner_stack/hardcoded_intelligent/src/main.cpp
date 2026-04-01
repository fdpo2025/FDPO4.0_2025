#include <ros/ros.h>

#include "hardcoded_intelligent_node.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hardcoded_intelligent_node");
  ros::NodeHandle nh("~");

  HardcodedIntelligentNode node(nh);
  ros::spin();
  return 0;
}
